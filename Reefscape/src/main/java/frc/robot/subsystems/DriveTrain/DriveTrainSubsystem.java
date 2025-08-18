// File: src/main/java/frc/robot/subsystems/DriveTrain/DriveTrainSubsystem.java
package frc.robot.subsystems.DriveTrain;

import java.util.Arrays;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.slsh.IDeer9427.lib.Vision.LimelightHelpers;
import com.slsh.IDeer9427.lib.Vision.ReefFieldPose;
import com.slsh.IDeer9427.lib.Vision.ReefFieldPose.ReefPoint;
import com.slsh.IDeer9427.lib.data.IDearLog;
import com.slsh.IDeer9427.lib.data.IDearLog.FieldType;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveTrain.SwerveConstants.DriveConstants;
import frc.robot.util.Phoenix6Util;

public class DriveTrainSubsystem extends SubsystemBase {
  private final MAXSwerveModule m_frontLeft =
      new MAXSwerveModule(
          DriveConstants.kFrontLeftDrivingCanId,
          DriveConstants.kFrontLeftTurningCanId,
          DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight =
      new MAXSwerveModule(
          DriveConstants.kFrontRightDrivingCanId,
          DriveConstants.kFrontRightTurningCanId,
          DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft =
      new MAXSwerveModule(
          DriveConstants.kRearLeftDrivingCanId,
          DriveConstants.kRearLeftTurningCanId,
          DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight =
      new MAXSwerveModule(
          DriveConstants.kRearRightDrivingCanId,
          DriveConstants.kRearRightTurningCanId,
          DriveConstants.kBackRightChassisAngularOffset);

  private final Pigeon2 m_gyro;

  private SwerveDriveOdometry m_odometry;
  private SwerveDrivePoseEstimator m_poseEstimator;
  public static final Field2d m_field = new Field2d();

  SwerveModuleState[] m_desireState;

  private static final String LL_LEFT = "limelight-left";
  private static final String LL_RIGHT = "limelight-right";
  private static final double LL_MIN_TA = 0.57;

  private final Supplier<LimelightHelpers.PoseEstimate> leftLL =
      () -> LimelightHelpers.getBotPoseEstimate_wpiBlue(LL_LEFT);
  private final Supplier<LimelightHelpers.PoseEstimate> rightLL =
      () -> LimelightHelpers.getBotPoseEstimate_wpiBlue(LL_RIGHT);

  StructArrayPublisher<SwerveModuleState> CurrentPublisher;
  StructArrayPublisher<SwerveModuleState> TargetPublisher;
  StructPublisher<Pose2d> odometerPublisher;

  public DriveTrainSubsystem() {
    CurrentPublisher =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("CurrentState", SwerveModuleState.struct)
            .publish();
    TargetPublisher =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("TargetState", SwerveModuleState.struct)
            .publish();
    odometerPublisher =
        NetworkTableInstance.getDefault().getStructTopic("odometer", Pose2d.struct).publish();

    m_gyro = new Pigeon2(16);
    Phoenix6Util.checkErrorAndRetry(
        () -> m_gyro.getConfigurator().apply(new Pigeon2Configuration()));

    m_odometry =
        new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics, getRotation2d(), getModulePositions());

    m_poseEstimator =
        new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            getRotation2d(),
            getModulePositions(),
            new Pose2d(),
            /* stateStdDevs */ VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            /* visionStdDevs */ VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(5)));

    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    SmartDashboard.putData("Field", m_field);

    zeroHeading();
  }

  public static double readShortSidePx(String table, int imgW, int imgH) {
    // t2d:
    // [valid,count,lat,cl,tx,ty,txnc,tync,ta,tid,clsIdxDet,clsIdxCls,longPx,shortPx,hExt,vExt,skew]
    double[] t2d = LimelightHelpers.getT2DArray(table);
    if (t2d.length == 17 && t2d[0] == 1.0 && t2d[13] > 0.0) return t2d[13];

    // rawdetections: 每筆 12 值 = classId, txnc, tync, ta, c0x,c0y,c1x,c1y,c2x,c2y,c3x,c3y
    LimelightHelpers.RawDetection[] dets = LimelightHelpers.getRawDetections(table);
    if (dets.length > 0) {
      var d = dets[0];
      double[] xs = {d.corner0_X, d.corner1_X, d.corner2_X, d.corner3_X};
      double[] ys = {d.corner0_Y, d.corner1_Y, d.corner2_Y, d.corner3_Y};
      double minx = Arrays.stream(xs).min().orElse(0);
      double maxx = Arrays.stream(xs).max().orElse(0);
      double miny = Arrays.stream(ys).min().orElse(0);
      double maxy = Arrays.stream(ys).max().orElse(0);
      // -1..1 正規化 → 像素
      double wPx = (maxx - minx) * 0.5 * imgW;
      double hPx = (maxy - miny) * 0.5 * imgH;
      double spx = Math.min(wPx, hPx);
      if (spx > 0) return spx;
    }
    return 0.0;
  }

  @Override
  public void periodic() {
    double shortPx = readShortSidePx("limelight-back", 640, 480);
    SmartDashboard.putNumber("LL/shortPx", shortPx);
    SmartDashboard.putNumber("LL/t2d_len", LimelightHelpers.getT2DArray("limelight-back").length);
    SmartDashboard.putNumber("LL/targetCount", LimelightHelpers.getTargetCount("limelight-back"));

    m_odometry.update(getRotation2d(), getModulePositions());
    m_poseEstimator.update(getRotation2d(), getModulePositions());
    CurrentPublisher.set(getModuleStates());
    TargetPublisher.set(m_desireState);
    odometerPublisher.set(getOdomPose());

    try {
      boolean leftTV = LimelightHelpers.getTV(LL_LEFT);
      boolean rightTV = LimelightHelpers.getTV(LL_RIGHT);
      double leftTA = LimelightHelpers.getTA(LL_LEFT);
      double rightTA = LimelightHelpers.getTA(LL_RIGHT);

      if (leftTV && leftTA > LL_MIN_TA) {
        var est = leftLL.get();
        m_poseEstimator.addVisionMeasurement(est.pose, est.timestampSeconds);
        m_odometry.resetPosition(getRotation2d(), getModulePositions(), est.pose);
      } else if (!leftTV && rightTV && rightTA > LL_MIN_TA) {
        var est = rightLL.get();
        m_poseEstimator.addVisionMeasurement(est.pose, est.timestampSeconds);
        m_odometry.resetPosition(getRotation2d(), getModulePositions(), est.pose);
      } else if (leftTV && rightTV) {
        if (leftTA >= rightTA && leftTA > LL_MIN_TA) {
          var est = leftLL.get();
          m_poseEstimator.addVisionMeasurement(est.pose, est.timestampSeconds);
          m_odometry.resetPosition(getRotation2d(), getModulePositions(), est.pose);
        } else if (rightTA > leftTA && rightTA > LL_MIN_TA) {
          var est = rightLL.get();
          m_poseEstimator.addVisionMeasurement(est.pose, est.timestampSeconds);
          m_odometry.resetPosition(getRotation2d(), getModulePositions(), est.pose);
        }
      }
    } catch (Exception e) {
      DriverStation.reportError("Vision update error: " + e.getMessage(), false);
    }

    m_field.setRobotPose(getFusedPose());

    IDearLog.getInstance().addField("PoseX", () -> getFusedPose().getX(), FieldType.NON_CAN);
    IDearLog.getInstance().addField("PoseY", () -> getFusedPose().getY(), FieldType.NON_CAN);
    IDearLog.getInstance().addField("HeadingDeg", () -> getHeadingDegrees(), FieldType.NON_CAN);

    IDearLog.getInstance()
        .addField(
            "AX", () -> ReefFieldPose.getReefPose(ReefPoint.A, 0.30).getX(), FieldType.NON_CAN);
    IDearLog.getInstance()
        .addField(
            "AY", () -> ReefFieldPose.getReefPose(ReefPoint.A, 0.30).getY(), FieldType.NON_CAN);
    IDearLog.getInstance()
        .addField(
            "AR",
            () -> ReefFieldPose.getReefPose(ReefPoint.A, 0.30).getRotation().getDegrees(),
            FieldType.NON_CAN);
  }

  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    };
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState()
    };
  }

  private double getHeadingDegrees() {
    double yaw = m_gyro.getYaw().getValueAsDouble(); // deg
    double sign = DriveConstants.kGyroReversed ? -1.0 : 1.0;
    return Math.IEEEremainder(sign * yaw, 360.0);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public Pose2d getOdomPose() {
    return m_odometry.getPoseMeters();
  }

  public Pose2d getFusedPose() {
    return new Pose2d(
        m_poseEstimator.getEstimatedPosition().getX(),
        m_poseEstimator.getEstimatedPosition().getY(),
        Rotation2d.fromDegrees(getHeadingDegrees()));
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    m_poseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double x = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double y = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double omega = rot * DriveConstants.kMaxAngularSpeed;

    ChassisSpeeds speeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, omega, getRotation2d())
            : new ChassisSpeeds(x, y, omega);

    SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    m_desireState = states;
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(states[0]);
    m_frontRight.setDesiredState(states[1]);
    m_rearLeft.setDesiredState(states[2]);
    m_rearRight.setDesiredState(states[3]);
  }

  public void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    m_desireState = states;
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(states[0]);
    m_frontRight.setDesiredState(states[1]);
    m_rearLeft.setDesiredState(states[2]);
    m_rearRight.setDesiredState(states[3]);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  public void zeroHeading() {
    m_gyro.reset();
    Alliance al = DriverStation.getAlliance().orElse(Alliance.Blue);
    if (al == Alliance.Blue) {
      m_gyro.setYaw(0.0);
    } else {
      m_gyro.setYaw(180.0);
    }
  }

  public double getTurnRate() {
    double rate = m_gyro.getAngularVelocityZWorld().getValueAsDouble();
    return (DriveConstants.kGyroReversed ? -rate : rate);
  }
}
