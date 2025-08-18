package frc.robot.Auto;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.tan;
import static java.lang.Math.toRadians;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.slsh.IDeer9427.lib.Vision.GamepieceVisionIO;
import com.slsh.IDeer9427.lib.Vision.VisionConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain.DriveTrainSubsystem;

public class DriveToOrChaseGamepiece extends Command {

  public interface SwerveLike {
    Pose2d getFusedPose();

    void drive(ChassisSpeeds speeds);
  }

  private static final String OBJ_CORAL = "CoralTarget";
  private static final String OBJ_CORAL_TRACE = "CoralTrace";
  private static final String OBJ_FIXED = "FixedGoal";

  private final SwerveLike swerve;
  private final Supplier<Pose2d> fixedGoalSupplier;
  private final GamepieceVisionIO vision;
  private final VisionConfig cfg;

  private final int maxLostCycles = 6;
  private int lostCnt = 0;

  private Optional<Pose2d> latchedTargetField = Optional.empty();

  private final ArrayDeque<Pose2d> trace = new ArrayDeque<>();
  private final int traceLen = 40;

  private final PIDController rPid = new PIDController(5.0, 0.0, 0.0);
  private final PIDController yawPid = new PIDController(5.0, 0.0, 0.00);
  private final double kVmax = 1.8; // m/s
  private final double kWmax = 3.0; // rad/s
  private final double posTol = 0.03; // m
  private final double yawTol = toRadians(2.0);

  public DriveToOrChaseGamepiece(
      SwerveLike swerve,
      Supplier<Pose2d> fixedGoalSupplier,
      GamepieceVisionIO vision,
      VisionConfig cfg) {
    this.swerve = swerve;
    this.fixedGoalSupplier = fixedGoalSupplier;
    this.vision = vision;
    this.cfg = cfg;
  }

  @Override
  public void initialize() {
    rPid.setTolerance(posTol);
    yawPid.enableContinuousInput(-PI, PI);
    yawPid.setTolerance(yawTol);
    rPid.reset();
    yawPid.reset();
    lostCnt = 0;

    latchedTargetField = Optional.empty();

    DriveTrainSubsystem.m_field.getObject(OBJ_FIXED).setPose(fixedGoalSupplier.get());
    DriveTrainSubsystem.m_field.getObject(OBJ_CORAL).setPose(new Pose2d());
    DriveTrainSubsystem.m_field.getObject(OBJ_CORAL_TRACE).setPoses(List.of());
    trace.clear();
  }

  @Override
  public void execute() {
    if (latchedTargetField.isEmpty()) {
      Optional<Pose2d> firstSeen = estimateTargetPoseField();
      if (firstSeen.isPresent()) {
        latchedTargetField = firstSeen;
        lostCnt = 0;
      } else {
        if (++lostCnt > maxLostCycles) lostCnt = maxLostCycles;
      }
    }

    Pose2d visPose = latchedTargetField.orElseGet(() -> estimateTargetPoseField().orElse(null));
    if (visPose != null) {
      DriveTrainSubsystem.m_field.getObject(OBJ_CORAL).setPose(visPose);
      trace.addLast(visPose);
      while (trace.size() > traceLen) trace.removeFirst();
      DriveTrainSubsystem.m_field.getObject(OBJ_CORAL_TRACE).setPoses(new ArrayList<>(trace));
    }

    Pose2d goal = latchedTargetField.orElseGet(fixedGoalSupplier);

    Pose2d pose = swerve.getFusedPose();
    Translation2d toGoal = goal.getTranslation().minus(pose.getTranslation());
    double r = toGoal.getNorm();
    Translation2d dir = (r > 1e-6) ? toGoal.div(r) : new Translation2d();

    double vCmd = MathUtil.clamp(rPid.calculate(r, 0.0), -kVmax, kVmax);
    double vxField = dir.getX() * vCmd;
    double vyField = dir.getY() * vCmd;

    double yawErr = goal.getRotation().minus(pose.getRotation()).getRadians();
    double wCmd = MathUtil.clamp(yawPid.calculate(yawErr, 0.0), -kWmax, kWmax);

    swerve.drive(new ChassisSpeeds(-vxField, -vyField, -wCmd));
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    if (latchedTargetField.isPresent()) return false;

    Pose2d pose = swerve.getFusedPose();
    Pose2d goal = fixedGoalSupplier.get();
    boolean posOK = pose.getTranslation().getDistance(goal.getTranslation()) <= posTol;
    boolean yawOK = abs(goal.getRotation().minus(pose.getRotation()).getRadians()) <= yawTol;
    return posOK && yawOK;
  }

  private Optional<Pose2d> estimateTargetPoseField() {
    GamepieceVisionIO.Measurement m = vision.get();
    if (!m.hasTarget()) return Optional.empty();

    double distanceM;
    boolean widthOk = !m.cutOff() && (m.shortSidePx() >= cfg.minShortPxForWidthMethod());
    if (widthOk) {
      double px2radX = cfg.hfovRad() / cfg.imageWidthPx();
      double px2radY = cfg.vfovRad() / cfg.imageHeightPx();
      double pixelToRad = Math.min(px2radX, px2radY);
      double theta = m.shortSidePx() * pixelToRad;
      if (abs(tan(theta)) < 1e-6) return Optional.empty();
      distanceM = cfg.gamepieceDiameterM() / tan(theta);
      if (!(distanceM > 0 && distanceM < 20.0)) return Optional.empty();
    } else {
      double ang = cfg.camPitchRad() + m.tyRad();
      if (abs(tan(ang)) < 1e-6) return Optional.empty();
      distanceM = (cfg.camHeightM() - cfg.gamepieceHeightM()) / tan(ang);
      if (!(distanceM > 0 && distanceM < 20.0)) return Optional.empty();
    }

    Pose2d robotPose = swerve.getFusedPose();
    double headingField = robotPose.getRotation().getRadians() + cfg.camYawOffsetRad() + m.txRad();

    Translation2d camInField =
        robotPose.getTranslation().plus(cfg.robotToCam().rotateBy(robotPose.getRotation()));

    Translation2d camToTarget = new Translation2d(distanceM, new Rotation2d(headingField));
    Translation2d targetXY = camInField.plus(camToTarget);
    Rotation2d targetYaw = new Rotation2d(headingField);

    return Optional.of(new Pose2d(targetXY, targetYaw));
  }

  public void clearLatchedTarget() {
    latchedTargetField = Optional.empty();
  }
}
