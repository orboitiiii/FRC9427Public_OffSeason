package frc.robot.Auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.DriveTrain.SwerveConstants;
import java.util.function.Supplier;

public class AutoDrive {

  private static final double MIN_TURN_RADIUS =
      Math.hypot(
          SwerveConstants.DriveConstants.kTrackWidth / 2.0,
          SwerveConstants.DriveConstants.kWheelBase / 2.0);
  private static final double EPS = 1e-6;

  private final PIDController drivePID;
  private final PIDController headingPID;

  private final Translation2d preTarget; // F
  private final Translation2d finalTarget; // E
  private final double bisectorOffset; // φ = ∠(FE)

  private final double maxVelocity;
  private final double maxAcceleration;
  private final double maxSkidAcceleration;
  private final double loopPeriod;
  private final double targetTolerance;
  private final double transTol;
  private final double rotTol;
  private final double rotMaxVel;

  private final Supplier<ChassisSpeeds> speedsSupplier;

  private double lastTransError;
  private double lastHeadingError;

  private Double prevDriveAngleRad = null;
  private static final double SNAP_DIST = 0.6;
  private static final double SNAP_DELTA = Math.PI - 0.35;
  private static final double MAX_DA_PER_STEP = Math.toRadians(5.0);

  private static final double START_CONE_DEG = 30.0;
  private static final double START_CONE_DIST = 0.5;

  private static double wrapDiff(double from, double to) {
    return MathUtil.inputModulus(to - from, -Math.PI, Math.PI);
  }

  private static double rateLimitAngle(double prev, double target) {
    double d = wrapDiff(prev, target);
    d = MathUtil.clamp(d, -MAX_DA_PER_STEP, MAX_DA_PER_STEP);
    return prev + d;
  }

  public AutoDrive(
      Translation2d preTarget,
      Translation2d finalTarget,
      Supplier<ChassisSpeeds> speedsSupplier,
      double maxVelocity,
      double maxAcceleration,
      double maxSkidAcceleration,
      double loopPeriod,
      double targetTolerance,
      // translation PID
      double transP,
      double transI,
      double transD,
      double transTol,
      // rotation PID
      double rotP,
      double rotI,
      double rotD,
      double rotTol,
      double rotMaxVel) {

    this.preTarget = preTarget;
    this.finalTarget = finalTarget;
    this.bisectorOffset = finalTarget.minus(preTarget).getAngle().getRadians();

    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;
    this.maxSkidAcceleration = maxSkidAcceleration;
    this.loopPeriod = loopPeriod;
    this.targetTolerance = targetTolerance;
    this.transTol = transTol;
    this.rotTol = rotTol;
    this.rotMaxVel = rotMaxVel;
    this.speedsSupplier = speedsSupplier;

    this.drivePID = new PIDController(transP, transI, transD);
    this.drivePID.setTolerance(transTol);

    this.headingPID = new PIDController(rotP, rotI, rotD);
    this.headingPID.enableContinuousInput(-Math.PI, Math.PI);
    this.headingPID.setTolerance(rotTol);
  }

  public void reset(Pose2d currentPose) {
    drivePID.reset();
    headingPID.reset();
    prevDriveAngleRad = null;
  }

  public ChassisSpeeds calculateRobotRelativeSpeeds(Pose2d currentPose) {
    Translation2d A = currentPose.getTranslation();

    double driveAngle = computeHeuristicBisectorAngle(A);

    ChassisSpeeds rs = speedsSupplier.get();
    double vFwd =
        rs.vxMetersPerSecond * Math.cos(driveAngle) + rs.vyMetersPerSecond * Math.sin(driveAngle);

    double vCap = computeTargetVelocity(A, vFwd);

    double transOut = computeTranslationOutput(currentPose, vCap);
    double rotOut = computeRotationOutput(driveAngle, currentPose);

    Translation2d v = new Translation2d(Math.cos(driveAngle), Math.sin(driveAngle)).times(transOut);
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        v.getX(), v.getY(), rotOut, currentPose.getRotation());
  }

  private double computeHeuristicBisectorAngle(Translation2d A) {
    double dE = A.getDistance(finalTarget);
    double angleToE = finalTarget.minus(A).getAngle().getRadians();

    if (A.getDistance(preTarget) < EPS || preTarget.getDistance(finalTarget) < EPS) {
      double target = angleToE;
      if (dE > START_CONE_DIST) {
        double cone = Math.toRadians(START_CONE_DEG);
        double diff = MathUtil.inputModulus(target - angleToE, -Math.PI, Math.PI);
        target = angleToE + MathUtil.clamp(diff, -cone, cone);
      }
      if (prevDriveAngleRad == null) prevDriveAngleRad = target;
      prevDriveAngleRad = rateLimitAngle(prevDriveAngleRad, target);
      return prevDriveAngleRad;
    }

    double dF = A.getDistance(preTarget);
    double thetaF = preTarget.minus(A).getAngle().getRadians();
    double phiFE = bisectorOffset;

    double delta = Math.abs(MathUtil.inputModulus(thetaF - phiFE, -Math.PI, Math.PI));
    double targetAngle;
    if (dE < SNAP_DIST || delta > SNAP_DELTA) {
      targetAngle = angleToE;
    } else {
      double rawBisector = 2.0 * thetaF - phiFE;

      double R_dyn = Math.max(MIN_TURN_RADIUS, 0.50 * Math.min(dF, dE));

      double ratio = MathUtil.clamp(dF / (2.0 * R_dyn), 0.0, 1.0);
      double alphaCap = Math.asin(ratio);

      double rawAlpha = MathUtil.inputModulus(rawBisector - thetaF, -Math.PI, Math.PI);
      double alpha = MathUtil.clamp(rawAlpha, -alphaCap, alphaCap);
      targetAngle = thetaF + alpha;
    }

    if (dE > START_CONE_DIST) {
      double cone = Math.toRadians(START_CONE_DEG);
      double diff = MathUtil.inputModulus(targetAngle - angleToE, -Math.PI, Math.PI);
      targetAngle = angleToE + MathUtil.clamp(diff, -cone, cone);
    }

    if (prevDriveAngleRad == null) prevDriveAngleRad = targetAngle;
    prevDriveAngleRad = rateLimitAngle(prevDriveAngleRad, targetAngle);
    return prevDriveAngleRad;
  }

  private double computeTargetVelocity(Translation2d A, double currentFwdVel) {
    double vFwd = currentFwdVel;

    if (A.getDistance(finalTarget) < targetTolerance) return 0.0;

    double dE = A.getDistance(finalTarget);
    double vBrake = Math.sqrt(2.0 * maxAcceleration * dE);
    double desired = Math.min(maxVelocity, vBrake);

    double aWanted = (desired - vFwd) / loopPeriod;

    double forwardLimit =
        maxAcceleration * (1.0 - MathUtil.clamp(Math.abs(vFwd) / maxVelocity, 0.0, 1.0));

    double aAllow = Math.min(forwardLimit, maxSkidAcceleration);

    double accel = MathUtil.clamp(aWanted, -maxAcceleration, aAllow);

    double vNext = vFwd + accel * loopPeriod;
    return MathUtil.clamp(vNext, 0.0, maxVelocity);
  }

  private double computeTranslationOutput(Pose2d current, double vCap) {
    double e = current.getTranslation().getDistance(finalTarget);
    lastTransError = e;

    double vPid = -drivePID.calculate(e, 0.0); // setpoint=0
    if (Math.abs(e) < transTol) vPid = 0.0;

    return MathUtil.clamp(vPid, 0.0, vCap);
  }

  private double computeRotationOutput(double driveAngle, Pose2d current) {
    double cur = current.getRotation().getRadians();
    double err = MathUtil.inputModulus(cur - driveAngle, -Math.PI, Math.PI);
    lastHeadingError = Math.abs(err);

    double w = headingPID.calculate(cur, driveAngle);
    if (lastHeadingError < rotTol) w = 0.0;
    return MathUtil.clamp(w, -rotMaxVel, rotMaxVel);
  }

  public boolean atGoal() {
    return drivePID.atSetpoint() && headingPID.atSetpoint();
  }

  public boolean withinTolerance(double driveTol, Rotation2d thetaTol) {
    return lastTransError < driveTol && lastHeadingError < thetaTol.getRadians();
  }
}
