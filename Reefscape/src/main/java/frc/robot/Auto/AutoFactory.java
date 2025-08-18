package frc.robot.Auto;

import com.slsh.IDeer9427.lib.Vision.GamepieceVisionIO;
import com.slsh.IDeer9427.lib.Vision.LL3GamepieceVision;
import com.slsh.IDeer9427.lib.Vision.ReefFieldPose;
import com.slsh.IDeer9427.lib.Vision.ReefFieldPose.ReefPoint;
import com.slsh.IDeer9427.lib.Vision.VisionConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.DriveTrain.DriveTrainSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedBigArmState;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AutoFactory {
  private final Superstructure superstructure;
  private final DriveTrainSubsystem swerve;
  private final DriveToPoint driveToF;
  private final DriveToPoint driveToD;
  private final DriveToPoint driveToC;
  private final DriveToPoint driveToB;

  private final PIDController rPid = new PIDController(2.2, 0.0, 0.2);
  private final PIDController yawPid = new PIDController(3.0, 0.0, 0.0);

  DriveToOrChaseGamepiece.SwerveLike swerveLike;

  Supplier<Pose2d> fixedGoal = () -> new Pose2d(2.002, 1.490, Rotation2d.fromDegrees(45));

  VisionConfig visCfg =
      new VisionConfig(
          /* imageWidthPx  */ 640,
          /* imageHeightPx */ 480,
          /* hfovRad       */ 1.347447017188408,
          /* vfovRad       */ 1.010585262891306,
          /* diameterM     */ 0.1143,
          /* camHeightM    */ 0.91851014,
          /* objHeightM    */ 0.0,
          /* camPitchRad   */ Math.toRadians(-60.0),
          /* camYawOffset  */ 0.0,
          /* robotToCam    */ new Translation2d(0.1270021, 0.0),
          /* minShortPx    */ 10);

  GamepieceVisionIO vision = new LL3GamepieceVision("limelight-back", visCfg);

  public AutoFactory(Superstructure _Superstructure, DriveTrainSubsystem _swerve) {
    this.superstructure = _Superstructure;
    this.swerve = _swerve;
    driveToF =
        new DriveToPoint(
            _swerve,
            ReefFieldPose.getReefPose(ReefPoint.F, 1.0).getTranslation(),
            ReefFieldPose.getReefPose(ReefPoint.F, 0.47).getTranslation());
    driveToD =
        new DriveToPoint(
            _swerve,
            ReefFieldPose.getReefPose(ReefPoint.D, 1.0).getTranslation(),
            ReefFieldPose.getReefPose(ReefPoint.D, 0.47).getTranslation());
    driveToC =
        new DriveToPoint(
            _swerve,
            ReefFieldPose.getReefPose(ReefPoint.C, 1.0).getTranslation(),
            ReefFieldPose.getReefPose(ReefPoint.C, 0.47).getTranslation());
    driveToB =
        new DriveToPoint(
            _swerve,
            ReefFieldPose.getReefPose(ReefPoint.B, 1.0).getTranslation(),
            ReefFieldPose.getReefPose(ReefPoint.B, 0.47).getTranslation());

    swerveLike =
        new DriveToOrChaseGamepiece.SwerveLike() {
          @Override
          public Pose2d getFusedPose() {
            return swerve.getFusedPose();
          }

          @Override
          public void drive(ChassisSpeeds u) {
            swerve.drive(u);
          }
        };
  }

  public Command driveOrChase() {
    return new DriveToOrChaseGamepiece(swerveLike, fixedGoal, vision, visCfg);
  }

  public Command driveToF() {
    final Pose2d goal = ReefFieldPose.getReefPose(ReefPoint.F, 0.47);

    final BooleanSupplier isAtHome = superstructure.isHome;
    final BooleanSupplier armRaised = superstructure.isL4;

    DoubleSupplier dist =
        () -> swerve.getFusedPose().getTranslation().getDistance(goal.getTranslation());

    Command preLiftIfHasCoral =
        new WaitUntilCommand(() -> dist.getAsDouble() < 1.5 && isAtHome.getAsBoolean())
            .andThen(superstructure.setBigArmStateCommand(WantedBigArmState.PRE_L4));

    Command finalApproach =
        p2pToRadialDistance(goal, 0.0)
            .until(() -> !armRaised.getAsBoolean() && dist.getAsDouble() < 0.55)
            .andThen(
                Commands.either(
                    Commands.sequence(
                        p2pToRadialDistance(goal, 0.55), Commands.waitUntil(armRaised)),
                    Commands.sequence(
                        Commands.runOnce(() -> swerve.drive(new ChassisSpeeds()), swerve),
                        Commands.waitUntil(armRaised)),
                    () -> dist.getAsDouble() < 0.45))
            .andThen(p2pToRadialDistance(goal, 0.0));

    Command farApproach = driveToF;

    return farApproach
        .alongWith(preLiftIfHasCoral)
        .andThen(finalApproach)
        .andThen(superstructure.setBigArmStateCommand(WantedBigArmState.PLACE_L4));
  }

  private Command p2pToRadialDistance(Pose2d goal, double targetR) {
    rPid.setTolerance(0.02); // 2 cm
    yawPid.enableContinuousInput(-Math.PI, Math.PI);
    yawPid.setTolerance(Math.toRadians(2.0));

    // DoubleSupplier dist =
    //     () -> swerve.getFusedPose().getTranslation().getDistance(goal.getTranslation());

    return new FunctionalCommand(
        // init
        () -> {
          rPid.reset();
          yawPid.reset();
        },
        // execute
        () -> {
          Pose2d pose = swerve.getFusedPose();
          Translation2d toGoal = goal.getTranslation().minus(pose.getTranslation());
          double r = toGoal.getNorm();
          if (r < 1e-6) {
            swerve.drive(new ChassisSpeeds());
            return;
          }
          Translation2d dir = toGoal.div(r);
          //   double rErr = (r - targetR);
          double vCmd = MathUtil.clamp(rPid.calculate(r, targetR), -1.5, 1.5);
          double vxField = dir.getX() * vCmd;
          double vyField = dir.getY() * vCmd;

          double yawErr = goal.getRotation().minus(pose.getRotation()).getRadians();
          double omegaCmd = MathUtil.clamp(yawPid.calculate(0.0, -yawErr), -3.0, 3.0); // rad/s

          swerve.drive(new ChassisSpeeds(vxField, vyField, omegaCmd));
        },
        (interrupted) -> swerve.drive(new ChassisSpeeds()),
        () -> rPid.atSetpoint() && yawPid.atSetpoint(),
        swerve);
  }
}
