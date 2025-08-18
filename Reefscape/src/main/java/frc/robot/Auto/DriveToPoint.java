package frc.robot.Auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain.DriveTrainSubsystem;

public class DriveToPoint extends Command {
  private final DriveTrainSubsystem swerve;
  private AutoDrive autoDrive;
  private final double transP = 5.0;
  private final double transD = 0.0;
  private final double rotP = 5.0;
  private final double rotD = 0.0;

  public DriveToPoint(
      DriveTrainSubsystem _swerve, Translation2d preTarget, Translation2d finalTarget) {
    this.swerve = _swerve;
    autoDrive =
        new AutoDrive(
            preTarget,
            finalTarget,
            () -> swerve.getRobotRelativeSpeeds(),
            4.8,
            5.0,
            4.0,
            Constants.ROBOT_PERIODIC_MS,
            0.005,
            transP,
            0,
            transD,
            0.005,
            rotP,
            0,
            rotD,
            Math.toRadians(0.5),
            Math.PI);
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    autoDrive.reset(swerve.getFusedPose());
  }

  @Override
  public void execute() {
    swerve.drive(autoDrive.calculateRobotRelativeSpeeds(swerve.getFusedPose()));
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(0, 0, 0, true);
  }

  @Override
  public boolean isFinished() {
    return autoDrive.atGoal();
  }
}
