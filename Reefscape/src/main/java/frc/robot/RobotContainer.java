// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Auto.AutoFactory;
import frc.robot.subsystems.DriveTrain.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorArmSubsystem.ElevatorArmSubsystem;
import frc.robot.subsystems.ElevatorArmSubsystem.Gripper.Gripper;
import frc.robot.subsystems.ElevatorArmSubsystem.JointedArm.JointedArmSubsystem;
import frc.robot.subsystems.ElevatorArmSubsystem.LinearExtension.LinearExtensionSubSystem;
import frc.robot.subsystems.IntakeSubsystem.PivotSystem.IntakePivot;
import frc.robot.subsystems.IntakeSubsystem.RollerAndIndexerSystem.RollerAndIndexer;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedBigArmState;

public class RobotContainer {

  private final JointedArmSubsystem armSubsystem = new JointedArmSubsystem();

  private final LinearExtensionSubSystem linearExtensionSubSystem = new LinearExtensionSubSystem();

  private final ElevatorArmSubsystem elevatorArmSubsystem =
      new ElevatorArmSubsystem(armSubsystem, linearExtensionSubSystem);

  private final IntakePivot intakePivot = new IntakePivot();

  public final CommandPS5Controller commandPS5Controller = new CommandPS5Controller(0);

  private final DriveTrainSubsystem swerve = new DriveTrainSubsystem();

  // private final DriveToPoint m_DriveToPoint =
  //     new DriveToPoint(
  //         swerve,
  //         ReefFieldPose.getReefPose(ReefPoint.A, 1.0).getTranslation(),
  //         ReefFieldPose.getReefPose(ReefPoint.A, 0.40).getTranslation());

  private final RollerAndIndexer rollerAndIndexer = new RollerAndIndexer();

  private final Gripper gripper = new Gripper();

  private final Superstructure superstructure =
      new Superstructure(elevatorArmSubsystem, intakePivot, rollerAndIndexer, gripper, this);

  private final AutoFactory autoFactory = new AutoFactory(superstructure, swerve);

  public RobotContainer() {
    swerve.setDefaultCommand(
        new RunCommand(
            () ->
                swerve.drive(
                    MathUtil.applyDeadband(-commandPS5Controller.getLeftY(), 0.08),
                    MathUtil.applyDeadband(-commandPS5Controller.getLeftX(), 0.08),
                    MathUtil.applyDeadband(-commandPS5Controller.getRightX(), 0.08),
                    true),
            swerve));
    configureBindings();
  }

  private void configureBindings() {
    commandPS5Controller
        .square()
        .onTrue(superstructure.setBigArmStateCommand(WantedBigArmState.PRE_L3));

    commandPS5Controller
        .cross()
        .onTrue(superstructure.setBigArmStateCommand(WantedBigArmState.PLACE_L3));

    commandPS5Controller
        .R1()
        .onTrue(superstructure.setBigArmStateCommand(WantedBigArmState.MOVE_TO_HOME));

    commandPS5Controller
        .button(15)
        .onTrue(superstructure.setBigArmStateCommand(WantedBigArmState.MANUAL));

    // commandPS5Controller.triangle().whileTrue(m_DriveToPoint);
  }

  public Command getAutonomousCommand() {
    return autoFactory.driveOrChase();
  }
}
