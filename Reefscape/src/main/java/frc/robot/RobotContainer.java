// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.ElevatorArmSubsystem.JointedArm.JointedArmSubsystem;
import frc.robot.subsystems.ElevatorArmSubsystem.LinearExtension.LinearExtensionSubSystem;

public class RobotContainer {

  private final JointedArmSubsystem armSubsystem = new JointedArmSubsystem();

  private final LinearExtensionSubSystem linearExtensionSubSystem = new LinearExtensionSubSystem();

  private final CommandPS5Controller commandPS5Controller = new CommandPS5Controller(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    armSubsystem.setDefaultCommand(armSubsystem.holdingCommand());
    linearExtensionSubSystem.setDefaultCommand(linearExtensionSubSystem.holCommand());
    commandPS5Controller.circle().whileTrue(linearExtensionSubSystem.setVoltageCommand(3.0));
    commandPS5Controller.square().whileTrue(linearExtensionSubSystem.setVoltageCommand(-3.0));
    commandPS5Controller.triangle().whileTrue(linearExtensionSubSystem.setTarget(0.25));
    // commandPS5Controller.triangle().whileTrue(armSubsystem.setVoltageCommand(1.25));
    // commandPS5Controller.cross().whileTrue(armSubsystem.setVoltageCommand(-1.25));
    // commandPS5Controller.square().whileTrue(armSubsystem.setTargetCommand(45));
    // commandPS5Controller
    //     .cross()
    //     .whileTrue(armSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // commandPS5Controller
    //     .triangle()
    //     .whileTrue(armSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // commandPS5Controller
    //     .square()
    //     .whileTrue(armSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // commandPS5Controller
    //     .circle()
    //     .whileTrue(armSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
