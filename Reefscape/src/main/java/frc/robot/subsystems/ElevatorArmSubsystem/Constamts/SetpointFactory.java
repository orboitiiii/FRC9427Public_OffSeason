package frc.robot.subsystems.ElevatorArmSubsystem.Constamts;

import frc.robot.subsystems.ElevatorArmSubsystem.ElevatorArmPosition;

public class SetpointFactory {
  public static final ElevatorArmPosition ZEROED = new ElevatorArmPosition(0, -90);
  public static final ElevatorArmPosition SAFETY_ELEVATOR_POSITION =
      new ElevatorArmPosition(0.15, 0);
  public static final ElevatorArmPosition SAFETY_ARM_POSITION = new ElevatorArmPosition(0, 90);
}
