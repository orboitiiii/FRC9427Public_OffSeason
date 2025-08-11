package frc.robot.subsystems.ElevatorArmSubsystem;

public class ElevatorArmPosition {
  private double extensionLengthMeters = 0;
  private double jointedArmAngleDegrees = 0;

  public ElevatorArmPosition(double extensionLengthMeters, double jointedArmAngleDegrees) {
    this.extensionLengthMeters = extensionLengthMeters;
    this.jointedArmAngleDegrees = jointedArmAngleDegrees;
  }

  public double getExtensionLengthMeters() {
    return extensionLengthMeters;
  }

  public double getJointedArmAngleDegrees() {
    return jointedArmAngleDegrees;
  }
}
