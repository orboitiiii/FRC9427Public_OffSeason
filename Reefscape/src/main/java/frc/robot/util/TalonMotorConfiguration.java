package frc.robot.util;

import com.ctre.phoenix6.hardware.TalonFX;

public record TalonMotorConfiguration(TalonFX... motors) {
  public static TalonMotorConfiguration createElevatorMotor(int leftID, int rightID) {
    TalonFX leftMotor = new TalonFX(leftID);
    TalonFX rightMotor = new TalonFX(rightID);
    // TalonUtil.applyAndCheckConfiguration(
    //     leftMotor, LinearExtensionConfigs.leftElevatorMotorConfig());
    // TalonUtil.applyAndCheckConfiguration(
    //     rightMotor, LinearExtensionConfigs.rightElevatorMotorConfig());
    return new TalonMotorConfiguration(leftMotor, rightMotor);
  }
}
