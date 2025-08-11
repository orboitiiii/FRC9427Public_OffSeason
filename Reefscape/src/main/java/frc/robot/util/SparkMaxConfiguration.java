package frc.robot.util;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.subsystems.ElevatorArmSubsystem.JointedArm.JointedArmConfig;

public record SparkMaxConfiguration(SparkMax... motors) {
  public static SparkMaxConfiguration createArmMotor(int leftID, int rightID) {
    SparkMax leftMotor = new SparkMax(leftID, MotorType.kBrushless);
    SparkMax rightMotor = new SparkMax(rightID, MotorType.kBrushless);
    leftMotor.configure(
        JointedArmConfig.getLeftArmMotorConfig(),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    rightMotor.configure(
        JointedArmConfig.getRightArmMotorConfig(),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    return new SparkMaxConfiguration(leftMotor, rightMotor);
  }
}
