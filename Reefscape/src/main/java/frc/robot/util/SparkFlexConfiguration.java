package frc.robot.util;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.subsystems.IntakeSubsystem.PivotSystem.IntakePivotConfig;
import frc.robot.subsystems.IntakeSubsystem.RollerAndIndexerSystem.IndexerConfig;
import frc.robot.subsystems.IntakeSubsystem.RollerAndIndexerSystem.RollerConfig;

public record SparkFlexConfiguration(SparkFlex... motors) {
  public static SparkFlexConfiguration createIntakePivotMotor(int leftID, int rightID) {
    SparkFlex leftMotor = new SparkFlex(leftID, MotorType.kBrushless);
    SparkFlex rightMotor = new SparkFlex(rightID, MotorType.kBrushless);
    leftMotor.configure(
        IntakePivotConfig.getLeftArmMotorConfig(),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    rightMotor.configure(
        IntakePivotConfig.getRightArmMotorConfig().follow(leftMotor, true),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    return new SparkFlexConfiguration(leftMotor, rightMotor);
  }

  public static SparkFlexConfiguration createRollerAndIndexerMotor(
      int leftID, int rightID, int rollerID) {
    SparkFlex leftMotor = new SparkFlex(leftID, MotorType.kBrushless);
    SparkFlex rightMotor = new SparkFlex(rightID, MotorType.kBrushless);
    SparkFlex rollerMotor = new SparkFlex(rollerID, MotorType.kBrushless);

    leftMotor.configure(
        IndexerConfig.getLeftIndexerMotorConfig(),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    rightMotor.configure(
        IndexerConfig.getRightIndexerMotorConfig(),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    rollerMotor.configure(
        RollerConfig.getRollerMotorConfig(),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    return new SparkFlexConfiguration(leftMotor, rightMotor, rollerMotor);
  }
}
