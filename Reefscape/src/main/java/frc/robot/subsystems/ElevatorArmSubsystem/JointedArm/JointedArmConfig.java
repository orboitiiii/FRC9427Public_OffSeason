package frc.robot.subsystems.ElevatorArmSubsystem.JointedArm;

import com.revrobotics.spark.config.SparkMaxConfig;

public class JointedArmConfig {
  public static SparkMaxConfig getLeftArmMotorConfig() {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .encoder
        .positionConversionFactor(0.0625)
        .velocityConversionFactor(0.0625)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    config
        .inverted(false)
        .idleMode(SparkMaxConfig.IdleMode.kBrake)
        .smartCurrentLimit(67)
        .voltageCompensation(12.0);
    config
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    config
        .absoluteEncoder
        .inverted(false)
        .positionConversionFactor(360)
        .velocityConversionFactor(6)
        .zeroCentered(true)
        .averageDepth(2);
    return config;
  }

  public static SparkMaxConfig getRightArmMotorConfig() {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(true)
        .idleMode(SparkMaxConfig.IdleMode.kBrake)
        .smartCurrentLimit(67)
        .voltageCompensation(12.0);
    return config;
  }
}
