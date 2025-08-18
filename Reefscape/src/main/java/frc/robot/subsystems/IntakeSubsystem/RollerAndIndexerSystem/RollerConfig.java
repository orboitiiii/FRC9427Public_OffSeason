package frc.robot.subsystems.IntakeSubsystem.RollerAndIndexerSystem;

import com.revrobotics.spark.config.SparkMaxConfig;

public class RollerConfig {
  public static SparkMaxConfig getRollerMotorConfig() {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(true)
        .idleMode(SparkMaxConfig.IdleMode.kCoast)
        .smartCurrentLimit(67)
        .voltageCompensation(12.0);
    return config;
  }
}
