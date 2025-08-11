package frc.robot.subsystems.ElevatorArmSubsystem.LinearExtension;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.Util;

/**
 * ElevatorConfigs provides the configuration for the elevator's SparkMax controllers.
 *
 * <p>This class uses a template method pattern. The initialization methods are explicitly called in
 * the constructor.
 */
public class LinearExtensionConfigs {

  public LinearExtensionConfigs() {}

  public static TalonFXConfiguration leftElevatorMotorConfig() {
    TalonFXConfiguration FXConfig = new TalonFXConfiguration();
    FXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    FXConfig.CurrentLimits.SupplyCurrentLimit = 80.0;
    FXConfig.CurrentLimits.SupplyCurrentLowerLimit = 80.0;
    FXConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

    FXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    FXConfig.CurrentLimits.StatorCurrentLimit = 120.0;

    FXConfig.Voltage.PeakForwardVoltage = 12.0;
    FXConfig.Voltage.PeakReverseVoltage = -12.0;

    FXConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    FXConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Util.linearDistanceToRotation(
            LinearExtensionConstants.Pyhsical.kMaxHeight,
            LinearExtensionConstants.System.radius,
            LinearExtensionConstants.System.G);
    FXConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    FXConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Util.linearDistanceToRotation(
            LinearExtensionConstants.Pyhsical.kMaxHeight,
            LinearExtensionConstants.System.radius,
            LinearExtensionConstants.System.G);
    FXConfig.Feedback.SensorToMechanismRatio = LinearExtensionConstants.System.G;

    FXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    return FXConfig;
  }

  public static TalonFXConfiguration rightElevatorMotorConfig() {
    TalonFXConfiguration FXConfig = new TalonFXConfiguration();
    FXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    FXConfig.CurrentLimits.SupplyCurrentLimit = 80.0;
    FXConfig.CurrentLimits.SupplyCurrentLowerLimit = 80.0;
    FXConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

    FXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    FXConfig.CurrentLimits.StatorCurrentLimit = 120.0;

    FXConfig.Voltage.PeakForwardVoltage = 12.0;
    FXConfig.Voltage.PeakReverseVoltage = -12.0;

    FXConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    FXConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Util.linearDistanceToRotation(
            LinearExtensionConstants.Pyhsical.kMaxHeight,
            LinearExtensionConstants.System.radius,
            LinearExtensionConstants.System.G);
    FXConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    FXConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Util.linearDistanceToRotation(
            LinearExtensionConstants.Pyhsical.kMaxHeight,
            LinearExtensionConstants.System.radius,
            LinearExtensionConstants.System.G);
    FXConfig.Feedback.SensorToMechanismRatio = LinearExtensionConstants.System.G;

    FXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    return FXConfig;
  }
}
