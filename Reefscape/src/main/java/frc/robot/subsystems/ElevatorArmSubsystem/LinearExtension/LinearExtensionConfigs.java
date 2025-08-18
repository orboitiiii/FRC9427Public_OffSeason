package frc.robot.subsystems.ElevatorArmSubsystem.LinearExtension;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class LinearExtensionConfigs {

  private LinearExtensionConfigs() {}

  private static double metersToDrumRot(double meters) {
    return meters / (2.0 * Math.PI * LinearExtensionConstants.System.radius);
  }

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
    // FXConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // FXConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
    //     metersToDrumRot(LinearExtensionConstants.Pyhsical.kMaxHeight);
    // FXConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // FXConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = metersToDrumRot(0.0);
    FXConfig.Slot0.kP = LinearExtensionConstants.ClosedLoop.kP;
    FXConfig.Slot0.kD = LinearExtensionConstants.ClosedLoop.kD;
    FXConfig.Slot0.kV =
        LinearExtensionConstants.ClosedLoop.kV
            * 2.0
            * Math.PI
            * LinearExtensionConstants.System.radius;
    FXConfig.MotionMagic.MotionMagicCruiseVelocity =
        metersToDrumRot(LinearExtensionConstants.Pyhsical.kMaxSpeed);
    FXConfig.MotionMagic.MotionMagicAcceleration =
        metersToDrumRot(LinearExtensionConstants.Pyhsical.kMaxAcceleration);
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
    // FXConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // FXConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
    //     metersToDrumRot(LinearExtensionConstants.Pyhsical.kMaxHeight);
    // FXConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // FXConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = metersToDrumRot(0.0);
    FXConfig.Slot0.kP = LinearExtensionConstants.ClosedLoop.kP;
    FXConfig.Slot0.kD = LinearExtensionConstants.ClosedLoop.kD;
    FXConfig.Slot0.kV =
        LinearExtensionConstants.ClosedLoop.kV
            * 2.0
            * Math.PI
            * LinearExtensionConstants.System.radius;
    FXConfig.MotionMagic.MotionMagicCruiseVelocity =
        metersToDrumRot(LinearExtensionConstants.Pyhsical.kMaxSpeed);
    FXConfig.MotionMagic.MotionMagicAcceleration =
        metersToDrumRot(LinearExtensionConstants.Pyhsical.kMaxAcceleration);
    FXConfig.Feedback.SensorToMechanismRatio = LinearExtensionConstants.System.G;
    FXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    return FXConfig;
  }
}
