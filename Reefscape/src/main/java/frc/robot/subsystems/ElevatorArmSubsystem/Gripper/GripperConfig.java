package frc.robot.subsystems.ElevatorArmSubsystem.Gripper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class GripperConfig {
    public static TalonFXConfiguration gripperMotorConfig() {
    TalonFXConfiguration FXConfig = new TalonFXConfiguration();
    FXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    FXConfig.CurrentLimits.SupplyCurrentLimit = 80.0;
    FXConfig.CurrentLimits.SupplyCurrentLowerLimit = 80.0;
    FXConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;
    FXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    FXConfig.CurrentLimits.StatorCurrentLimit = 120.0;
    FXConfig.Voltage.PeakForwardVoltage = 12.0;
    FXConfig.Voltage.PeakReverseVoltage = -12.0;
    FXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    return FXConfig;
  }
}
