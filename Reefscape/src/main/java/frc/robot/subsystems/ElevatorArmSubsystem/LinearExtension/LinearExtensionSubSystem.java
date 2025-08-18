package frc.robot.subsystems.ElevatorArmSubsystem.LinearExtension;

import static frc.robot.Constants.ROBOT_NORMAL_CAN_TIMEOUT_MS;
import static frc.robot.Constants.kZERO;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.slsh.IDeer9427.lib.data.IDearLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorArmSubsystem.LinearExtension.LinearExtensionConstants.Hardware;
import frc.robot.subsystems.ElevatorArmSubsystem.LinearExtension.LinearExtensionConstants.System;
import frc.robot.util.TalonMotorConfiguration;

public class LinearExtensionSubSystem extends SubsystemBase {

  private final TalonFX elevatorLeftMotor;
  private final TalonFX elevatorRightMotor;

  private final DCMotor motor = DCMotor.getKrakenX60(System.numMotor);

  private final double gravityFeedforwardVoltage =
      ((System.m * Constants.GRAVITY * System.radius * motor.rOhms)
          / (System.G * motor.KtNMPerAmp));

  private final TalonMotorConfiguration motorConfig =
      TalonMotorConfiguration.createElevatorMotor(
          Hardware.LEFT_ELEVATOR_MOTOR_ID, Hardware.RIGHT_ELEVATOR_MOTOR_ID);

  private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(kZERO);

  private SysIdRoutine sysIdRoutine;

  public LinearExtensionSubSystem() {
    elevatorLeftMotor = motorConfig.motors()[0];
    elevatorRightMotor = motorConfig.motors()[1];

    IDearLog.getInstance()
        .addField(
            "ElevatorPosition",
            () -> drumRotToMeters(elevatorLeftMotor.getPosition().getValueAsDouble()),
            IDearLog.FieldType.CAN);
    IDearLog.getInstance()
        .addField(
            "ElevatorVelocity",
            () -> drumRotToMeters(elevatorLeftMotor.getVelocity().getValueAsDouble()),
            IDearLog.FieldType.CAN);
    IDearLog.getInstance()
        .addField("eleGF", () -> gravityFeedforwardVoltage, IDearLog.FieldType.NON_CAN);

    elevatorLeftMotor.getConfigurator().apply(LinearExtensionConfigs.leftElevatorMotorConfig());
    elevatorRightMotor.getConfigurator().apply(LinearExtensionConfigs.rightElevatorMotorConfig());

    elevatorLeftMotor.setPosition(kZERO, ROBOT_NORMAL_CAN_TIMEOUT_MS);
    elevatorRightMotor.setPosition(kZERO, ROBOT_NORMAL_CAN_TIMEOUT_MS);

    // Phoenix6Util.checkErrorAndRetry(
    //     () -> elevatorLeftMotor.setPosition(kZERO, ROBOT_NORMAL_CAN_TIMEOUT_MS));
    // Phoenix6Util.checkErrorAndRetry(
    //     () -> elevatorRightMotor.setPosition(kZERO, ROBOT_NORMAL_CAN_TIMEOUT_MS));

    elevatorRightMotor.setControl(
        new com.ctre.phoenix6.controls.Follower(elevatorLeftMotor.getDeviceID(), true));

    IDearLog.getInstance()
        .addField(
            "L_V",
            () -> elevatorLeftMotor.getMotorVoltage().getValueAsDouble(),
            IDearLog.FieldType.CAN);
    IDearLog.getInstance()
        .addField(
            "R_V",
            () -> elevatorRightMotor.getMotorVoltage().getValueAsDouble(),
            IDearLog.FieldType.CAN);
    IDearLog.getInstance()
        .addField(
            "L_Duty",
            () -> elevatorLeftMotor.getDutyCycle().getValueAsDouble(),
            IDearLog.FieldType.CAN);
    IDearLog.getInstance()
        .addField(
            "R_Duty",
            () -> elevatorRightMotor.getDutyCycle().getValueAsDouble(),
            IDearLog.FieldType.CAN);

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Units.Volts.of(1).per(Units.Second), Units.Volts.of(4), Units.Seconds.of(3.5)),
            new SysIdRoutine.Mechanism(this::moveWithVoltage, this::logTelemetry, this));
  }

  private static double metersToDrumRot(double meters) {
    return meters / (2.0 * Math.PI * System.radius);
  }

  private static double drumRotToMeters(double rot) {
    return rot * 2.0 * Math.PI * System.radius;
  }

  private void moveWithVoltage(Voltage voltage) {
    setVoltage(voltage.in(Units.Volts));
  }

  private void logTelemetry(SysIdRoutineLog log) {
    log.motor("Lift")
        .voltage(
            Voltage.ofBaseUnits(
                elevatorLeftMotor.getMotorVoltage().getValueAsDouble(), Units.Volts))
        .linearVelocity(
            LinearVelocity.ofBaseUnits(
                drumRotToMeters(elevatorLeftMotor.getVelocity().getValueAsDouble()),
                Units.MetersPerSecond))
        .linearPosition(
            Distance.ofBaseUnits(
                drumRotToMeters(elevatorLeftMotor.getPosition().getValueAsDouble()), Units.Meters));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  public void stop() {
    elevatorLeftMotor.stopMotor();
  }

  public void holdPosition() {
    elevatorLeftMotor.setVoltage(gravityFeedforwardVoltage);
  }

  public void setVoltage(double voltage) {
    elevatorLeftMotor.setVoltage(voltage);
  }

  @Override
  public void periodic() {}

  public void setReference(double desiredPositionMeters) {
    elevatorLeftMotor.setControl(motionMagic.withPosition(metersToDrumRot(desiredPositionMeters)));
  }

  public double getLinearExtensionMeters() {
    return drumRotToMeters(elevatorLeftMotor.getPosition().getValueAsDouble());
  }

  public Command setVoltageCommand(double voltage) {
    return Commands.runEnd(() -> setVoltage(voltage), this::stop, this);
  }

  public Command holCommand() {
    return Commands.runEnd(() -> setVoltage(gravityFeedforwardVoltage), this::stop, this);
  }

  public Command setTarget(double targetMeters) {
    return Commands.runEnd(() -> setReference(targetMeters), this::stop, this);
  }
}
