package frc.robot.subsystems.ElevatorArmSubsystem.LinearExtension;

import static frc.robot.Constants.ROBOT_NORMAL_CAN_TIMEOUT_MS;
import static frc.robot.Constants.kZERO;

import com.ctre.phoenix6.hardware.TalonFX;
import com.slsh.IDeer9427.lib.data.IDearLog;

import Control.Filter.KalmanFilter;
import Control.LQR.LQRController;
import Control.LQR.StateSpaceControlLoop;
import Control.LQR.StateSpaceModel;
import Control.LQR.StateSpaceSystem;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorArmSubsystem.LinearExtension.LinearExtensionConstants.Hardware;
import frc.robot.subsystems.ElevatorArmSubsystem.LinearExtension.LinearExtensionConstants.Pyhsical;
import frc.robot.subsystems.ElevatorArmSubsystem.LinearExtension.LinearExtensionConstants.System;
import frc.robot.util.Phoenix6Util;
import frc.robot.util.TalonMotorConfiguration;

public class LinearExtensionSubSystem extends SubsystemBase {

  private final TalonFX elevatorLeftMotor;
  private final TalonFX elevatorRightMotor;

  private final DCMotor motor = DCMotor.getKrakenX60(System.numMotor);

  // Precomputed feedforward voltage to counteract gravity
  // V_ff = ((mass * gravity * drumRadius) / (elevatorGear * motor.KtNMPerAmp)) * motor.rOhms
  private final double gravityFeedforwardVoltage =
      ((System.m * Constants.GRAVITY * System.radius * motor.rOhms) / (System.G * motor.KtNMPerAmp)
          + 0.10);

  private final TalonMotorConfiguration motorConfig =
      TalonMotorConfiguration.createElevatorMotor(
          Hardware.LEFT_ELEVATOR_MOTOR_ID, Hardware.RIGHT_ELEVATOR_MOTOR_ID);

  private final TrapezoidProfile m_profile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              Pyhsical.kMaxSpeed,
              Pyhsical.kMaxAcceleration)); // Max elevator speed and acceleration.

  private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();

  // State-space system for the elevator
  private final StateSpaceSystem<N2, N1, N1> m_elevatorPlant =
      StateSpaceModel.createElevatorSystem(motor, System.m, System.radius, System.G);

  // Observer (Kalman Filter) configuration using position measurements only
  private final KalmanFilter<N2, N1, N1> m_observer =
      new KalmanFilter<>(
          Nat.N2(),
          Nat.N1(),
          m_elevatorPlant,
          VecBuilder.fill(0.03, 0.10), // Process noise: [position, velocity]
          VecBuilder.fill(0.00001), // Measurement noise: [position]
          Constants.ROBOT_PERIODIC_MS);

  // LQR controller configuration with Q and R weights
  private final LQRController<N2, N1, N1> m_controller =
      new LQRController<>(
          m_elevatorPlant,
          VecBuilder.fill(0.05, 0.20), // Q weights: [position, velocity]
          VecBuilder.fill(Constants.ROBOT_NORMAL_VOLTAGE), // R weight: voltage penalty
          Constants.ROBOT_PERIODIC_MS);

  // Combined state-space control loop
  private final StateSpaceControlLoop<N2, N1, N1> m_loop =
      new StateSpaceControlLoop<>(
          m_elevatorPlant,
          m_controller,
          m_observer,
          Constants.ROBOT_NORMAL_VOLTAGE,
          Constants.ROBOT_PERIODIC_MS);

  public LinearExtensionSubSystem() {
    elevatorLeftMotor = motorConfig.motors()[0];
    elevatorRightMotor = motorConfig.motors()[1];

    IDearLog.getInstance()
        .addField(
            "ElevatorPosition",
            () -> elevatorLeftMotor.getPosition().getValueAsDouble(),
            IDearLog.FieldType.CAN);
    IDearLog.getInstance()
        .addField(
            "ElevatorVelocity",
            () -> elevatorLeftMotor.getVelocity().getValueAsDouble(),
            IDearLog.FieldType.CAN);

    IDearLog.getInstance()
        .addField("EleXhatP", () -> m_loop.getXHat().get(0, 0), IDearLog.FieldType.NON_CAN);
    IDearLog.getInstance()
        .addField("EleXhatV", () -> m_loop.getXHat().get(1, 0), IDearLog.FieldType.NON_CAN);

    IDearLog.getInstance()
        .addField("eleGF", () -> gravityFeedforwardVoltage, IDearLog.FieldType.NON_CAN);

    Phoenix6Util.checkErrorAndRetry(
        () -> elevatorLeftMotor.setPosition(kZERO, ROBOT_NORMAL_CAN_TIMEOUT_MS));

    m_loop.reset(
        VecBuilder.fill(
            elevatorRightMotor.getPosition().getValueAsDouble(),
            elevatorLeftMotor.getVelocity().getValueAsDouble()));

    m_lastProfiledReference =
        new TrapezoidProfile.State(
            elevatorRightMotor.getPosition().getValueAsDouble(),
            elevatorLeftMotor.getVelocity().getValueAsDouble());
  }

  public void stop() {
    elevatorLeftMotor.set(0);
    elevatorRightMotor.set(0);
  }

  public void holdPosition() {
    elevatorLeftMotor.setVoltage(gravityFeedforwardVoltage);
    elevatorRightMotor.setVoltage(gravityFeedforwardVoltage);
  }

  public void setVoltage(double voltage) {
    elevatorLeftMotor.setVoltage(voltage);
    elevatorRightMotor.setVoltage(-voltage);
  }

  @Override
  public void periodic() {}

  public void setReference(double desiredPosition) {
    TrapezoidProfile.State goal =
        new TrapezoidProfile.State(elevatorLeftMotor.getPosition().getValueAsDouble(), 0);

    m_lastProfiledReference =
        m_profile.calculate(Constants.ROBOT_PERIODIC_MS, m_lastProfiledReference, goal);

    // Set the next reference state: [desired position, desired velocity]
    m_loop.setNextR(desiredPosition, m_lastProfiledReference.velocity);

    // Update the state estimate using the measured position
    m_loop.correct(VecBuilder.fill(elevatorLeftMotor.getPosition().getValueAsDouble()));
    m_loop.predict(Constants.ROBOT_PERIODIC_MS);

    // Calculate feedback voltage from the LQR controller
    double feedbackVoltage = m_loop.getU(0);
    // Retrieve constant feedforward voltage for gravity compensation
    double totalVoltage = feedbackVoltage + gravityFeedforwardVoltage;

    // Clamp the total voltage to within allowed limits
    totalVoltage = Math.max(-7.0, Math.min(7.0, totalVoltage));

    setVoltage(totalVoltage);
  }

  public double getLinearExtensionMeters() {
    return elevatorLeftMotor.getPosition().getValueAsDouble();
  }

  public Command setVoltageCommand(double voltage) {
    return Commands.runEnd(() -> setVoltage(voltage), () -> stop(), this);
  }

  public Command holCommand() {
    return Commands.runEnd(() -> setVoltage(gravityFeedforwardVoltage), () -> stop(), this);
  }

  public Command setTarget(double target) {
    return Commands.runEnd(() -> setReference(target), () -> stop(), this);
  }
}
