package frc.robot.subsystems.ElevatorArmSubsystem.LinearExtension;

import static frc.robot.Constants.ROBOT_NORMAL_CAN_TIMEOUT_MS;
import static frc.robot.Constants.ROBOT_NORMAL_VOLTAGE;
import static frc.robot.Constants.ROBOT_PERIODIC_MS;
import static frc.robot.Constants.kZERO;

import com.ctre.phoenix6.hardware.TalonFX;
import com.slsh.IDeer9427.lib.controls.LinearController.LinearPlantInversionFeedforward;
import com.slsh.IDeer9427.lib.controls.LinearController.LinearQuadraticRegulator;
import com.slsh.IDeer9427.lib.controls.LinearLoops.LinearSystemLoop;
import com.slsh.IDeer9427.lib.controls.filter.KalmanFilter;
import com.slsh.IDeer9427.lib.controls.plant.LinearSystem;
import com.slsh.IDeer9427.lib.controls.plant.StateSpaceFactory;
import com.slsh.IDeer9427.lib.data.IDearLog;
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
import org.ejml.simple.SimpleMatrix;

public class LinearExtensionSubSystem extends SubsystemBase {

  private final TalonFX elevatorLeftMotor;
  private final TalonFX elevatorRightMotor;

  private final DCMotor motor = DCMotor.getKrakenX60(System.numMotor);

  // Precomputed feedforward voltage to counteract gravity
  // V_ff = ((mass * gravity * drumRadius) / (elevatorGear * motor.KtNMPerAmp)) * motor.rOhms
  private final double gravityFeedforwardVoltage =
      ((System.m * Constants.GRAVITY * System.radius * motor.rOhms)
          / (System.G * motor.KtNMPerAmp));

  private final TalonMotorConfiguration motorConfig =
      TalonMotorConfiguration.createElevatorMotor(
          Hardware.LEFT_ELEVATOR_MOTOR_ID, Hardware.RIGHT_ELEVATOR_MOTOR_ID);

  private final LinearSystem plant =
      StateSpaceFactory.createElevatorSystem(motor, System.m, System.radius, System.G);

  private final KalmanFilter filter =
      new KalmanFilter(plant, System.kalman_q, System.kalman_r, ROBOT_PERIODIC_MS);

  private final LinearQuadraticRegulator linearQuadraticRegulator =
      new LinearQuadraticRegulator(plant, System.lqr_q, System.lqr_r, ROBOT_PERIODIC_MS);

  private final LinearPlantInversionFeedforward linearPlantInversionFeedforward =
      new LinearPlantInversionFeedforward(plant, ROBOT_NORMAL_CAN_TIMEOUT_MS);

  private final LinearSystemLoop linearSystemLoop =
      new LinearSystemLoop(
          linearQuadraticRegulator,
          linearPlantInversionFeedforward,
          filter,
          LinearSystemLoop.createVoltageClamp(ROBOT_NORMAL_VOLTAGE));

  private final TrapezoidProfile m_profile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              Pyhsical.kMaxSpeed,
              Pyhsical.kMaxAcceleration)); // Max elevator speed and acceleration.

  private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();

  private final SimpleMatrix referenceMatrix = new SimpleMatrix(2, 1);

  private SimpleMatrix stateMatrix;

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

    Phoenix6Util.checkErrorAndRetry(
        () -> elevatorLeftMotor.setPosition(kZERO, ROBOT_NORMAL_CAN_TIMEOUT_MS));

    stateMatrix =
        new SimpleMatrix(
            new double[][] {
              {elevatorLeftMotor.getPosition().getValueAsDouble()},
              {elevatorLeftMotor.getVelocity().getValueAsDouble()}
            });

    linearSystemLoop.reset(stateMatrix);

    m_lastProfiledReference =
        new TrapezoidProfile.State(stateMatrix.get(0, 0), stateMatrix.get(1, 0));
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
    elevatorLeftMotor.setVoltage(-voltage);
    elevatorRightMotor.setVoltage(voltage);
  }

  public void setReference(double desiredPosition) {
    TrapezoidProfile.State goal = new TrapezoidProfile.State(desiredPosition, 0);
    m_lastProfiledReference = m_profile.calculate(ROBOT_PERIODIC_MS, m_lastProfiledReference, goal);
    stateMatrix.set(0, 0, elevatorLeftMotor.getPosition().getValueAsDouble());
    stateMatrix.set(1, 0, elevatorLeftMotor.getVelocity().getValueAsDouble());
    referenceMatrix.set(0, 0, desiredPosition);
    referenceMatrix.set(1, 0, m_lastProfiledReference.velocity);
    linearSystemLoop.setNextR(referenceMatrix);
    linearSystemLoop.correct(stateMatrix);
    linearSystemLoop.predict(ROBOT_PERIODIC_MS);
    setVoltage(linearSystemLoop.getU().get(0, 0));
  }

  public double getLinearExtensionMeters() {
    return elevatorLeftMotor.getPosition().getValueAsDouble();
  }

  public Command setVoltageCommand(double voltage) {
    return Commands.runEnd(() -> setVoltage(voltage), () -> stop(), this);
  }
}
