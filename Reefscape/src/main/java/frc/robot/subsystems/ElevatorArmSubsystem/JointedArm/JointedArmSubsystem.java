package frc.robot.subsystems.ElevatorArmSubsystem.JointedArm;

import static frc.robot.Constants.ROBOT_NORMAL_CAN_TIMEOUT_MS;
import static frc.robot.Constants.ROBOT_NORMAL_VOLTAGE;
import static frc.robot.Constants.ROBOT_PERIODIC_MS;

import com.revrobotics.spark.SparkMax;
import com.slsh.IDeer9427.lib.controls.LinearController.LinearPlantInversionFeedforward;
import com.slsh.IDeer9427.lib.controls.LinearController.LinearQuadraticRegulator;
import com.slsh.IDeer9427.lib.controls.LinearLoops.LinearSystemLoop;
import com.slsh.IDeer9427.lib.controls.filter.KalmanFilter;
import com.slsh.IDeer9427.lib.controls.plant.LinearSystem;
import com.slsh.IDeer9427.lib.controls.plant.StateSpaceFactory;
import com.slsh.IDeer9427.lib.data.IDearLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorArmSubsystem.JointedArm.JointedArmConstants.Hardware;
import frc.robot.subsystems.ElevatorArmSubsystem.JointedArm.JointedArmConstants.Pyhsical;
import frc.robot.subsystems.ElevatorArmSubsystem.JointedArm.JointedArmConstants.System;
import frc.robot.util.SparkMaxConfiguration;
import org.ejml.simple.SimpleMatrix;

public class JointedArmSubsystem {
  private final SparkMax leftJointedArmMotor;
  private final SparkMax rightJointedArmMotor;

  private final DCMotor motor = DCMotor.getNEO(System.numMotor);

  // Precomputed feedforward voltage to counteract gravity
  // V_ff = (L / 2) * ((R * M * G) / (G * kt)) * cos(Theata)
  private final double gravityFeedforwardVoltage =
      (System.L / 2)
          * ((motor.rOhms * System.M * Constants.GRAVITY) / (System.G * motor.KtNMPerAmp));

  private final SparkMaxConfiguration motorConfig =
      SparkMaxConfiguration.createArmMotor(Hardware.LEFT_ARM_MOTOR_ID, Hardware.RIGHT_ARM_MOTOR_ID);

  private final LinearSystem plant =
      StateSpaceFactory.createSingleJointedArmSystem(motor, System.J, System.G);

  private final KalmanFilter filter =
      new KalmanFilter(plant, System.kalman_q, System.kalman_r, ROBOT_PERIODIC_MS);

  private final LinearQuadraticRegulator linearQuadraticRegulator =
      new LinearQuadraticRegulator(plant, System.kalman_q, System.lqr_r, ROBOT_PERIODIC_MS);

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
          new TrapezoidProfile.Constraints(Pyhsical.kMaxDegrees, Pyhsical.kMaxAcceleration));

  private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();

  private final SimpleMatrix referenceMatrix = new SimpleMatrix(2, 1);

  private SimpleMatrix stateMatrix = new SimpleMatrix(2, 1);

  public JointedArmSubsystem() {
    leftJointedArmMotor = motorConfig.motors()[0];
    rightJointedArmMotor = motorConfig.motors()[1];
    IDearLog.getInstance()
        .addField(
            "ArmPosition",
            () -> leftJointedArmMotor.getAbsoluteEncoder().getPosition(),
            IDearLog.FieldType.CAN);
    IDearLog.getInstance()
        .addField(
            "ArmVelocity",
            () -> leftJointedArmMotor.getAbsoluteEncoder().getVelocity(),
            IDearLog.FieldType.CAN);

    stateMatrix =
        new SimpleMatrix(
            new double[][] {
              {leftJointedArmMotor.getAbsoluteEncoder().getPosition()},
              {leftJointedArmMotor.getAbsoluteEncoder().getVelocity()}
            });

    linearSystemLoop.reset(stateMatrix);

    m_lastProfiledReference =
        new TrapezoidProfile.State(stateMatrix.get(0, 0), stateMatrix.get(1, 0));
  }

  public void stop() {
    leftJointedArmMotor.set(0);
    rightJointedArmMotor.set(0);
  }

  public void holdPosition() {
    double ff = gravityFeedforwardVoltage * Math.cos(Math.toRadians(getJointedArmAngleDegrees()));
    setVoltage(ff);
  }

  public void setVoltage(double voltage) {
    leftJointedArmMotor.setVoltage(voltage);
    rightJointedArmMotor.setVoltage(voltage);
  }

  public void setReference(double desiredPosition) {
    TrapezoidProfile.State goal = new TrapezoidProfile.State(desiredPosition, 0);
    m_lastProfiledReference = m_profile.calculate(ROBOT_PERIODIC_MS, m_lastProfiledReference, goal);
    stateMatrix.set(0, 0, leftJointedArmMotor.getAbsoluteEncoder().getPosition());
    stateMatrix.set(1, 0, leftJointedArmMotor.getAbsoluteEncoder().getVelocity());
    referenceMatrix.set(0, 0, desiredPosition);
    referenceMatrix.set(1, 0, m_lastProfiledReference.velocity);
    linearSystemLoop.setNextR(referenceMatrix);
    linearSystemLoop.correct(stateMatrix);
    linearSystemLoop.predict(ROBOT_PERIODIC_MS);
    setVoltage(linearSystemLoop.getU().get(0, 0));
  }

  public double getJointedArmAngleDegrees() {
    return leftJointedArmMotor.getAbsoluteEncoder().getPosition();
  }
}
