package frc.robot.subsystems.ElevatorArmSubsystem.JointedArm;

import com.revrobotics.spark.SparkMax;
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
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorArmSubsystem.JointedArm.JointedArmConstants.Hardware;
import frc.robot.subsystems.ElevatorArmSubsystem.JointedArm.JointedArmConstants.Pyhsical;
import frc.robot.subsystems.ElevatorArmSubsystem.JointedArm.JointedArmConstants.System;
import frc.robot.util.SparkMaxConfiguration;

public class JointedArmSubsystem extends SubsystemBase {

  private final SparkMax leftJointedArmMotor;
  private final SparkMax rightJointedArmMotor;
  private SysIdRoutine sysIdRoutine;

  private final DCMotor motor = DCMotor.getNEO(System.numMotor);

  // Precomputed feedforward voltage to counteract gravity
  // V_ff = (L / 3.5) * ((R * M * G) / (G * kt)) * cos(Theata)
  private final double gravityFeedforwardVoltage =
      (System.L / 3.5)
          * ((motor.rOhms * System.M * Constants.GRAVITY) / (System.G * motor.KtNMPerAmp));

  private final SparkMaxConfiguration motorConfig =
      SparkMaxConfiguration.createArmMotor(Hardware.LEFT_ARM_MOTOR_ID, Hardware.RIGHT_ARM_MOTOR_ID);

  // private final PIDController controller = new PIDController(0.25, 0, 0.03);

  private final TrapezoidProfile m_profile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              Pyhsical.kMaxRadians,
              Pyhsical.kMaxRadiansAcceleration)); // Max arm speed and acceleration.
  private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();

  // State-space system for the arm
  private final StateSpaceSystem<N2, N1, N1> m_ArmPlant =
      StateSpaceModel.createSingleJointedArmSystem(motor, System.J, System.G);

  // Observer (Kalman Filter) configuration using position measurements only
  private final KalmanFilter<N2, N1, N1> m_observer =
      new KalmanFilter<>(
          Nat.N2(),
          Nat.N1(),
          m_ArmPlant,
          VecBuilder.fill(
              Math.toRadians(1.0), Math.toRadians(15.0)), // Process noise: [position, velocity]
          VecBuilder.fill(Math.toRadians(0.01)), // Measurement noise: [position]
          Constants.ROBOT_PERIODIC_MS);

  // LQR controller configuration with Q and R weights
  private final LQRController<N2, N1, N1> m_controller =
      new LQRController<>(
          m_ArmPlant,
          VecBuilder.fill(
              Math.toRadians(1.0), Math.toRadians(5.0)), // Q weights: [position, velocity]
          VecBuilder.fill(Constants.ROBOT_NORMAL_VOLTAGE), // R weight: voltage penalty
          Constants.ROBOT_PERIODIC_MS);

  // Combined state-space control loop
  private final StateSpaceControlLoop<N2, N1, N1> m_loop =
      new StateSpaceControlLoop<>(
          m_ArmPlant,
          m_controller,
          m_observer,
          Constants.ROBOT_NORMAL_VOLTAGE,
          Constants.ROBOT_PERIODIC_MS);

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

    IDearLog.getInstance()
        .addField(
            "gravityFeedforwardVoltage",
            () -> gravityFeedforwardVoltage * Math.cos(Math.toRadians(getJointedArmAngleDegrees())),
            IDearLog.FieldType.NON_CAN);

    IDearLog.getInstance()
        .addField(
            "EleXhatP",
            () -> Math.toDegrees(m_loop.getXHat().get(0, 0)),
            IDearLog.FieldType.NON_CAN);

    IDearLog.getInstance()
        .addField(
            "EleXhatV",
            () -> Math.toDegrees(m_loop.getXHat().get(1, 0)),
            IDearLog.FieldType.NON_CAN);

    IDearLog.getInstance()
        .addField("leftVot", () -> leftJointedArmMotor.getBusVoltage(), IDearLog.FieldType.NON_CAN);

    m_loop.reset(
        VecBuilder.fill(
            Math.toRadians(leftJointedArmMotor.getAbsoluteEncoder().getPosition()),
            Math.toRadians(leftJointedArmMotor.getAbsoluteEncoder().getVelocity())));

    m_lastProfiledReference =
        new TrapezoidProfile.State(
            Math.toRadians(leftJointedArmMotor.getAbsoluteEncoder().getPosition()),
            Math.toRadians(leftJointedArmMotor.getAbsoluteEncoder().getVelocity()));

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Units.Volts.of(0.5).per(Units.Second), Units.Volts.of(1.25), Units.Seconds.of(4)),
            new SysIdRoutine.Mechanism(this::moveWithVoltage, this::logTelemetry, this));
    // controller.enableContinuousInput(-Math.PI, Math.PI);

  }

  private void moveWithVoltage(Voltage voltage) {
    setVoltage(voltage.in(Units.Volts));
  }

  private void logTelemetry(SysIdRoutineLog log) {
    log.motor("Arm")
        .voltage(Voltage.ofBaseUnits(leftJointedArmMotor.getBusVoltage(), Units.Volts))
        .angularVelocity(
            AngularVelocity.ofBaseUnits(
                leftJointedArmMotor.getAbsoluteEncoder().getVelocity(), Units.DegreesPerSecond))
        .angularPosition(
            Angle.ofBaseUnits(
                leftJointedArmMotor.getAbsoluteEncoder().getPosition(), Units.Degrees));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
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

  public void setPower(double power) {
    leftJointedArmMotor.set(power);
    rightJointedArmMotor.set(power);
  }

  @Override
  public void periodic() {
    // m_loop.correct(VecBuilder.fill(Math.toRadians(getJointedArmAngleDegrees())));
    // m_loop.predict(Constants.ROBOT_PERIODIC_MS);
  }

  public void setReference(double desiredPosition) {

    TrapezoidProfile.State goal =
        new TrapezoidProfile.State(Math.toRadians(getJointedArmAngleDegrees()), 0);

    m_lastProfiledReference =
        m_profile.calculate(Constants.ROBOT_PERIODIC_MS, m_lastProfiledReference, goal);

    // Set the next reference state: [desired position, desired velocity]
    m_loop.setNextR(Math.toRadians(desiredPosition), m_lastProfiledReference.velocity);

    // Update the state estimate using the measured position
    m_loop.correct(VecBuilder.fill(Math.toRadians(getJointedArmAngleDegrees())));
    m_loop.predict(Constants.ROBOT_PERIODIC_MS);

    // Calculate feedback voltage from the LQR controller
    double feedbackVoltage = m_loop.getU(0);
    // Retrieve constant feedforward voltage for gravity compensation
    double feedforwardVoltage =
        gravityFeedforwardVoltage * Math.cos(Math.toRadians(getJointedArmAngleDegrees()));
    double totalVoltage = feedbackVoltage + feedforwardVoltage;

    // Clamp the total voltage to within allowed limits
    totalVoltage = Math.max(-5, Math.min(5, totalVoltage));

    java.lang.System.out.println(totalVoltage);

    setVoltage(totalVoltage);
    // setPower(
    //     controller.calculate(
    //             Math.toRadians(getJointedArmAngleDegrees()), Math.toRadians(desiredPosition))
    //         + gravityFeedforwardVoltage
    //             * Math.cos(Math.toRadians(getJointedArmAngleDegrees()))
    //             / 12);

  }

  public double getJointedArmAngleDegrees() {
    return leftJointedArmMotor.getAbsoluteEncoder().getPosition();
  }

  public Command setVoltageCommand(double voltage) {
    return Commands.runEnd(() -> setVoltage(voltage), () -> stop(), this);
  }

  public Command holdingCommand() {
    return Commands.runEnd(() -> holdPosition(), () -> stop(), this);
  }

  public Command setTargetCommand(double target) {
    return Commands.runEnd(() -> setReference(target), () -> stop(), this);
  }
}
