package frc.robot.subsystems.IntakeSubsystem.RollerAndIndexerSystem;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.core.CoreCANrange;
import com.revrobotics.spark.SparkFlex;
import com.slsh.IDeer9427.lib.data.IDearLog;
import com.slsh.IDeer9427.lib.data.IDearLog.FieldType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkFlexConfiguration;

public class RollerAndIndexer extends SubsystemBase {

  final int kCANrangeId = 40;
  final int leftID = 32;
  final int rightID = 31;
  final int rollerID = 61;

  public final CoreCANrange m_range1 = new CoreCANrange(kCANrangeId);
  final CANrangeConfiguration configs = new CANrangeConfiguration();

  private final SparkFlex leftIndexerMotor;
  private final SparkFlex rightIndexerMotor;
  private final SparkFlex rollerMotor;

  private final SparkFlexConfiguration motorConfiguration =
      SparkFlexConfiguration.createRollerAndIndexerMotor(leftID, rightID, rollerID);

  private static final double kAmbientThresh = 4.0;
  private static final long kNearDebounceUs = 60_000; // 60 ms
  private static final long kAutoTimeoutUs = 10_000_000; // 10 s

  private static final double kRollerVolts = 4.0;
  private static final double kIndexerVoltsL = 2.0;
  private static final double kIndexerVoltsR = -2.0;

  public enum WantedState {
    IDLE,
    INTAKE_AUTO,
    INTAKE_ONLY
  }

  private WantedState state = WantedState.IDLE;
  private WantedState prev = WantedState.IDLE;

  private long autoStartUs = 0;
  private Long underStartUs = null;

  private double lastVL = Double.NaN, lastVR = Double.NaN, lastVRoll = Double.NaN;
  private static final double kVoltEps = 0.02;

  public volatile boolean hasCoral = false;

  public RollerAndIndexer() {
    leftIndexerMotor = motorConfiguration.motors()[0];
    rightIndexerMotor = motorConfiguration.motors()[1];
    rollerMotor = motorConfiguration.motors()[2];

    m_range1.getAmbientSignal().setUpdateFrequency(400); // 400 Hz

    IDearLog.getInstance()
        .addField(
            "AmbientSignal", () -> m_range1.getAmbientSignal().getValueAsDouble(), FieldType.CAN);
  }

  public void startAutoIntake() {
    setState(WantedState.INTAKE_AUTO);
  }

  public void startIntakeOnly() {
    setState(WantedState.INTAKE_ONLY);
  }

  public void stop() {
    setState(WantedState.IDLE);
  }

  @Override
  public void periodic() {
    if (state != prev) {
      onEnter(state);
      prev = state;
    }
    updateHasCoral();
    apply();
    SmartDashboard.putBoolean("hasCoral", hasCoral);
  }

  private void onEnter(WantedState s) {
    switch (s) {
      case IDLE:
        setVoltages(0, 0, 0);
        underStartUs = null;
        break;
      case INTAKE_ONLY:
        setVoltages(kIndexerVoltsL, kIndexerVoltsR, kRollerVolts);
        underStartUs = null;
        break;
      case INTAKE_AUTO:
        setVoltages(kIndexerVoltsL, kIndexerVoltsR, kRollerVolts);
        autoStartUs = RobotController.getFPGATime();
        underStartUs = null;
        break;
    }
  }

  private void apply() {
    switch (state) {
      case IDLE:
        return;

      case INTAKE_ONLY:
        return;

      case INTAKE_AUTO:
        long nowUs = RobotController.getFPGATime();

        if (hasCoral) {
          setState(WantedState.IDLE);
          return;
        }
        if ((nowUs - autoStartUs) >= kAutoTimeoutUs) {
          setState(WantedState.IDLE);
        }
        return;
    }
  }

  public void setState(WantedState s) {
    state = s;
  }

  private void setVoltages(double vL, double vR, double vRoll) {
    if (Double.isNaN(lastVL) || Math.abs(vL - lastVL) > kVoltEps) {
      leftIndexerMotor.setVoltage(vL);
      lastVL = vL;
    }
    if (Double.isNaN(lastVR) || Math.abs(vR - lastVR) > kVoltEps) {
      rightIndexerMotor.setVoltage(vR);
      lastVR = vR;
    }
    if (Double.isNaN(lastVRoll) || Math.abs(vRoll - lastVRoll) > kVoltEps) {
      rollerMotor.setVoltage(vRoll);
      lastVRoll = vRoll;
    }
  }

  private void updateHasCoral() {
    double ambient = m_range1.getAmbientSignal().getValueAsDouble();
    long nowUs = RobotController.getFPGATime();
    boolean under = ambient < kAmbientThresh;

    if (under) {
      if (underStartUs == null) underStartUs = nowUs;
      if ((nowUs - underStartUs) >= kNearDebounceUs) {
        hasCoral = true;
      }
    } else {
      underStartUs = null;
      hasCoral = false;
    }
  }

  public Command setStateCommand(WantedState wantedState) {
    return Commands.runOnce(() -> setState(wantedState), this);
  }
}
