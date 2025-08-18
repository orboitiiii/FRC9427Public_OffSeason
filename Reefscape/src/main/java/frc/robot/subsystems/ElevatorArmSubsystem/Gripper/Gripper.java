package frc.robot.subsystems.ElevatorArmSubsystem.Gripper;

import com.ctre.phoenix6.hardware.TalonFX;
import com.slsh.IDeer9427.lib.data.IDearLog;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {
  private final TalonFX gripper = new TalonFX(GripperConstants.motorID);

  private static final double kTakeVoltage = 5.0;
  private static final double kHoldVoltage = 0.50;
  private static final double kAlgaeCurrentA = 25.0;
  private static final long kDebounceUs = 60_000L; // 60 ms

  private WantedState wantedState = WantedState.IDLE;
  private WantedState prevState = WantedState.IDLE;

  private boolean gripLatched = false;
  private Long overThreshStartUs = null;

  private double lastVoltage = Double.NaN;
  private static final double kVoltEps = 0.02;

  public Gripper() {
    gripper.getConfigurator().apply(GripperConfig.gripperMotorConfig());
    IDearLog.getInstance()
        .addField(
            "gripperStatorCurrent",
            () -> gripper.getStatorCurrent().getValueAsDouble(),
            IDearLog.FieldType.CAN);
  }

  public enum WantedState {
    IDLE,
    TAKEING_ALGER,
    TAKEING_CORAL,
    HOLDALGEA,
    SHOOT
  }

  private void setVoltageIfChanged(double v) {
    if (Double.isNaN(lastVoltage) || Math.abs(v - lastVoltage) > kVoltEps) {
      gripper.setVoltage(v);
      lastVoltage = v;
    }
  }

  @Override
  public void periodic() {
    if (wantedState != prevState) {
      onEnter(wantedState);
      prevState = wantedState;
    }
    applyStateActions();
  }

  private void onEnter(WantedState s) {
    switch (s) {
      case TAKEING_ALGER:
        gripLatched = false;
        overThreshStartUs = null;
        setVoltageIfChanged(kTakeVoltage);
        break;
      case HOLDALGEA:
        setVoltageIfChanged(kHoldVoltage);
        break;
      case TAKEING_CORAL:
        setVoltageIfChanged(4.0);
        break;
      case SHOOT:
        setVoltageIfChanged(-7.0);
        break;
      case IDLE:
      default:
        setVoltageIfChanged(0.0);
        break;
    }
  }

  private void applyStateActions() {
    switch (wantedState) {
      case IDLE:
        return;

      case TAKEING_ALGER:
        {
          double iStatorA = gripper.getStatorCurrent().getValueAsDouble();

          long nowUs = RobotController.getFPGATime();
          boolean over = iStatorA >= kAlgaeCurrentA;

          if (over) {
            if (overThreshStartUs == null) overThreshStartUs = nowUs;
            if (!gripLatched && (nowUs - overThreshStartUs) >= kDebounceUs) {
              gripLatched = true;
            }
          } else {
            overThreshStartUs = null;
          }

          if (gripLatched) {
            setVoltageIfChanged(kHoldVoltage);
            wantedState = WantedState.HOLDALGEA;
          }
          return;
        }

      case TAKEING_CORAL:
        return;

      case HOLDALGEA:
        return;

      case SHOOT:
        return;
    }
  }

  public void setState(WantedState state) {
    wantedState = state;
  }

  public Command setStateCommand(WantedState state) {
    return Commands.runOnce(() -> setState(state), this);
  }
}
