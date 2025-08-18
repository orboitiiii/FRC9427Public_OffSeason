package frc.robot.subsystems.ElevatorArmSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ElevatorArmSubsystem.Constants.SetpointFactory;
import frc.robot.subsystems.ElevatorArmSubsystem.JointedArm.JointedArmSubsystem;
import frc.robot.subsystems.ElevatorArmSubsystem.LinearExtension.LinearExtensionSubSystem;

public class ElevatorArmSubsystem extends SubsystemBase {

  public enum WantedState {
    IDLE,
    MOVING_BOTH,
    TAKEING_CORAL
  }

  private enum SystemState {
    IDLING,
    MOVING_BOTH
  }

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLING;

  private final JointedArmSubsystem jointedArmSubsystem;
  private final LinearExtensionSubSystem linearExtensionSubSystem;

  private ElevatorArmPosition wantedElevatorArmPosition;

  public ElevatorArmSubsystem(
      JointedArmSubsystem jointedArmSubsystem, LinearExtensionSubSystem linearExtensionSubSystem) {
    this.jointedArmSubsystem = jointedArmSubsystem;
    this.linearExtensionSubSystem = linearExtensionSubSystem;
    wantedElevatorArmPosition = SetpointFactory.TAKEING_CORAL;
  }

  @Override
  public void periodic() {
    systemState = handleStateTransitions();
    applyStateActions();
    SmartDashboard.putBoolean("isZero", isZero());
  }

  private SystemState handleStateTransitions() {
    return (wantedState == WantedState.IDLE) ? SystemState.IDLING : SystemState.MOVING_BOTH;
  }

  private void applyStateActions() {
    switch (systemState) {
      case IDLING:
        jointedArmSubsystem.stop();
        linearExtensionSubSystem.stop();
        return;

      case MOVING_BOTH:
        final double currElev = linearExtensionSubSystem.getLinearExtensionMeters();
        final double tgtElev = wantedElevatorArmPosition.getExtensionLengthMeters();
        final double currArmDeg = jointedArmSubsystem.getJointedArmAngleDegrees();
        final double tgtArmDeg = wantedElevatorArmPosition.getJointedArmAngleDegrees();

        if (wantedState != WantedState.TAKEING_CORAL) {
          if (tgtElev > 0.69 && tgtArmDeg > 21 && !MathUtil.isNear(currElev, tgtElev, 0.01)) {
            jointedArmSubsystem.setReference(10);
            if (currArmDeg < 25) {
              linearExtensionSubSystem.setReference(tgtElev);
            } else {
              linearExtensionSubSystem.holdPosition();
            }
            return;
          }

          if (tgtElev < 0.25 && tgtArmDeg > -80 && !MathUtil.isNear(currElev, tgtElev, 0.01)) {
            jointedArmSubsystem.setReference(tgtArmDeg);
            if (tgtArmDeg > -60) {
              linearExtensionSubSystem.setReference(tgtElev);
            } else {
              linearExtensionSubSystem.holdPosition();
            }
            return;
          }

          if (currElev < 0.25 && tgtArmDeg < -80) {
            linearExtensionSubSystem.setReference(tgtElev);
            if (MathUtil.isNear(currElev, tgtElev, 0.01)) {
              jointedArmSubsystem.setReference(tgtArmDeg);
            } else {
              jointedArmSubsystem.holdPosition();
            }
            return;
          }
        }
        jointedArmSubsystem.setReference(tgtArmDeg);
        linearExtensionSubSystem.setReference(tgtElev);

        return;
    }
  }

  public void setWantedState(WantedState wantedState, ElevatorArmPosition position) {
    this.wantedState = wantedState;
    this.wantedElevatorArmPosition = position;
  }

  public Command setWantedStateCommand(
      WantedState wantedState, ElevatorArmPosition wantedElevatorArmPosition) {
    return Commands.runOnce(() -> setWantedState(wantedState, wantedElevatorArmPosition), this);
  }

  public void setIdle() {
    this.wantedState = WantedState.IDLE;
  }

  public boolean isAtSetPoint() {
    final double currElev = linearExtensionSubSystem.getLinearExtensionMeters();
    final double tgtElev = wantedElevatorArmPosition.getExtensionLengthMeters();
    final double currArm = jointedArmSubsystem.getJointedArmAngleDegrees();
    final double tgtArm = wantedElevatorArmPosition.getJointedArmAngleDegrees();

    return MathUtil.isNear(currElev, tgtElev, 0.010) && MathUtil.isNear(currArm, tgtArm, 5.0);
  }

  public boolean isZero() {
    final double currElev = linearExtensionSubSystem.getLinearExtensionMeters();
    final double tgtElev = SetpointFactory.TAKEING_CORAL.getExtensionLengthMeters();
    final double currArm = jointedArmSubsystem.getJointedArmAngleDegrees();
    final double tgtArm = SetpointFactory.TAKEING_CORAL.getJointedArmAngleDegrees();

    return MathUtil.isNear(currElev, tgtElev, 0.010) && MathUtil.isNear(currArm, tgtArm, 5.0);
  }

  public boolean isHome() {
    final double currElev = linearExtensionSubSystem.getLinearExtensionMeters();
    final double tgtElev = SetpointFactory.CORALHOME.getExtensionLengthMeters();
    final double currArm = jointedArmSubsystem.getJointedArmAngleDegrees();
    final double tgtArm = SetpointFactory.CORALHOME.getJointedArmAngleDegrees();

    return MathUtil.isNear(currElev, tgtElev, 0.010) && MathUtil.isNear(currArm, tgtArm, 5.0);
  }

  public boolean isL4() {
    final double currElev = linearExtensionSubSystem.getLinearExtensionMeters();
    final double tgtElev = SetpointFactory.PRE_L4.getExtensionLengthMeters();
    final double currArm = jointedArmSubsystem.getJointedArmAngleDegrees();
    final double tgtArm = SetpointFactory.PRE_L4.getJointedArmAngleDegrees();

    return MathUtil.isNear(currElev, tgtElev, 0.010) && MathUtil.isNear(currArm, tgtArm, 5.0);
  }

  public void setEleVoltage(double v) {
    linearExtensionSubSystem.setVoltage(v);
  }

  public void setArmVoltage(double v) {
    jointedArmSubsystem.setVoltage(v);
  }

  public void eleHold() {
    linearExtensionSubSystem.holdPosition();
  }

  public void armHold() {
    jointedArmSubsystem.holdPosition();
  }
}
