package frc.robot.subsystems.ElevatorArmSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ElevatorArmSubsystem.Constamts.SetpointFactory;
import frc.robot.subsystems.ElevatorArmSubsystem.JointedArm.JointedArmSubsystem;
import frc.robot.subsystems.ElevatorArmSubsystem.LinearExtension.LinearExtensionSubSystem;

public class ElevatorArmSubsystem extends SubsystemBase {

  public enum WantedState {
    IDLE,
    MOVE_TO_PLACING_CORAL, // Corrected typo from PlACEING_CORAOL for clarity, assuming it's
    // PLACING_CORAL
    MOVE_TO_SCORE_TAKE_ALGAE,
    MOVE_TO_HOME
  }

  private enum SystemState {
    IDLING,
    MOVING_ELEVATOR_TO_SAFETY_POSITION,
    MOVING_ARM_TO_SAFETY_POSITION,
    MOVING_ELEVATOR_FIRST,
    MOVING_ARM_FIRST,
    MOVING_BOTH
  }

  private WantedState wantedState = WantedState.IDLE;
  private WantedState previousWantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLING;

  private final JointedArmSubsystem jointedArmSubsystem;
  private final LinearExtensionSubSystem linearExtensionSubSystem;

  private ElevatorArmPosition wantedElevatorArmPosition;

  private double safetyArmAngle =
      SetpointFactory.SAFETY_ARM_POSITION
          .getJointedArmAngleDegrees(); // Dynamic safety arm angle, default to fixed safety

  private double safetyElevatorPosition =
      SetpointFactory.SAFETY_ELEVATOR_POSITION.getExtensionLengthMeters();

  public ElevatorArmSubsystem(
      JointedArmSubsystem jointedArmSubsystem, LinearExtensionSubSystem linearExtensionSubSystem) {
    this.jointedArmSubsystem = jointedArmSubsystem;
    this.linearExtensionSubSystem = linearExtensionSubSystem;
    wantedElevatorArmPosition = SetpointFactory.ZEROED;
  }

  @Override
  public void periodic() {
    systemState = handleStateTransitions();
    applyStateActions();
    previousWantedState = this.wantedState;
  }

  private SystemState handleStateTransitions() {
    if (wantedState == WantedState.IDLE) {
      return SystemState.IDLING;
    }

    double currentElevator = linearExtensionSubSystem.getLinearExtensionMeters();
    double wantedElevator = wantedElevatorArmPosition.getExtensionLengthMeters();

    double currentArmDeg = jointedArmSubsystem.getJointedArmAngleDegrees();
    double wantedArmDeg = wantedElevatorArmPosition.getJointedArmAngleDegrees();

    // General conditions regardless of wantedState
    // If current arm > 100 degrees and elevator needs to go up, move arm to 90 degrees first
    if (currentArmDeg > 100 && wantedElevator > currentElevator) {
      safetyArmAngle = 90.0; // As specified, move to 90 degrees
      return SystemState.MOVING_ARM_TO_SAFETY_POSITION;
    }

    // If elevator near 0, arm near -90, and arm needs to raise up, move elevator to safety (0.15m)
    // first
    if (MathUtil.isNear(currentElevator, 0.0, 0.08)
        && MathUtil.isNear(currentArmDeg, -90.0, 5.0)
        && wantedArmDeg > currentArmDeg) {
      return SystemState.MOVING_ELEVATOR_TO_SAFETY_POSITION;
    }

    // State-specific transitions
    if (wantedState == WantedState.MOVE_TO_PLACING_CORAL) {
      // No additional specific conditions beyond generals; move both
      return SystemState.MOVING_BOTH;
    }

    if (wantedState == WantedState.MOVE_TO_SCORE_TAKE_ALGAE) {
      if (wantedElevator > 1.0 && !MathUtil.isNear(currentArmDeg, 90, 2.0)) {
        safetyArmAngle = 90.0;
        return SystemState.MOVING_ARM_TO_SAFETY_POSITION;
      } else if (MathUtil.isNear(currentArmDeg, 90, 2.0)
          && wantedElevator > 1.0
          && !MathUtil.isNear(currentElevator, wantedElevator, 0.02)) {
        return SystemState.MOVING_ELEVATOR_FIRST;
      }
      // No additional specific conditions; move both
      return SystemState.MOVING_BOTH;
    }

    if (wantedState == WantedState.MOVE_TO_HOME) {
      // Specific for previous state SCORE_OR_TAKE_ALGAE and elevator >1.5m descending
      if (previousWantedState == WantedState.MOVE_TO_SCORE_TAKE_ALGAE
          && currentElevator > 1.0
          && currentElevator > wantedElevator) {
        if (!MathUtil.isNear(currentArmDeg, 90, 2.0) && !MathUtil.isNear(currentArmDeg, -90, 2.0)) {
          // Choose nearest safe arm position (90 or -90)
          double distTo90 = Math.abs(currentArmDeg - 90);
          double distToNeg90 = Math.abs(currentArmDeg + 90);
          safetyArmAngle = (distTo90 <= distToNeg90) ? 90.0 : -90.0;
          return SystemState.MOVING_ARM_TO_SAFETY_POSITION;
        } else {
          // Already at a safe position; if at 90 but wanted is -90, descend holding arm at 90
          if (MathUtil.isNear(currentArmDeg, 90, 2.0) && !MathUtil.isNear(wantedArmDeg, 90, 2.0)) {
            return SystemState.MOVING_ELEVATOR_FIRST; // Descend elevator, hold arm at current safe
          }
        }
      }

      // Sequence for moving to home (-90 arm, 0 elevator) when elevator in 0 to 0.15m
      if (MathUtil.isNear(wantedArmDeg, -90.0, 2.0) && MathUtil.isNear(wantedElevator, 0.0, 0.01)) {
        if (currentElevator < 0.14) { // If below safety threshold, raise to 0.15m first
          return SystemState.MOVING_ELEVATOR_TO_SAFETY_POSITION;
        }
        if (MathUtil.isNear(currentElevator, 0.15, 0.02)) { // At safety height
          if (!MathUtil.isNear(currentArmDeg, wantedArmDeg, 2.0)) {
            return SystemState.MOVING_ARM_FIRST; // Move arm to -90, hold elevator
          } else {
            return SystemState.MOVING_ELEVATOR_FIRST; // Arm ready, lower elevator to 0, hold arm
          }
        }
      }

      // Other conditions: move both simultaneously
      return SystemState.MOVING_BOTH;
    }

    return SystemState.MOVING_BOTH;
  }

  private void applyStateActions() {
    switch (systemState) {
      case IDLING:
        jointedArmSubsystem.stop();
        linearExtensionSubSystem.stop();
        break;

      case MOVING_ELEVATOR_TO_SAFETY_POSITION:
        jointedArmSubsystem.holdPosition();
        linearExtensionSubSystem.setReference(safetyElevatorPosition);
        break;

      case MOVING_ARM_TO_SAFETY_POSITION:
        linearExtensionSubSystem.holdPosition();
        jointedArmSubsystem.setReference(safetyArmAngle); // Use dynamic safety angle
        break;

      case MOVING_ARM_FIRST:
        jointedArmSubsystem.setReference(wantedElevatorArmPosition.getJointedArmAngleDegrees());
        linearExtensionSubSystem.holdPosition();
        break;

      case MOVING_ELEVATOR_FIRST:
        linearExtensionSubSystem.setReference(wantedElevatorArmPosition.getExtensionLengthMeters());
        jointedArmSubsystem.holdPosition();
        break;

      case MOVING_BOTH:
        jointedArmSubsystem.setReference(wantedElevatorArmPosition.getJointedArmAngleDegrees());
        linearExtensionSubSystem.setReference(wantedElevatorArmPosition.getExtensionLengthMeters());
        break;
    }
  }

  public void setWantedState(WantedState wantedState, ElevatorArmPosition position) {
    this.wantedState = wantedState;
    this.wantedElevatorArmPosition = position;
  }

  public void setIdle() {
    this.wantedState = WantedState.IDLE;
  }
}
