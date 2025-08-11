// File: src/main/java/frc/robot/subsystems/ElevatorArmSubsystem/ElevatorArmSubsystem.java
package frc.robot.subsystems.ElevatorArmSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ElevatorArmSubsystem.Constamts.ElevatorArmConstants;
import frc.robot.subsystems.ElevatorArmSubsystem.Constamts.SetpointFactory;
import frc.robot.subsystems.ElevatorArmSubsystem.JointedArm.JointedArmSubsystem;
import frc.robot.subsystems.ElevatorArmSubsystem.LinearExtension.LinearExtensionSubSystem;

public class ElevatorArmSubsystem extends SubsystemBase {

  public enum WantedState {
    IDLE,
    MOVE_TO_PLACING_CORAL,
    MOVE_TO_SCORE_TAKE_ALGAE,
    MOVE_TO_HOME
  }

  private enum SystemState {
    IDLING,
    MOVING_BOTH
  }

  private WantedState wantedState = WantedState.IDLE;
  private WantedState previouState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLING;

  private final JointedArmSubsystem jointedArmSubsystem;
  private final LinearExtensionSubSystem linearExtensionSubSystem;

  private ElevatorArmPosition wantedElevatorArmPosition;

  // 由常數推導，避免硬編碼
  private final double safetyArmAngleDeg = ElevatorArmConstants.ARM_SAFE_DEG; // 90°
  private final double safetyElevatorMeters = ElevatorArmConstants.HOME_SAFETY_ELEV_M; // 0.15m

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
    previouState = wantedState;
  }

  /** 簡化：非 IDLE 一律進「計算目標與守門」流程 */
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
        // 讀當前與目標
        final double currElev = linearExtensionSubSystem.getLinearExtensionMeters();
        final double tgtElev = wantedElevatorArmPosition.getExtensionLengthMeters();
        final double currArmDeg = jointedArmSubsystem.getJointedArmAngleDegrees();
        final double tgtArmDeg = wantedElevatorArmPosition.getJointedArmAngleDegrees();

        // ========= 連續式守門（照 2910 思路）=========
        // A) 特定模式：ALGAE 高位動作，手臂需先到安全角 90°
        if (wantedState == WantedState.MOVE_TO_SCORE_TAKE_ALGAE
            && tgtElev > ElevatorArmConstants.SCORE_TAKE_ALGAE_ELEV_THRESHOLD_M
            && !MathUtil.isNear(
                currArmDeg, safetyArmAngleDeg, ElevatorArmConstants.TOL_ANGLE_DEG)) {

          jointedArmSubsystem.setReference(safetyArmAngleDeg);
          linearExtensionSubSystem.holdPosition();
          return;
        }

        // B) 手臂角度過大而升降目標為上升 → 先把手臂拉回安全角
        if (currArmDeg > ElevatorArmConstants.ARM_MOVE_UP_BEFORE_EXTEND_DEG
            && tgtElev > currElev
            && !MathUtil.isNear(
                currArmDeg, safetyArmAngleDeg, ElevatorArmConstants.TOL_ANGLE_DEG)) {

          jointedArmSubsystem.setReference(safetyArmAngleDeg);
          linearExtensionSubSystem.holdPosition();
          return;
        }

        // C) 升降幾乎在 0、手臂在 -90 且準備抬臂 → 先把升降抬到安全高度
        if (MathUtil.isNear(currElev, 0.0, ElevatorArmConstants.TOL_ELEV_NEAR_ZERO_M)
            && MathUtil.isNear(
                currArmDeg,
                ElevatorArmConstants.ARM_NEAR_NEG90_DEG,
                ElevatorArmConstants.TOL_ANGLE_WIDE_DEG)
            && tgtArmDeg > currArmDeg) {

          jointedArmSubsystem.holdPosition();
          linearExtensionSubSystem.setReference(safetyElevatorMeters);
          return;
        }

        // D) 回 HOME（-90°, 0m）序列化守門
        final boolean targetIsHome =
            MathUtil.isNear(
                    tgtArmDeg,
                    ElevatorArmConstants.ARM_NEAR_NEG90_DEG,
                    ElevatorArmConstants.TOL_ANGLE_DEG)
                && MathUtil.isNear(tgtElev, 0.0, ElevatorArmConstants.TOL_ELEV_ZERO_TARGET_M);

        if (targetIsHome) {
          if (previouState == WantedState.MOVE_TO_SCORE_TAKE_ALGAE) {
            if (!MathUtil.isNear(currArmDeg, 90, ElevatorArmConstants.TOL_ANGLE_DEG)) {
              jointedArmSubsystem.setReference(safetyArmAngleDeg);
              linearExtensionSubSystem.holdPosition();
            }
          }
          // 低於安全高度先抬到安全高度
          if (currElev < ElevatorArmConstants.HOME_SAFETY_ELEV_PRELIFT_BOUND_M) {
            jointedArmSubsystem.holdPosition();
            linearExtensionSubSystem.setReference(safetyElevatorMeters);
            return;
          }
          // 在安全高度附近：先把手臂到 -90，再降到 0
          if (!MathUtil.isNear(currArmDeg, tgtArmDeg, ElevatorArmConstants.TOL_ANGLE_DEG)) {
            jointedArmSubsystem.setReference(tgtArmDeg);
            linearExtensionSubSystem.holdPosition();
            return;
          } else {
            jointedArmSubsystem.holdPosition();
            linearExtensionSubSystem.setReference(0.0);
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

  public void setIdle() {
    this.wantedState = WantedState.IDLE;
  }
}
