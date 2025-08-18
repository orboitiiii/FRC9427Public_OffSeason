package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorArmSubsystem.ElevatorArmSubsystem;
import frc.robot.subsystems.ElevatorArmSubsystem.ElevatorArmSubsystem.WantedState;
import frc.robot.subsystems.ElevatorArmSubsystem.Constants.SetpointFactory;
import frc.robot.subsystems.ElevatorArmSubsystem.Gripper.Gripper;
import frc.robot.subsystems.IntakeSubsystem.PivotSystem.IntakePivot;
import frc.robot.subsystems.IntakeSubsystem.RollerAndIndexerSystem.RollerAndIndexer;

public class Superstructure extends SubsystemBase {

  private final Gripper gripper;
  private final ElevatorArmSubsystem elevatorArmSubsystem;
  private final IntakePivot intakePivot;
  private final RollerAndIndexer rollerAndIndexer;
  private final RobotContainer robotContainer;

  WantedBigArmState wantedBigArmState = WantedBigArmState.MOVE_TO_HOME;
  SystemBigArmState systemBigArmState = SystemBigArmState.IDLE;
  public SystemBigArmState previousBigArmState = SystemBigArmState.IDLE;

  WantedIntakePivotState wantedIntakePivotState = WantedIntakePivotState.UP;
  SystemIntakePivotState systemIntakePivotState = SystemIntakePivotState.IDLE;
  SystemIntakePivotState previousIntakePivotState = SystemIntakePivotState.IDLE;

  private static final long kZeroHoldUs = 60_000;
  private static final long kGrabTimeoutUs = 1_000_000;
  private static final long kHomeHoldUs = 30_000;
  private static final long kRetryCooldownUs = 120_000;

  private Long zeroHoldStartUs = null;
  private Long takingStartUs = null;
  private Long homeHoldStartUs = null;
  private long retryCooldownUntilUs = 0;

  private boolean coralSeqActive = false;

  private boolean disableAlgaeHome = false;

  private boolean l1Prev = false;
  private int l1ReleaseCount = 0;

  public BooleanSupplier hasCoral;

  public BooleanSupplier isAtSetPoint;

  public BooleanSupplier isHome;

  public BooleanSupplier isL4;

  public Superstructure(
      ElevatorArmSubsystem m_elevatorArmSubsystem,
      IntakePivot m_intakePivot,
      RollerAndIndexer m_rollerAndIndexer,
      Gripper m_Gripper,
      RobotContainer m_RobotContainer) {
    elevatorArmSubsystem = m_elevatorArmSubsystem;
    intakePivot = m_intakePivot;
    rollerAndIndexer = m_rollerAndIndexer;
    gripper = m_Gripper;
    robotContainer = m_RobotContainer;
    hasCoral = () -> rollerAndIndexer.hasCoral;
    isAtSetPoint = () -> elevatorArmSubsystem.isAtSetPoint();
    isHome = () -> elevatorArmSubsystem.isHome();
    isL4 = () -> elevatorArmSubsystem.isL4();
  }

  @Override
  public void periodic() {
    systemBigArmState = handleBigArmStateTransitions();
    systemIntakePivotState = handleIntakePivotStateTransitions();
    applyBigArmState();
    applyIntakePivotState();
    previousBigArmState = systemBigArmState;
    previousIntakePivotState = systemIntakePivotState;
    SmartDashboard.putString("systemBigArmState", systemBigArmState.name());
    SmartDashboard.putString(" previousBigArmState", previousBigArmState.name());
    SmartDashboard.putBoolean("disableAlgaeHome", disableAlgaeHome);
  }

  public enum WantedBigArmState {
    MANUAL,
    IDLE,
    MOVE_TO_HOME,
    TAKEING_ALGAE_FLOOR,
    TAKEING_ALGAE_REEF_LOW,
    TAKEING_ALGAE_REEF_HIGH,
    NET,
    PROCESSOR,
    PRE_L1,
    PRE_L2,
    PRE_L3,
    PRE_L4,
    PLACE_L1,
    PLACE_L2,
    PLACE_L3,
    PLACE_L4
  }

  private enum SystemBigArmState {
    MANUAL,
    IDLE,
    TAKING_CORAL,
    MOVE_TO_CORAL_HOME,
    MOVE_TO_ALGAE_HOME,
    TAKEING_ALGAE_FLOOR,
    TAKEING_ALGAE_REEF_LOW,
    TAKEING_ALGAE_REEF_HIGH,
    NET,
    PROCESSOR,
    PRE_L1,
    PRE_L2,
    PRE_L3,
    PRE_L4,
    PLACE_L1,
    PLACE_L2,
    PLACE_L3,
    PLACE_L4
  }

  public enum WantedIntakePivotState {
    DOWN,
    UP,
    IDLE
  }

  private enum SystemIntakePivotState {
    DOWN,
    UP,
    IDLE
  }

  private SystemIntakePivotState handleIntakePivotStateTransitions() {
    switch (wantedIntakePivotState) {
      case DOWN:
        return SystemIntakePivotState.DOWN;
      case IDLE:
        return SystemIntakePivotState.IDLE;
      case UP:
        return SystemIntakePivotState.UP;
      default:
        return SystemIntakePivotState.IDLE;
    }
  }

  private void applyIntakePivotState() {
    boolean l1 = robotContainer.commandPS5Controller.L1().getAsBoolean();

    if (l1Prev && !l1) {
      l1ReleaseCount++;

      if ((l1ReleaseCount & 1) == 1) {
        intakePivot.setTargetAngleRad(Math.toRadians(0));
        rollerAndIndexer.setState(
            frc.robot.subsystems.IntakeSubsystem.RollerAndIndexerSystem.RollerAndIndexer.WantedState
                .INTAKE_AUTO);
      } else {
        intakePivot.setTargetAngleRad(Math.toRadians(125));
        rollerAndIndexer.setState(
            frc.robot.subsystems.IntakeSubsystem.RollerAndIndexerSystem.RollerAndIndexer.WantedState
                .IDLE);
      }
    }

    l1Prev = l1;
  }

  private SystemBigArmState handleBigArmStateTransitions() {

    if (previousBigArmState == SystemBigArmState.MOVE_TO_CORAL_HOME) {
      long now = RobotController.getFPGATime();
      boolean atHome = elevatorArmSubsystem.isAtSetPoint();
      if (atHome) {
        if (homeHoldStartUs == null) homeHoldStartUs = now;
      } else {
        homeHoldStartUs = null;
      }
      boolean homeStable = (homeHoldStartUs != null) && (now - homeHoldStartUs >= kHomeHoldUs);

      if (!coralSeqActive
          && now >= retryCooldownUntilUs
          && homeStable
          && rollerAndIndexer.hasCoral) {
        coralSeqActive = true;
        homeHoldStartUs = null;
        zeroHoldStartUs = null;
        takingStartUs = now;
        return SystemBigArmState.TAKING_CORAL;
      }
    }

    if (previousBigArmState == SystemBigArmState.TAKING_CORAL) {
      long now = RobotController.getFPGATime();

      if (elevatorArmSubsystem.isZero()) {
        if (zeroHoldStartUs == null) zeroHoldStartUs = now;
        if (now - zeroHoldStartUs >= kZeroHoldUs) {
          zeroHoldStartUs = null;
          takingStartUs = null;
          coralSeqActive = false;
          retryCooldownUntilUs = now + kRetryCooldownUs;
          return SystemBigArmState.MOVE_TO_CORAL_HOME;
        }
      } else {
        zeroHoldStartUs = null;
      }

      if (takingStartUs != null && now - takingStartUs >= kGrabTimeoutUs) {
        coralSeqActive = false;
        takingStartUs = null;
        return SystemBigArmState.MOVE_TO_CORAL_HOME;
      }

      return SystemBigArmState.TAKING_CORAL;
    }

    switch (wantedBigArmState) {
      case MANUAL:
        return SystemBigArmState.MANUAL;
      case IDLE:
        return SystemBigArmState.IDLE;
      case MOVE_TO_HOME:
        if ((previousBigArmState == SystemBigArmState.TAKEING_ALGAE_FLOOR
                || previousBigArmState == SystemBigArmState.TAKEING_ALGAE_REEF_HIGH
                || previousBigArmState == SystemBigArmState.TAKEING_ALGAE_REEF_LOW
                || previousBigArmState == SystemBigArmState.MOVE_TO_ALGAE_HOME)
            && !disableAlgaeHome) {
          return SystemBigArmState.MOVE_TO_ALGAE_HOME;
        }
        return SystemBigArmState.MOVE_TO_CORAL_HOME;
      case TAKEING_ALGAE_FLOOR:
        return SystemBigArmState.TAKEING_ALGAE_FLOOR;
      case TAKEING_ALGAE_REEF_HIGH:
        return SystemBigArmState.TAKEING_ALGAE_REEF_HIGH;
      case TAKEING_ALGAE_REEF_LOW:
        return SystemBigArmState.TAKEING_ALGAE_REEF_LOW;
      case NET:
        return SystemBigArmState.NET;
      case PROCESSOR:
        return SystemBigArmState.PROCESSOR;
      case PRE_L1:
        return SystemBigArmState.PRE_L1;
      case PRE_L2:
        return SystemBigArmState.PRE_L2;
      case PRE_L3:
        return SystemBigArmState.PRE_L3;
      case PRE_L4:
        return SystemBigArmState.PRE_L4;
      case PLACE_L1:
        return SystemBigArmState.PLACE_L1;
      case PLACE_L2:
        return SystemBigArmState.PLACE_L2;
      case PLACE_L3:
        return SystemBigArmState.PLACE_L3;
      case PLACE_L4:
        return SystemBigArmState.PLACE_L4;
      default:
        return SystemBigArmState.IDLE;
    }
  }

  private void applyBigArmState() {
    switch (systemBigArmState) {
      case MANUAL:
        if (robotContainer.commandPS5Controller.pov(0).getAsBoolean()) {
          elevatorArmSubsystem.setEleVoltage(1.5);
        } else if (robotContainer.commandPS5Controller.pov(180).getAsBoolean()) {
          elevatorArmSubsystem.setEleVoltage(-1.5);
        } else {
          elevatorArmSubsystem.eleHold();
        }

        if (robotContainer.commandPS5Controller.pov(90).getAsBoolean()) {
          elevatorArmSubsystem.setArmVoltage(1.25);
        } else if (robotContainer.commandPS5Controller.pov(270).getAsBoolean()) {
          elevatorArmSubsystem.setArmVoltage(-1.25);
        } else {
          elevatorArmSubsystem.armHold();
        }

        break;
      case IDLE:
        elevatorArmSubsystem.setWantedState(WantedState.IDLE, null);
        break;
      case TAKING_CORAL:
        gripper.setState(
            frc.robot.subsystems.ElevatorArmSubsystem.Gripper.Gripper.WantedState.TAKEING_CORAL);
        elevatorArmSubsystem.setWantedState(
            WantedState.TAKEING_CORAL, SetpointFactory.TAKEING_CORAL);
        break;
      case MOVE_TO_CORAL_HOME:
        gripper.setState(
            frc.robot.subsystems.ElevatorArmSubsystem.Gripper.Gripper.WantedState.IDLE);
        elevatorArmSubsystem.setWantedState(WantedState.MOVING_BOTH, SetpointFactory.CORALHOME);
        break;
      case MOVE_TO_ALGAE_HOME:
        elevatorArmSubsystem.setWantedState(WantedState.MOVING_BOTH, SetpointFactory.ALGAEHOME);
        break;
      case TAKEING_ALGAE_FLOOR:
        elevatorArmSubsystem.setWantedState(WantedState.MOVING_BOTH, SetpointFactory.ALGAE_FLOOR);
        break;
      case TAKEING_ALGAE_REEF_HIGH:
        elevatorArmSubsystem.setWantedState(
            WantedState.MOVING_BOTH, SetpointFactory.ALGAE_REEF_HIGH);
        break;
      case TAKEING_ALGAE_REEF_LOW:
        elevatorArmSubsystem.setWantedState(
            WantedState.MOVING_BOTH, SetpointFactory.ALGAE_REEF_LOW);
        break;
      case NET:
        elevatorArmSubsystem.setWantedState(WantedState.MOVING_BOTH, SetpointFactory.NET);
        break;
      case PROCESSOR:
        elevatorArmSubsystem.setWantedState(WantedState.MOVING_BOTH, SetpointFactory.PROCESSOR);
        break;
      case PRE_L1:
        elevatorArmSubsystem.setWantedState(WantedState.MOVING_BOTH, SetpointFactory.PRE_L1);
        break;
      case PRE_L2:
        elevatorArmSubsystem.setWantedState(WantedState.MOVING_BOTH, SetpointFactory.PRE_L2);
        break;
      case PRE_L3:
        elevatorArmSubsystem.setWantedState(WantedState.MOVING_BOTH, SetpointFactory.PRE_L3);
        break;
      case PRE_L4:
        elevatorArmSubsystem.setWantedState(WantedState.MOVING_BOTH, SetpointFactory.PRE_L4);
        break;
      case PLACE_L1:
        elevatorArmSubsystem.setWantedState(WantedState.MOVING_BOTH, SetpointFactory.PLACE_L1);
        break;
      case PLACE_L2:
        elevatorArmSubsystem.setWantedState(WantedState.MOVING_BOTH, SetpointFactory.PLACE_L2);
        break;
      case PLACE_L3:
        elevatorArmSubsystem.setWantedState(WantedState.MOVING_BOTH, SetpointFactory.PLACE_L3);
        gripper.setState(
          frc.robot.subsystems.ElevatorArmSubsystem.Gripper.Gripper.WantedState.SHOOT);
        break;
      case PLACE_L4:
        elevatorArmSubsystem.setWantedState(WantedState.MOVING_BOTH, SetpointFactory.PLACE_L4);
        break;
    }
  }

  public void resetIntakeToggle() {
    l1ReleaseCount = 0;
  }

  private void setBigArmState(WantedBigArmState wantedBigArmState) {
    this.wantedBigArmState = wantedBigArmState;
  }

  public Command setBigArmStateCommand(WantedBigArmState wantedBigArmState) {
    return Commands.runOnce(() -> setBigArmState(wantedBigArmState));
  }

  public Command setGripperStateCommand(
      frc.robot.subsystems.ElevatorArmSubsystem.Gripper.Gripper.WantedState state) {
    return Commands.runOnce(() -> gripper.setState(state));
  }
}
