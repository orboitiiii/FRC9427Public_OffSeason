package frc.robot.subsystems.ElevatorArmSubsystem.Constamts;

public final class ElevatorArmConstants {

  public static final double TOL_ANGLE_DEG = 2.0;
  public static final double TOL_ANGLE_WIDE_DEG = 5.0;
  public static final double TOL_ELEV_M = 0.02;
  public static final double TOL_ELEV_NEAR_ZERO_M = 0.08;
  public static final double TOL_ELEV_ZERO_TARGET_M = 0.01;

  public static final double ARM_SAFE_DEG =
      SetpointFactory.SAFETY_ARM_POSITION.getJointedArmAngleDegrees(); // 90
  public static final double HOME_SAFETY_ELEV_M =
      SetpointFactory.SAFETY_ELEVATOR_POSITION.getExtensionLengthMeters(); // 0.15

  public static final double ARM_NEAR_NEG90_DEG = -90.0;
  public static final double ARM_MOVE_UP_BEFORE_EXTEND_DEG = 100.0;
  public static final double SCORE_TAKE_ALGAE_ELEV_THRESHOLD_M = 1.0;

  public static final double HOME_SAFETY_ELEV_PRELIFT_BOUND_M = HOME_SAFETY_ELEV_M - 0.01;

  private ElevatorArmConstants() {}
}
