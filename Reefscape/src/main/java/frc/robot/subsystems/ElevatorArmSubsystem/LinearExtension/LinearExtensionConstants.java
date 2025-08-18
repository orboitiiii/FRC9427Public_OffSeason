package frc.robot.subsystems.ElevatorArmSubsystem.LinearExtension;

public final class LinearExtensionConstants {
  public static class Hardware {
    public static final int LEFT_ELEVATOR_MOTOR_ID = 21;
    public static final int RIGHT_ELEVATOR_MOTOR_ID = 23;
  }

  public static class System {
    public static final int numMotor = 2;
    public static final double G = 4.28571428571; // 60 / 14 = 4.28571428571
    public static final double radius = 0.0191008; // meters
    public static final double m = 7.0;
  }

  public static class Pyhsical {
    public static final double kMaxHeight = 2.0;
    public static final double kMinHeight = 0.0;
    public static final double kMaxSpeed = 2.8; // 2.8 ,0.25
    public static final double kMaxAcceleration = 1.5; // 8.0 ,0.10
  }

  /** Closed loop gains for Motion Magic control. */
  public static class ClosedLoop {
    public static final double kP = 70.0; // Proportional gain
    public static final double kD = 0.0; // Derivative gain
    public static final double kV = 4.1441; // Velocity feedforward gain
  }
}
