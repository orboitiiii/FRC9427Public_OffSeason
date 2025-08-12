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
    public static final double m = 8.608;
  }

  public static class Pyhsical {
    public static final double kMaxHeight = 2.0;
    public static final double kMinHeight = 0.0;
    public static final double kMaxSpeed = 0.5;
    public static final double kMaxAcceleration = 0.15;
  }
}
