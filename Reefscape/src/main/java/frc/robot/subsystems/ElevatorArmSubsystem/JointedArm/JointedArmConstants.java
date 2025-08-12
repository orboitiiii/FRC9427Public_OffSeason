package frc.robot.subsystems.ElevatorArmSubsystem.JointedArm;

public class JointedArmConstants {
  public static class Hardware {
    public static final int LEFT_ARM_MOTOR_ID = 40;
    public static final int RIGHT_ARM_MOTOR_ID = 33;
  }

  public static class System {
    public static final int numMotor = 2;
    public static final double G = 16.0;
    public static final double J = 0.0679;
    public static final double L = 0.62;
    public static final double M = 2.722;
  }

  public static class Pyhsical {
    public static final double kMaxRadians = Math.toRadians(30.0);
    public static final double kMaxRadiansAcceleration = Math.toRadians(10.0);
  }
}
