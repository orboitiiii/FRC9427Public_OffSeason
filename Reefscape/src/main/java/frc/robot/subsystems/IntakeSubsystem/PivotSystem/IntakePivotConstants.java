package frc.robot.subsystems.IntakeSubsystem.PivotSystem;

public class IntakePivotConstants {
  public static class Hardware {
    public static final int LeftMotorID = 24;
    public static final int rightMotorID = 23;
  }

  public static class System {
    public static final double Gearing = 18.0;
    public static final double POS_RAD_PER_ROT = 2.0 * Math.PI / Gearing;
    public static final double VEL_RAD_PER_RPM = (2.0 * Math.PI) / (Gearing * 60);
  }

  public static class Physic {
    public static final double CRUISE_VEL_RAD_PER_S = Math.toRadians(30);
    public static final double MAX_ACCEL_RAD_PER_S2 = Math.toRadians(10);
    public static final double kP_V_PER_RAD = 0.10;
    public static final double kD_V_PER_RAD_PER_S = 0.20;
    public static final double kG_VOLTS = 0.2;
  }
}
