package frc.robot.subsystems.ElevatorArmSubsystem.LinearExtension;

import com.slsh.IDeer9427.lib.controls.util.StateSpaceUtil;
import org.ejml.simple.SimpleMatrix;

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
    public static final SimpleMatrix kalman_q =
        SimpleMatrix.diag(Math.pow(0.12, 2), Math.pow(0.20, 2));
    public static final SimpleMatrix kalman_r =
        SimpleMatrix.diag(Math.pow(0.05, 2), Math.pow(0.15, 2));
    public static final SimpleMatrix lqr_q =
        StateSpaceUtil.makeStateCostMatrix(new double[] {0.0070, 0.0135}, true, 2);
    public static final SimpleMatrix lqr_r =
        StateSpaceUtil.makeInputCostMatrix(new double[] {1.5}, true, 1);
  }

  public static class Pyhsical {
    public static final double kMaxHeight = 2.0;
    public static final double kMinHeight = 0.0;
    public static final double kMaxSpeed = 2.0;
    public static final double kMaxAcceleration = 15.0;
  }
}
