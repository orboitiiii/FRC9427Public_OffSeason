package frc.robot.subsystems.ElevatorArmSubsystem.JointedArm;

import com.slsh.IDeer9427.lib.controls.util.StateSpaceUtil;
import org.ejml.simple.SimpleMatrix;

public class JointedArmConstants {
  public static class Hardware {
    public static final int LEFT_ARM_MOTOR_ID = 0;
    public static final int RIGHT_ARM_MOTOR_ID = 0;
  }

  public static class System {
    public static final int numMotor = 2;
    public static final double G = 16.0;
    public static final double J = 0.015097917;
    public static final double L = 0.0;
    public static final double M = 0.0;
    public static final SimpleMatrix kalman_q =
        SimpleMatrix.diag(Math.pow(0.12, 2), Math.pow(20.0, 2));
    public static final SimpleMatrix kalman_r =
        SimpleMatrix.diag(Math.pow(0.05, 2), Math.pow(10.0, 2));
    public static final SimpleMatrix lqr_q =
        StateSpaceUtil.makeStateCostMatrix(new double[] {0.00020, 10.0}, true, 2);
    public static final SimpleMatrix lqr_r =
        StateSpaceUtil.makeInputCostMatrix(new double[] {1.5}, true, 1);
  }

  public static class Pyhsical {
    public static final double kMaxDegrees = 0.0;
    public static final double kMinDegrees = 0.0;
    public static final double kMaxSpeed = 0.0;
    public static final double kMaxAcceleration = 0.0;
  }
}
