package frc.robot.subsystems.ElevatorArmSubsystem.JointedArm;

import com.slsh.IDeer9427.lib.controls.util.StateSpaceUtil;
import org.ejml.simple.SimpleMatrix;

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
    public static final SimpleMatrix kalman_q =
        SimpleMatrix.diag(Math.pow(Math.toRadians(1.5), 2), Math.pow(Math.toRadians(8), 2));
    public static final SimpleMatrix kalman_r =
        SimpleMatrix.diag(Math.pow(Math.toRadians(0.05), 2), Math.pow(Math.toRadians(0.5), 2));
    public static final SimpleMatrix lqr_q =
        StateSpaceUtil.makeStateCostMatrix(
            new double[] {Math.toRadians(0.1), Math.toRadians(1.0)}, true, 2);
    public static final SimpleMatrix lqr_r =
        StateSpaceUtil.makeInputCostMatrix(new double[] {12.0}, true, 1);
  }

  public static class Pyhsical {
    public static final double kMaxRadians = Math.toRadians(30.0);
    public static final double kMaxRadiansAcceleration = Math.toRadians(15.0);
  }
}
