import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import com.slsh.IDeer9427.lib.controls.LinearController.LinearPlantInversionFeedforward;
import com.slsh.IDeer9427.lib.controls.LinearController.LinearQuadraticRegulator;
import com.slsh.IDeer9427.lib.controls.LinearLoops.LinearSystemLoop;
import com.slsh.IDeer9427.lib.controls.filter.KalmanFilter;
import com.slsh.IDeer9427.lib.controls.plant.LinearSystem;
import com.slsh.IDeer9427.lib.controls.plant.StateSpaceFactory;
import com.slsh.IDeer9427.lib.controls.util.StateSpaceUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.ElevatorArmSubsystem.LinearExtension.LinearExtensionConstants;
import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.Test;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

class ElevatorSubSystemTest {

  private static final Logger LOGGER = LoggerFactory.getLogger(ElevatorSubSystemTest.class);

  private static class TestConstants {
    static final double MASS_KG = 9.1;
    static final double RADIUS_M = 0.0191008;
    static final double GEARING = 4.28571428571;
    static final int NUM_MOTORS = 2;
    static final double DT_S = 0.02;
    static final double LQR_TOL_POS_M = 0.001;
    static final double LQR_TOL_VEL_MPS = 0.135;
    static final double LQR_TOL_U_VOLTS = 1.5;
  }

  //   @Test
  //   void testSubSystemConstruction() {
  //     assertDoesNotThrow(
  //         () -> new ElevatorSubSystem(), "Subsystem construction should not throw exceptions");
  //   }

  @Test
  void testLQRAndKalmanCalculation() {
    try {
      LinearSystem plant =
          StateSpaceFactory.createElevatorSystem(
              DCMotor.getKrakenX60(TestConstants.NUM_MOTORS),
              TestConstants.MASS_KG,
              TestConstants.RADIUS_M,
              TestConstants.GEARING);

      SimpleMatrix Q =
          StateSpaceUtil.makeStateCostMatrix(
              new double[] {TestConstants.LQR_TOL_POS_M, TestConstants.LQR_TOL_VEL_MPS}, true, 2);
      SimpleMatrix R =
          StateSpaceUtil.makeInputCostMatrix(new double[] {TestConstants.LQR_TOL_U_VOLTS}, true, 1);

      LinearQuadraticRegulator lqr = new LinearQuadraticRegulator(plant, Q, R, TestConstants.DT_S);

      SimpleMatrix K = lqr.getK();
      LOGGER.info("LQR Gain: {}", K);

      assertTrue(K.get(0, 0) > 0, "Position gain should be positive");
      assertTrue(K.get(0, 1) > 0, "Velocity gain should be positive");
      assertTrue(K.get(0, 0) > K.get(0, 1), "Position gain > velocity gain");

      // Kalman (assume no throw)
      // assertDoesNotThrow(
      //     () ->
      //         new KalmanFilter(
      //             plant,
      //             ElevatorConstants.System.kalman_q,
      //             ElevatorConstants.System.kalman_r,
      //             TestConstants.DT_S));

      KalmanFilter kalmanFilter =
          new KalmanFilter(
              plant,
              LinearExtensionConstants.System.kalman_q,
              LinearExtensionConstants.System.kalman_r,
              TestConstants.DT_S);

      assertEquals(
          plant.getC().getNumRows(),
          LinearExtensionConstants.System.kalman_r.getNumRows(),
          "kalman_r dimension mismatch with p");

      LinearPlantInversionFeedforward linearPlantInversionFeedforward =
          new LinearPlantInversionFeedforward(plant, TestConstants.DT_S);

      LinearSystemLoop loop =
          new LinearSystemLoop(
              lqr,
              linearPlantInversionFeedforward,
              kalmanFilter,
              LinearSystemLoop.createVoltageClamp(12.0));

      loop.reset(new SimpleMatrix(2, 1));
      loop.setNextR(new SimpleMatrix(new double[][] {{0.5}, {0.5}}));
      loop.correct(new SimpleMatrix(new double[][] {{0.0}, {0.0}}));
      loop.predict(TestConstants.DT_S);
      double u = loop.getU().get(0, 0);
      LOGGER.info("output is: {}", u);
      assertNotEquals(0.0, u, 1e-6, "Control input u should be non-zero for non-zero reference");

    } catch (Exception e) {
      fail("Calculation failed: " + e.getMessage());
    }
  }
}
