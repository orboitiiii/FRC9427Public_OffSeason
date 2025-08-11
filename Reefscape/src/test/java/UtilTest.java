import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.util.Util;
import org.junit.jupiter.api.Test;

public class UtilTest {
  @Test
  public void testLinearDistanceToRotation() {
    double radius = 0.1; // meters
    double gearing = 2.0;
    double distance = 2 * Math.PI * radius; // one wheel rotation
    double rotations = Util.linearDistanceToRotation(distance, radius, gearing);
    assertEquals(gearing, rotations, 1e-9);
  }
}
