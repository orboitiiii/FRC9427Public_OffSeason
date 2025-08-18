import static org.junit.jupiter.api.Assertions.assertTrue;

import com.slsh.IDeer9427.lib.Vision.ReefFieldPose;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

public class TestReefPose {
  private static final double EPS = 1e-9;

  @Test
  void offsetDeltaMatchesRotation() {
    double d1 = 0.12, d2 = 0.47;
    for (ReefFieldPose.ReefPoint p : ReefFieldPose.ReefPoint.values()) {
      Pose2d a = ReefFieldPose.getReefPose(p, d1);
      Pose2d b = ReefFieldPose.getReefPose(p, d2);

      // 旋轉應不受 d 影響
      assertAlmost(a.getRotation().getRadians(), b.getRotation().getRadians(), EPS);

      // 平移差應等於在機器人 +X 反向移動 (d2-d1) 後旋轉到場座標
      Rotation2d rot = a.getRotation();
      Translation2d expected = new Translation2d(-(d2 - d1), 0.0).rotateBy(rot);
      Translation2d actual = new Translation2d(b.getX() - a.getX(), b.getY() - a.getY());
      assertTransAlmost(expected, actual, EPS);
    }
  }

  @Test
  void negativeAndZeroOffsetsAreConsistent() {
    double d0 = 0.0, dPos = 0.3, dNeg = -0.25;
    for (ReefFieldPose.ReefPoint p : ReefFieldPose.ReefPoint.values()) {
      Pose2d base = ReefFieldPose.getReefPose(p, d0);
      Pose2d pos = ReefFieldPose.getReefPose(p, dPos);
      Pose2d neg = ReefFieldPose.getReefPose(p, dNeg);

      // 從 base 到 pos 的位移
      Translation2d dp = new Translation2d(pos.getX() - base.getX(), pos.getY() - base.getY());
      Translation2d ep = new Translation2d(-dPos, 0.0).rotateBy(base.getRotation());
      assertTransAlmost(ep, dp, EPS);

      // 從 base 到 neg 的位移
      Translation2d dn = new Translation2d(neg.getX() - base.getX(), neg.getY() - base.getY());
      Translation2d en = new Translation2d(-dNeg, 0.0).rotateBy(base.getRotation());
      assertTransAlmost(en, dn, EPS);
    }
  }

  @Test
  void sameInputIsIdempotent() {
    for (ReefFieldPose.ReefPoint p : ReefFieldPose.ReefPoint.values()) {
      Pose2d a = ReefFieldPose.getReefPose(p, 0.33);
      Pose2d b = ReefFieldPose.getReefPose(p, 0.33);
      assertPoseAlmost(a, b, EPS);
    }
  }

  // ---- helpers ----
  private static void assertAlmost(double expected, double actual, double eps) {
    assertTrue(
        Math.abs(expected - actual) <= eps, () -> "expected=" + expected + " actual=" + actual);
  }

  private static void assertTransAlmost(Translation2d e, Translation2d a, double eps) {
    assertAlmost(e.getX(), a.getX(), eps);
    assertAlmost(e.getY(), a.getY(), eps);
  }

  private static void assertPoseAlmost(Pose2d e, Pose2d a, double eps) {
    assertAlmost(e.getX(), a.getX(), eps);
    assertAlmost(e.getY(), a.getY(), eps);
    assertAlmost(e.getRotation().getRadians(), a.getRotation().getRadians(), eps);
  }
}
