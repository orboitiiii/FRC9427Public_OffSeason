package com.slsh.IDeer9427.lib.Vision;

import java.util.HashMap;
import java.util.Map;

public class AprilTagField {

  private final Map<Integer, AprilTag> tagCoordinates = new HashMap<>();

  public static class AprilTag {
    private final int id;
    private final double x;
    private final double y;
    private final double z;

    public AprilTag(int id, double x, double y, double z) {
      this.id = id;
      this.x = x;
      this.y = y;
      this.z = z;
    }

    public int getId() {
      return id;
    }

    public double getX() {
      return x;
    }

    public double getY() {
      return y;
    }

    public double getZ() {
      return z;
    }

    @Override
    public String toString() {
      return String.format("AprilTag{id=%d, x=%.6f, y=%.6f, z=%.6f}", id, x, y, z);
    }
  }

  private static final double OFFSET_X = 8.7741252;
  private static final double OFFSET_Y = 4.0259508;

  public AprilTagField() {
    // Tag 1: (16.6973232, 0.6552708, 1.4859)
    // Tag 2: (16.6973232, 7.3964308, 1.4859)
    // Tag 3: (11.5609352, 8.0555608, 1.30175)
    // Tag 4: ( 9.2762052, 6.1376068, 1.8679160)
    // Tag 5: ( 9.2762052, 1.9148568, 1.8679160)
    // Tag 6: (13.4745712, 3.3062688, 0.3081020)
    // Tag 7: (13.8906232, 4.0258508, 0.3081020)
    // Tag 8: (13.4745712, 4.7454328, 0.3081020)
    // Tag 9: (12.6434832, 4.7454328, 0.3081020)
    // Tag 10: (12.2274312, 4.0258508, 0.3081020)
    // Tag 11: (12.6434832, 3.3062688, 0.3081020)
    // Tag 12: ( 0.8512792, 0.6552708, 1.4859)
    // Tag 13: ( 0.8512792, 7.3964308, 1.4859)
    // Tag 14: ( 8.2723972, 6.1376068, 1.8679160)
    // Tag 15: ( 8.2723972, 1.9148568, 1.8679160)
    // Tag 16: ( 5.9876672, -0.0038592, 1.30175)
    // Tag 17: ( 4.0740312, 3.3062688, 0.3081020)
    // Tag 18: ( 3.6577252, 4.0258508, 0.3081020)
    // Tag 19: ( 4.0740312, 4.7454328, 0.3081020)
    // Tag 20: ( 4.9048652, 4.7454328, 0.3081020)
    // Tag 21: ( 5.3211712, 4.0258508, 0.3081020)
    // Tag 22: ( 4.9048652, 3.3062688, 0.3081020)
    tagCoordinates.put(
        1, new AprilTag(1, 7.923198000000001 + OFFSET_X, -3.37068 + OFFSET_Y, 1.4859));
    tagCoordinates.put(
        2, new AprilTag(2, 7.923198000000001 + OFFSET_X, 3.37048 + OFFSET_Y, 1.4859));
    tagCoordinates.put(3, new AprilTag(3, 2.78681 + OFFSET_X, 4.02961 + OFFSET_Y, 1.30175));
    tagCoordinates.put(
        4, new AprilTag(4, 0.50208 + OFFSET_X, 2.111656 + OFFSET_Y, 1.8679160000000001));
    tagCoordinates.put(
        5, new AprilTag(5, 0.50208 + OFFSET_X, -2.111094 + OFFSET_Y, 1.8679160000000001));
    tagCoordinates.put(
        6, new AprilTag(6, 4.700446000000001 + OFFSET_X, -0.7196820000000002 + OFFSET_Y, 0.308102));
    tagCoordinates.put(7, new AprilTag(7, 5.116498 + OFFSET_X, -0.0001 + OFFSET_Y, 0.308102));
    tagCoordinates.put(
        8, new AprilTag(8, 4.700446000000001 + OFFSET_X, 0.7194820000000002 + OFFSET_Y, 0.308102));
    tagCoordinates.put(
        9, new AprilTag(9, 3.869358 + OFFSET_X, 0.7194820000000002 + OFFSET_Y, 0.308102));
    tagCoordinates.put(10, new AprilTag(10, 3.453306 + OFFSET_X, -0.0001 + OFFSET_Y, 0.308102));
    tagCoordinates.put(
        11, new AprilTag(11, 3.869358 + OFFSET_X, -0.7196820000000002 + OFFSET_Y, 0.308102));
    tagCoordinates.put(
        12,
        new AprilTag(12, -7.922845999999999 + OFFSET_X, -3.3706799999999997 + OFFSET_Y, 1.4859));
    tagCoordinates.put(
        13, new AprilTag(13, -7.922845999999999 + OFFSET_X, 3.3704799999999997 + OFFSET_Y, 1.4859));
    tagCoordinates.put(
        14, new AprilTag(14, -0.501728 + OFFSET_X, 2.111656 + OFFSET_Y, 1.8679160000000001));
    tagCoordinates.put(
        15,
        new AprilTag(15, -0.501728 + OFFSET_X, -2.1110939999999996 + OFFSET_Y, 1.8679160000000001));
    tagCoordinates.put(
        16, new AprilTag(16, -2.786458 + OFFSET_X, -4.0298099999999994 + OFFSET_Y, 1.30175));
    tagCoordinates.put(
        17, new AprilTag(17, -4.700094 + OFFSET_X, -0.7196820000000002 + OFFSET_Y, 0.308102));
    tagCoordinates.put(
        18, new AprilTag(18, -5.116399999999999 + OFFSET_X, -0.0001 + OFFSET_Y, 0.308102));
    tagCoordinates.put(
        19, new AprilTag(19, -4.700094 + OFFSET_X, 0.7194820000000002 + OFFSET_Y, 0.308102));
    tagCoordinates.put(
        20,
        new AprilTag(20, -3.8692599999999997 + OFFSET_X, 0.7194820000000002 + OFFSET_Y, 0.308102));
    tagCoordinates.put(
        21, new AprilTag(21, -3.452953999999999 + OFFSET_X, -0.0001 + OFFSET_Y, 0.308102));
    tagCoordinates.put(
        22,
        new AprilTag(22, -3.8692599999999997 + OFFSET_X, -0.7196820000000002 + OFFSET_Y, 0.308102));
  }

  public AprilTag getTag(int id) {
    return tagCoordinates.get(id);
  }

  public Map<Integer, AprilTag> getAllTags() {
    return tagCoordinates;
  }

  public void printAllTags() {
    tagCoordinates.values().forEach(System.out::println);
  }
}

