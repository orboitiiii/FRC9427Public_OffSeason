package com.slsh.IDeer9427.lib.Vision;

import java.util.EnumMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.DriveTrain.SwerveConstants;

public final class ReefFieldPose {
  public enum Alliance {
    BLUE
  }

  public enum ReefPoint {
    A,
    B,
    C,
    D,
    E,
    F,
    G,
    H,
    I,
    J,
    K,
    L
  }

  private static final Map<ReefPoint, Pose2d> reefMap = new EnumMap<>(ReefPoint.class);

  static {
    initializeBluePoints();
  }

  public static Pose2d getReefPose(ReefPoint point, double dMeters) {
    Pose2d base = reefMap.get(point);
    if (base == null) throw new IllegalArgumentException("Unknown ReefPoint: " + point);
    return applyForwardOffset(base, dMeters);
  }

  private static Pose2d applyForwardOffset(Pose2d base, double dMeters) {
    Translation2d delta = new Translation2d(-dMeters, 0.0).rotateBy(base.getRotation());
    return new Pose2d(base.getX() + delta.getX(), base.getY() + delta.getY(), base.getRotation());
  }

  private static void initializeBluePoints() {
    final AprilTagField tagField = new AprilTagField();
    final double d = SwerveConstants.DriveConstants.kWheelBase / 2.0;
    final double sin30 = 0.5;
    final double cos30 = 0.86602540378;
    final double tag_To_Reef = 0.164338;

    // A, B (0°)
    reefMap.put(
        ReefPoint.A,
        new Pose2d(
            tagField.getTag(18).getX() - d,
            tagField.getTag(18).getY() + tag_To_Reef,
            new Rotation2d(Math.toRadians(0.0))));
    reefMap.put(
        ReefPoint.B,
        new Pose2d(
            tagField.getTag(18).getX() - d,
            tagField.getTag(18).getY() - tag_To_Reef,
            new Rotation2d(Math.toRadians(0.0))));

    // C, D (60°)
    reefMap.put(
        ReefPoint.C,
        new Pose2d(
            tagField.getTag(17).getX() - tag_To_Reef * cos30 - d * sin30,
            tagField.getTag(17).getY() + tag_To_Reef * sin30 - d * cos30,
            new Rotation2d(Math.toRadians(60.0))));
    reefMap.put(
        ReefPoint.D,
        new Pose2d(
            tagField.getTag(17).getX() + tag_To_Reef * cos30 - d * sin30,
            tagField.getTag(17).getY() - tag_To_Reef * sin30 - d * cos30,
            new Rotation2d(Math.toRadians(60.0))));

    // E, F (120°)
    reefMap.put(
        ReefPoint.E,
        new Pose2d(
            tagField.getTag(22).getX() - tag_To_Reef * cos30 + d * sin30,
            tagField.getTag(22).getY() - tag_To_Reef * sin30 - d * cos30,
            new Rotation2d(Math.toRadians(120.0))));
    reefMap.put(
        ReefPoint.F,
        new Pose2d(
            tagField.getTag(22).getX() + tag_To_Reef * cos30 + d * sin30,
            tagField.getTag(22).getY() + tag_To_Reef * sin30 - d * cos30,
            new Rotation2d(Math.toRadians(120.0))));

    // G, H (180°)
    reefMap.put(
        ReefPoint.G,
        new Pose2d(
            tagField.getTag(21).getX() + d,
            tagField.getTag(21).getY() - tag_To_Reef,
            new Rotation2d(Math.toRadians(180.0))));
    reefMap.put(
        ReefPoint.H,
        new Pose2d(
            tagField.getTag(21).getX() + d,
            tagField.getTag(21).getY() + tag_To_Reef,
            new Rotation2d(Math.toRadians(180.0))));

    // I, J (-120°)
    reefMap.put(
        ReefPoint.I,
        new Pose2d(
            tagField.getTag(20).getX() + tag_To_Reef * cos30 + d * sin30,
            tagField.getTag(20).getY() - tag_To_Reef * sin30 + d * cos30,
            new Rotation2d(Math.toRadians(-120.0))));
    reefMap.put(
        ReefPoint.J,
        new Pose2d(
            tagField.getTag(20).getX() - tag_To_Reef * cos30 + d * sin30,
            tagField.getTag(20).getY() + tag_To_Reef * sin30 + d * cos30,
            new Rotation2d(Math.toRadians(-120.0))));

    // K, L (-60°)
    reefMap.put(
        ReefPoint.K,
        new Pose2d(
            tagField.getTag(19).getX() + tag_To_Reef * cos30 - d * sin30,
            tagField.getTag(19).getY() + tag_To_Reef * sin30 + d * cos30,
            new Rotation2d(Math.toRadians(-60.0))));
    reefMap.put(
        ReefPoint.L,
        new Pose2d(
            tagField.getTag(19).getX() - tag_To_Reef * cos30 - d * sin30,
            tagField.getTag(19).getY() - tag_To_Reef * sin30 + d * cos30,
            new Rotation2d(Math.toRadians(-60.0))));
  }

  private ReefFieldPose() {}
}
