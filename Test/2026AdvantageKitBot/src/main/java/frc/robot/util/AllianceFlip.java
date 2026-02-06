// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Utility class for flipping poses between blue and red alliance perspectives.
 *
 * <p>The FRC field is symmetric, with blue alliance on one side and red alliance on the other.
 * Coordinates are mirrored across the center line of the field.
 */
public class AllianceFlip {

  // 2026 field dimensions (adjust these for your specific game year)
  // Field length in meters (X direction)
  private static final double FIELD_LENGTH = 16.54099; // 54.268 feet in meters

  // Field width in meters (Y direction) - not used for flipping but included for reference
  private static final double FIELD_WIDTH = 8.06928; // 26.47 feet in meters

  /**
   * Transposes a Pose3d from blue alliance perspective to red alliance perspective.
   *
   * <p>In FRC, the field coordinate system is defined with: - Origin at the blue alliance driver
   * station corner - +X pointing toward red alliance - +Y pointing to the left (when facing red
   * alliance) - +Z pointing up
   *
   * <p>When flipping from blue to red alliance: - X coordinate: mirrored across field center
   * (FIELD_LENGTH - x) - Y coordinate: unchanged (field is symmetric) - Z coordinate: unchanged
   * (height doesn't flip) - Rotation: adjusted to maintain robot orientation relative to the field
   *
   * @param bluePose The pose in blue alliance perspective
   * @return The equivalent pose in red alliance perspective
   */
  public static Pose3d blueToRed(Pose3d bluePose) {
    // Mirror the X coordinate across the field center
    double redX = FIELD_LENGTH - bluePose.getX();

    // Y and Z coordinates remain the same
    double redY = bluePose.getY();
    double redZ = bluePose.getZ();

    // For rotation, we need to flip the yaw (rotation around Z axis)
    // Roll and pitch remain the same as they're relative to the robot
    Rotation3d blueRotation = bluePose.getRotation();

    // Flip the yaw by rotating 180 degrees and negating
    // This maintains the robot's orientation relative to the field
    double redRoll = blueRotation.getX(); // Roll stays the same
    double redPitch = blueRotation.getY(); // Pitch stays the same
    double redYaw = Math.PI - blueRotation.getZ(); // Yaw is flipped

    Translation3d redTranslation = new Translation3d(redX, redY, redZ);
    Rotation3d redRotation = new Rotation3d(redRoll, redPitch, redYaw);

    return new Pose3d(redTranslation, redRotation);
  }

  /**
   * Transposes a Pose3d from red alliance perspective to blue alliance perspective.
   *
   * <p>This is the inverse operation of blueToRed().
   *
   * @param redPose The pose in red alliance perspective
   * @return The equivalent pose in blue alliance perspective
   */
  public static Pose3d redToBlue(Pose3d redPose) {
    // The transformation is symmetric, so we can use the same logic
    return blueToRed(redPose);
  }

  /**
   * Transposes a Pose2d from blue alliance perspective to red alliance perspective.
   *
   * <p>Similar to the 3D version but for 2D poses (no Z or roll/pitch).
   *
   * @param bluePose The pose in blue alliance perspective
   * @return The equivalent pose in red alliance perspective
   */
  public static Pose2d blueToRed(Pose2d bluePose) {
    // Mirror the X coordinate across the field center
    double redX = FIELD_LENGTH - bluePose.getX();

    // Y coordinate remains the same
    double redY = bluePose.getY();

    // Flip the rotation by rotating 180 degrees
    Rotation2d redRotation =
        bluePose.getRotation().rotateBy(Rotation2d.fromDegrees(180)).unaryMinus();

    return new Pose2d(redX, redY, redRotation);
  }

  /**
   * Transposes a Pose2d from red alliance perspective to blue alliance perspective.
   *
   * <p>This is the inverse operation of blueToRed() for 2D poses.
   *
   * @param redPose The pose in red alliance perspective
   * @return The equivalent pose in blue alliance perspective
   */
  public static Pose2d redToBlue(Pose2d redPose) {
    // The transformation is symmetric, so we can use the same logic
    return blueToRed(redPose);
  }

  /**
   * Automatically flips a pose based on the current alliance. If on red alliance, converts from
   * blue to red perspective. If on blue alliance, returns the pose unchanged.
   *
   * <p>This is useful for using blue alliance coordinates in your code and automatically converting
   * them based on alliance color.
   *
   * @param bluePose The pose in blue alliance perspective
   * @return The pose in the current alliance's perspective
   */
  public static Pose3d flipForAlliance(Pose3d bluePose) {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      return blueToRed(bluePose);
    }
    return bluePose;
  }

  /**
   * Automatically flips a 2D pose based on the current alliance.
   *
   * @param bluePose The pose in blue alliance perspective
   * @return The pose in the current alliance's perspective
   */
  public static Pose2d flipForAlliance(Pose2d bluePose) {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      return blueToRed(bluePose);
    }
    return bluePose;
  }

  /**
   * Flips a Translation2d from blue to red alliance perspective.
   *
   * @param blueTranslation The translation in blue alliance perspective
   * @return The equivalent translation in red alliance perspective
   */
  public static Translation2d blueToRed(Translation2d blueTranslation) {
    return new Translation2d(FIELD_LENGTH - blueTranslation.getX(), blueTranslation.getY());
  }

  /**
   * Flips a Translation3d from blue to red alliance perspective.
   *
   * @param blueTranslation The translation in blue alliance perspective
   * @return The equivalent translation in red alliance perspective
   */
  public static Translation3d blueToRed(Translation3d blueTranslation) {
    return new Translation3d(
        FIELD_LENGTH - blueTranslation.getX(), blueTranslation.getY(), blueTranslation.getZ());
  }

  /**
   * Sets the field dimensions for the current game year. Call this in Robot.java during
   * initialization if using non-standard dimensions.
   *
   * @param length Field length in meters (X direction)
   */
  public static void setFieldLength(double length) {
    // Note: This would require making FIELD_LENGTH non-final
    // For now, just modify the constant at the top of the class
    throw new UnsupportedOperationException(
        "Modify FIELD_LENGTH constant in AllianceFlipUtil.java for your game year");
  }
}
