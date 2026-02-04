package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Utility class for transforming Pose3d objects between different coordinate frames. Supports
 * transformations between field, robot, and any custom coordinate systems.
 *
 * <p>Coordinate system conventions: - Field: Origin at field corner, +X forward, +Y left, +Z up -
 * Robot: Origin at robot center, +X forward, +Y left, +Z up - Shooter/Component: Custom origin and
 * orientation relative to robot
 */
public class CoordinateTransform {

  /**
   * Transforms a pose from field coordinates to robot-relative coordinates.
   *
   * @param fieldPose The pose in field coordinates
   * @param robotPose The robot's pose in field coordinates
   * @return The pose expressed in robot-relative coordinates
   */
  public static Pose3d fieldToRobot(Pose3d fieldPose, Pose3d robotPose) {
    // To transform from field to robot frame:
    // 1. Get the inverse of robot's pose (robot frame origin in field coordinates)
    // 2. Apply this inverse transform to the field pose
    return fieldPose.relativeTo(robotPose);
  }

  /**
   * Transforms a pose from robot-relative coordinates to field coordinates.
   *
   * @param robotRelativePose The pose in robot-relative coordinates
   * @param robotPose The robot's pose in field coordinates
   * @return The pose expressed in field coordinates
   */
  public static Pose3d robotToField(Pose3d robotRelativePose, Pose3d robotPose) {
    // To transform from robot to field frame:
    // Apply the robot's pose transform to the robot-relative pose
    return robotPose.plus(robotRelativePose.minus(new Pose3d()));
  }

  /**
   * Transforms a pose from robot coordinates to a component's coordinate system.
   *
   * @param robotRelativePose The pose in robot-relative coordinates
   * @param componentToRobotTransform Transform from component frame to robot frame
   * @return The pose expressed in component-relative coordinates
   */
  public static Pose3d robotToComponent(
      Pose3d robotRelativePose, Transform3d componentToRobotTransform) {
    // Convert transform to pose (origin of component in robot frame)
    Pose3d componentOriginInRobot = new Pose3d().plus(componentToRobotTransform);

    // Transform to component frame
    return robotRelativePose.relativeTo(componentOriginInRobot);
  }

  /**
   * Transforms a pose from a component's coordinate system to robot coordinates.
   *
   * @param componentRelativePose The pose in component-relative coordinates
   * @param componentToRobotTransform Transform from component frame to robot frame
   * @return The pose expressed in robot-relative coordinates
   */
  public static Pose3d componentToRobot(
      Pose3d componentRelativePose, Transform3d componentToRobotTransform) {
    // Apply the component-to-robot transform
    return componentRelativePose.plus(componentToRobotTransform);
  }

  /**
   * Transforms a pose from field coordinates to a component's coordinate system. Convenience method
   * that chains field->robot->component transformations.
   *
   * @param fieldPose The pose in field coordinates
   * @param robotPose The robot's pose in field coordinates
   * @param componentToRobotTransform Transform from component frame to robot frame
   * @return The pose expressed in component-relative coordinates
   */
  public static Pose3d fieldToComponent(
      Pose3d fieldPose, Pose3d robotPose, Transform3d componentToRobotTransform) {
    Pose3d robotRelative = fieldToRobot(fieldPose, robotPose);
    return robotToComponent(robotRelative, componentToRobotTransform);
  }

  /**
   * Transforms a pose from a component's coordinate system to field coordinates. Convenience method
   * that chains component->robot->field transformations.
   *
   * @param componentRelativePose The pose in component-relative coordinates
   * @param robotPose The robot's pose in field coordinates
   * @param componentToRobotTransform Transform from component frame to robot frame
   * @return The pose expressed in field coordinates
   */
  public static Pose3d componentToField(
      Pose3d componentRelativePose, Pose3d robotPose, Transform3d componentToRobotTransform) {
    Pose3d robotRelative = componentToRobot(componentRelativePose, componentToRobotTransform);
    return robotToField(robotRelative, robotPose);
  }

  /**
   * Generic transformation between any two coordinate frames.
   *
   * @param pose The pose to transform
   * @param fromFrame The origin/orientation of the source coordinate frame
   * @param toFrame The origin/orientation of the target coordinate frame
   * @return The pose expressed in the target coordinate frame
   */
  public static Pose3d transform(Pose3d pose, Pose3d fromFrame, Pose3d toFrame) {
    // First express the pose in a common frame (world/field)
    Pose3d inCommonFrame = fromFrame.plus(pose.minus(new Pose3d()));

    // Then express it relative to the target frame
    return inCommonFrame.relativeTo(toFrame);
  }

  /**
   * Creates a Transform3d from a component's position and orientation relative to the robot. Helper
   * method for defining component mounting positions.
   *
   * @param x Forward offset from robot center (meters, positive = forward)
   * @param y Left offset from robot center (meters, positive = left)
   * @param z Up offset from robot center (meters, positive = up)
   * @param roll Roll angle (radians, rotation about X axis)
   * @param pitch Pitch angle (radians, rotation about Y axis)
   * @param yaw Yaw angle (radians, rotation about Z axis)
   * @return Transform3d representing component position/orientation in robot frame
   */
  public static Transform3d createComponentTransform(
      double x, double y, double z, double roll, double pitch, double yaw) {
    return new Transform3d(new Translation3d(x, y, z), new Rotation3d(roll, pitch, yaw));
  }

  /**
   * Extracts just the translation component when transforming between frames. Useful when you only
   * care about position, not orientation.
   *
   * @param position The position (as Translation3d) to transform
   * @param fromFrame The source coordinate frame
   * @param toFrame The target coordinate frame
   * @return The position expressed in the target coordinate frame
   */
  public static Translation3d transformPosition(
      Translation3d position, Pose3d fromFrame, Pose3d toFrame) {
    // Create a pose with zero rotation at the position
    Pose3d pose = new Pose3d(position, new Rotation3d());

    // Transform the pose
    Pose3d transformed = transform(pose, fromFrame, toFrame);

    // Return just the translation
    return transformed.getTranslation();
  }

  /**
   * Calculates the transform between two coordinate frames.
   *
   * @param frame1 First coordinate frame (as Pose3d in common reference)
   * @param frame2 Second coordinate frame (as Pose3d in common reference)
   * @return Transform from frame1 to frame2
   */
  public static Transform3d getTransformBetween(Pose3d frame1, Pose3d frame2) {
    return new Transform3d(frame1, frame2);
  }

  /** Example usage for a typical FRC shooter setup */
  public static class ShooterExample {
    // Define shooter position on robot
    // Shooter is 0.3m forward, 0m to side, 0.4m up from robot center
    // Tilted back 30 degrees (pitch)
    private static final Transform3d SHOOTER_TO_ROBOT =
        createComponentTransform(
            0.3, // 0.3m forward
            0.0, // centered
            0.4, // 0.4m up
            0.0, // no roll
            Math.toRadians(30), // 30° pitch back
            0.0 // no yaw
            );

    /** Example: Transform a target from field coordinates to shooter coordinates */
    public static Pose3d getTargetInShooterFrame(Pose3d targetInField, Pose3d robotPose) {
      return CoordinateTransform.fieldToComponent(targetInField, robotPose, SHOOTER_TO_ROBOT);
    }

    /** Example: Get the shooter's position and orientation in field coordinates */
    public static Pose3d getShooterPoseInField(Pose3d robotPose) {
      // Shooter at origin in its own frame
      Pose3d shooterOrigin = new Pose3d();
      return CoordinateTransform.componentToField(shooterOrigin, robotPose, SHOOTER_TO_ROBOT);
    }

    /** Example: Calculate distance from shooter to target */
    public static double getDistanceToTarget(Pose3d targetInField, Pose3d robotPose) {
      Pose3d targetInShooter = getTargetInShooterFrame(targetInField, robotPose);
      return targetInShooter.getTranslation().getNorm();
    }

    /** Example: Get elevation angle to target from shooter's perspective */
    public static double getElevationToTarget(Pose3d targetInField, Pose3d robotPose) {
      Pose3d targetInShooter = getTargetInShooterFrame(targetInField, robotPose);
      Translation3d toTarget = targetInShooter.getTranslation();

      // Calculate elevation angle in shooter's XZ plane
      double horizontalDist =
          Math.sqrt(toTarget.getX() * toTarget.getX() + toTarget.getY() * toTarget.getY());
      return Math.atan2(toTarget.getZ(), horizontalDist);
    }

    /** Example: Get azimuth angle to target from shooter's perspective */
    public static double getAzimuthToTarget(Pose3d targetInField, Pose3d robotPose) {
      Pose3d targetInShooter = getTargetInShooterFrame(targetInField, robotPose);
      Translation3d toTarget = targetInShooter.getTranslation();

      // Calculate azimuth angle in shooter's XY plane
      return Math.atan2(toTarget.getY(), toTarget.getX());
    }
  }
}
