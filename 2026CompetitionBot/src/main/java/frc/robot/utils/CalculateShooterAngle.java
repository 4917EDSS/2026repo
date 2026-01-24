// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

/** Add your docs here. */
public class CalculateShooterAngle {
  public double getAngle(Pose3d robot) {
    Alliance alliance = GameData.getAlliance();

    Pose3d shooterPoseinfield = new Pose3d();
    Transform3d shooterToRobot = new Transform3d(
        new Translation3d(0.3, 0.0, 0.3),
        new Rotation3d(0.0, 0.0, 0.0));
    shooterPoseinfield = componentToField(shooterPoseinfield, robot, shooterToRobot);

    double distX = 1; // default values
    double distY = 0;

    if(alliance == Alliance.Blue) { // get the distance based on alliance
      distX = shooterPoseinfield.getX() - Constants.FieldElements.kBlueHubX;
      distY = shooterPoseinfield.getY() - Constants.FieldElements.kBlueHubY;
    } else if(alliance == Alliance.Red) {
      distX = shooterPoseinfield.getX() - Constants.FieldElements.kRedHubX;
      distY = shooterPoseinfield.getY() - Constants.FieldElements.kRedHubY;
    }

    // calculate the shooter angle, atan2 works in all quadrants, atan doesn't
    double angle = Math.toDegrees(Math.atan2(distY, distX));

    // get the robot's angle
    double heading = shooterPoseinfield.toPose2d().getRotation().getDegrees();

    // subtract the robot heading from the shooter angle
    angle -= heading;

    return angle;
  }

  private static Pose3d componentToField(Pose3d componentRelativePose, Pose3d robotPose,
      Transform3d componentToRobotTransform) {
    Pose3d robotRelative = componentToRobot(componentRelativePose, componentToRobotTransform);
    return robotToField(robotRelative, robotPose);
  }

  private static Pose3d componentToRobot(Pose3d componentRelativePose, Transform3d componentToRobotTransform) {
    return componentRelativePose.plus(componentToRobotTransform);
  }

  private static Pose3d robotToField(Pose3d robotRelativePose, Pose3d robotPose) {
    return robotRelativePose.relativeTo(robotPose);
  }

}
