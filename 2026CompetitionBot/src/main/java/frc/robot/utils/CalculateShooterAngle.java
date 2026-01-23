// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

/** Add your docs here. */
public class CalculateShooterAngle {
  public double getAngle(Pose2d robot) {
    Alliance alliance = GameData.getAlliance();

    double distX = 1; // default values
    double distY = 0;

    if(alliance == Alliance.Blue) { // get the distance based on alliance
      distX = robot.getX() - Constants.FieldElements.kBlueHubX;
      distY = robot.getY() - Constants.FieldElements.kBlueHubY;
    } else if(alliance == Alliance.Red) {
      distX = robot.getX() - Constants.FieldElements.kRedHubX;
      distY = robot.getY() - Constants.FieldElements.kRedHubY;
    }

    // calculate the shooter angle, atan2 works in all quadrants, atan doesn't
    double angle = Math.toDegrees(Math.atan2(distY, distX));

    // get the robot's angle
    double heading = robot.getRotation().getDegrees();

    // subtract the robot heading from the shooter angle
    angle -= heading;

    return angle;
  }
}
