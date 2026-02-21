// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

/** Add your docs here. */
public class ShooterAimingCalcs {
  private double targetYawAngle;
  private double robotYawAngle;
  private double relativeTargetYawAngle;

  private double flywheelPower;

  public double calculateShooterFlywheelPower() {
    return 0.0;
  }

  public double calculateShooterYawDegrees(Pose2d robot) {
    robotYawAngle = robot.getRotation().getDegrees() + 180; //robot angle 0 to 360

    targetYawAngle =
        Math.toDegrees(Math.atan2(getDistanceFromHub(robot).getY(), getDistanceFromHub(robot).getX())) + 180;

    relativeTargetYawAngle = (360 + (targetYawAngle - robotYawAngle)) % 360;

    return relativeTargetYawAngle;
  }

  public double calculateShooterYawInMotion(Pose2d robot, double velocityX, double velocityY, double angularVelocity,
      double launchVelocity, double launchPitch) {
    double offsetX = velocityX * calculateTimeOfFlight(robot, launchVelocity, launchPitch); // Get the offset by getting the product of the x velocity and the time of flight
    double offsetY = velocityY * calculateTimeOfFlight(robot, launchVelocity, launchPitch); // Same with y
    double offsetRot = angularVelocity * calculateTimeOfFlight(robot, launchVelocity, launchPitch); // Same but with angular velocity
    return calculateShooterYawDegrees(new Pose2d(robot.getX() + offsetX, robot.getY() + offsetY,
        robot.getRotation().plus(new Rotation2d(offsetRot)))); // Returns a new pose2d with the offset added
  }

  public double calculateTimeOfFlight(Pose2d robot, double launchVelocity, double launchPitch) {
    double distanceX = getDistanceFromHub(robot).getX();
    double velocityX = launchVelocity * Math.cos(launchPitch);
    return distanceX / velocityX;
  }

  public Translation2d getDistanceFromHub(Pose2d robot) {
    Alliance alliance = GameData.getAlliance();
    Translation2d distToHub;
    Pose2d turretPos = robot.plus(new Transform2d(-0.1, 0.1, new Rotation2d(0.0)));

    if(alliance == Alliance.Blue) {
      distToHub = new Translation2d(Constants.FieldElements.kBlueHubX - turretPos.getX(),
          Constants.FieldElements.kBlueHubY - turretPos.getY());
    } else {
      distToHub = new Translation2d(Constants.FieldElements.kRedHubX - turretPos.getX(),
          Constants.FieldElements.kRedHubY - turretPos.getY());
    }
    return distToHub;
  }

  public double calculateShooterPitchDegrees() {
    return 0.0;
  }
}
