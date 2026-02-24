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

  public double calculateShooterYawInMotion(Pose2d robot, double velocityX, double velocityY,
      double angularVelocityDegrees,
      double launchVelocity, double launchPitch) {
    double offsetX = velocityX * calculateTimeOfFlight(robot, launchVelocity, launchPitch); // Get the offset by getting the product of the x velocity and the time of flight
    double offsetY = velocityY * calculateTimeOfFlight(robot, launchVelocity, launchPitch); // Same with y
    double offsetRot = angularVelocityDegrees * calculateTimeOfFlight(robot, launchVelocity, launchPitch); // Same but with angular velocity
    return calculateShooterYawDegrees(new Pose2d(robot.getX() + offsetX, robot.getY() + offsetY,
        robot.getRotation().plus(new Rotation2d(Math.toRadians(offsetRot))))); // Returns a new pose2d with the offset added
  }

  public double calculateTimeOfFlight(Pose2d robot, double launchVelocity, double launchPitch) {
    double distanceX = getDistanceFromHub(robot).getX();
    double velocityX = launchVelocity * Math.cos(Math.toRadians(launchPitch));
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

  public double calculateShooterPitchDegrees(Pose2d robot, double velocity) {
    // link to the visual trajectory calculation desmos graph
    // desmos.com/calculator/kd3svmcurr
    // link to the proof for each equation
    // desmos.com/calculator/er6sltycq6
    // 0 degrees output is horizontal, 90 is vertical pointing up

    double dx;
    double dy;
    double g = Constants.Shooter.kGravity;
    double v = velocity;
    double pitch;

    double uShoot = Constants.TrajectoryCalculations.interpolationShoot;
    double uLob = Constants.TrajectoryCalculations.interpolationLobber;

    if(RobotStatus.isLobbing()) {
      if(RobotStatus.getCurrentFieldPosition() == "RightLob") {
        dy = Constants.TrajectoryCalculations.kRightLobY;
        dx = Constants.TrajectoryCalculations.kLobX;
      } else {
        dy = Constants.TrajectoryCalculations.kLeftLobY;
        dx = Constants.TrajectoryCalculations.kLobX;
      }
    } else if(RobotStatus.isShooting()) {
      dy = Constants.TrajectoryCalculations.shooterToHubHeight;
      dx = Math.hypot(getDistanceFromHub(robot).getX(), getDistanceFromHub(robot).getY());
    } else {
      dx = 0.0;
      dy = 0.0;
    }


    double D = 1 - (2 * g * dy) / Math.pow(v, 2) - (Math.pow(g, 2) * Math.pow(dx, 2)) / Math.pow(v, 4);
    double T1 = (Math.pow(v, 2) / (g * dx)) * (1 + Math.sqrt(D)); // Higher angle
    double T2 = (Math.pow(v, 2) / (g * dx)) * (1 - Math.sqrt(D)); // Lower angle

    double vMin = Math.sqrt(g * (dy + Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2))));
    double TMin = Math.pow(vMin, 2) / (g * dx);

    double angleLow = Math.toDegrees(Math.atan(T2));
    double anleHigh = Math.toDegrees(Math.atan(T1));
    double angleInterpShoot = Math.toDegrees(Math.atan(uShoot * T1 + (1 - uShoot) * T2));
    double angleInterpLob = Math.toDegrees(Math.atan(uLob * T1 + (1 - uLob) * T2));
    double angleVMin = Math.toDegrees(Math.atan(TMin));

    if(RobotStatus.isLobbing()) {
      pitch = angleInterpLob;
    } else if (RobotStatus.isShooting()) {
      pitch =
          (dx< Constants.TrajectoryCalculations.piecewiseSwapCalculationDistance) ? angleInterpShoot : angleVMin;
    } else {
      pitch = 0.0;
    }

    if(D < 0) {
      pitch = angleVMin;
    }

    return pitch;
  }

  public double calculateShooterPitchinMotion(Pose2d robot, double velocityX, double velocityY,
      double angularVelocityDegrees,
      double launchVelocity, double launchPitch) {

    double offsetX = velocityX * calculateTimeOfFlight(robot, launchVelocity, launchPitch); // Get the offset by getting the product of the x velocity and the time of flight
    double offsetY = velocityY * calculateTimeOfFlight(robot, launchVelocity, launchPitch); // Same with y
    double offsetRot = angularVelocityDegrees * calculateTimeOfFlight(robot, launchVelocity, launchPitch); // Same but with angular velocity
    return calculateShooterPitchDegrees(new Pose2d(robot.getX() + offsetX, robot.getY() + offsetY,
        robot.getRotation().plus(new Rotation2d(Math.toRadians(offsetRot)))), launchVelocity); // Returns a new pose2d with the offset added
  }
}
