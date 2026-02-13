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
  private double targetAngle;
  private double robotAngle;
  private Pose2d turretPos;
  private Translation2d distToHub;
  private double relativeTargetAngle;

  public double calculateShooterRotation(Pose2d robot) {
    Alliance alliance = GameData.getAlliance();

    turretPos = robot.plus(new Transform2d(-0.1,0.1, new Rotation2d(0.0)));
    robotAngle = robot.getRotation().getDegrees() + 180; //robot angle 0 to 360

    if (alliance == Alliance.Blue){
      distToHub = new Translation2d(Constants.FieldElements.kBlueHubX-turretPos.getX(), Constants.FieldElements.kBlueHubY-turretPos.getY());
    } else {
      distToHub = new Translation2d(Constants.FieldElements.kRedHubX-turretPos.getX(), Constants.FieldElements.kRedHubY-turretPos.getY());
    }

    targetAngle = Math.toDegrees(Math.atan2(distToHub.getY(), distToHub.getX()))+180;

    relativeTargetAngle = (360+targetAngle-robotAngle)%360;
    
    return relativeTargetAngle;
  }
}
