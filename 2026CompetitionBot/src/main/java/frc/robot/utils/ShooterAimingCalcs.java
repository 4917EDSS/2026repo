// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class ShooterAimingCalcs {
  private double targetAngle;
  private double robotAngle;

  public double calculateShooterRotation(Pose2d robot) {
    robotAngle = robot.getRotation().getDegrees() + 180; //robot angle 0 to 360
    targetAngle = 360 - robotAngle;
    return targetAngle;
  }
}
