// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Arrays;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

/** Add your docs here. */
public class ShooterAimingCalcs {
  public double calculateShooterFlywheelRps(Pose2d current, Pose2d target) {
    InterpolatingDoubleTreeMap velocityInterpolation = new InterpolatingDoubleTreeMap();
    //This will need to be derived experementally, currentlty based on theoretical values
    velocityInterpolation.put(0.0, 0.0);
    velocityInterpolation.put(1.0, 10.0);
    velocityInterpolation.put(2.0, 10.0);
    velocityInterpolation.put(3.0, 10.0);
    velocityInterpolation.put(4.0, 11.0);
    velocityInterpolation.put(5.0, 11.5);
    velocityInterpolation.put(6.0, 12.0);
    velocityInterpolation.put(7.0, 12.5);
    velocityInterpolation.put(8.0, 13.0);
    velocityInterpolation.put(9.0, 13.0);
    velocityInterpolation.put(10.0, 13.0);
    return velocityInterpolation.get(current.minus(target).getTranslation().getNorm());
  }

  public double calculateShooterYawDegrees(Pose2d current, Pose2d target) {
    double robotYawAngle = current.getRotation().getDegrees() + 180; //robot angle 0 to 360
    double targetYawAngle =
        Math.toDegrees(Math.atan2(current.minus(target).getX(), current.minus(target).getY())) + 180 + 315;
    return (360 + (targetYawAngle - robotYawAngle)) % 360;
  }

  public double calculateTimeOfFlight(Pose2d current, Pose2d target, double flywheelVelocity, double PitchAngleDeg) {
    double distanceX = current.minus(target).getTranslation().getNorm();
    double flywheelVelocityX = flywheelVelocity * Math.cos(PitchAngleDeg);
    return distanceX / flywheelVelocityX;
  }

  public double[] calculateShooterPitchDegrees(Pose2d current, Pose2d target, double flywheelVelocity,
      boolean isLobbing) {
    // link to the visual trajectory calculation desmos graph
    // desmos.com/calculator/kd3svmcurr
    // link to the proof for each equation
    // desmos.com/calculator/er6sltycq6
    // 0 degrees output is horizontal, 90 is vertical pointing up

    double dx;
    double dy;
    double g = Constants.Shooter.kGravity;
    double v = flywheelVelocity;
    double pitch;

    double uShoot = Constants.TrajectoryCalculations.interpolationShoot;
    double uLob = Constants.TrajectoryCalculations.interpolationLobber;

    dy = -Constants.TrajectoryCalculations.kShooterToFloorHeight;
    dx = current.minus(target).getTranslation().getNorm();

    //dx, dy, g, v, pitch

    double D = 1 - (2 * g * dy) / Math.pow(v, 2) - (Math.pow(g, 2) * Math.pow(dx, 2)) / Math.pow(v, 4);
    double T1 = (Math.pow(v, 2) / (g * dx)) * (1 + Math.sqrt(D)); // Higher angle
    double T2 = (Math.pow(v, 2) / (g * dx)) * (1 - Math.sqrt(D)); // Lower angle
    //System.out.println("T1: " + T1 + ", T2: " + T2 + ", D: " + D);

    double vMin = Math.sqrt(g * (dy + Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2))));
    double TMin = Math.pow(vMin, 2) / (g * dx);

    // If needed
    // double angleLow = Math.toDegrees(Math.atan(T2));
    // double angleHigh = Math.toDegrees(Math.atan(T1));
    double angleInterpShoot = Math.toDegrees(Math.atan(uShoot * T1 + (1 - uShoot) * T2));
    double angleInterpLob = Math.toDegrees(Math.atan(uLob * T1 + (1 - uLob) * T2));
    double angleVMin = Math.toDegrees(Math.atan(TMin));

    double finalVelocity = 0.0;

    if(isLobbing) {
      pitch = angleInterpLob;
      //System.out.println("angleInterplob:" + pitch);
    } else {
      pitch =
          (dx < Constants.TrajectoryCalculations.piecewiseSwapCalculationDistance) ? angleInterpShoot : angleVMin;
      //System.out.println("shootOrVmin:" + pitch);
      finalVelocity = (dx < Constants.TrajectoryCalculations.piecewiseSwapCalculationDistance) ? v : vMin;
    }

    if(D < 0) {
      pitch = angleVMin;
      //System.out.println("vmin:" + vMin);
      finalVelocity = vMin;
    }

    double[] returns = {pitch, finalVelocity};
    //System.out.println(pitch + ", " + finalVelocity);
    //System.out.println(dx + ", " + dy + ", " + g + ", " + v + ", " + pitch);
    return returns;
  }

  public double[] calculationsInMotion(Pose2d current, Pose2d target, ChassisSpeeds velocity, boolean isLobbing) {
    double velocityX = velocity.vxMetersPerSecond;
    double velocityY = velocity.vyMetersPerSecond;
    double[] pitchAngleDegAndFlywheelVelocity =
        calculateShooterPitchDegrees(current, target, calculateShooterFlywheelRps(current, target), isLobbing);
    double pitchAngleDeg = pitchAngleDegAndFlywheelVelocity[0];
    double flywheelVelocity = pitchAngleDegAndFlywheelVelocity[1];
    //double angularVelocityDegrees = Math.toDegrees(velocity.omegaRadiansPerSecond);
    double offsetX = velocityX * calculateTimeOfFlight(current, target, flywheelVelocity, pitchAngleDeg); //realistically i dont see a better alternative to just using the current position to calculate the pitch of the shooter since it's necessary calculate the tof. It should be fine, since the value will be close enough to correct but we can always just add a fudge-facor based on our velocity in each direction 
    double offsetY = velocityY * calculateTimeOfFlight(current, target, flywheelVelocity, pitchAngleDeg);
    //double offsetRot = angularVelocityDegrees * calculateTimeOfFlight(robot);
    Pose2d offsetPos = new Pose2d(target.getX() - offsetX, target.getY() - offsetY,
        target.getRotation());
    double[] pitchAndVelocity =
        calculateShooterPitchDegrees(current, offsetPos, calculateShooterFlywheelRps(current, offsetPos), isLobbing);
    double[] trajectoriesArray = {pitchAndVelocity[0], calculateShooterYawDegrees(current, offsetPos),
        pitchAndVelocity[1], offsetPos.getX(), offsetPos.getY(), offsetPos.getRotation().getDegrees()};

    System.out.println(Arrays.toString(trajectoriesArray));
    //System.out.println(calculateTimeOfFlight(robot));
    return trajectoriesArray;
  }

  public double[] setTargets(Pose2d robot, ChassisSpeeds velocity) {
    Alliance alliance = GameData.getAlliance();
    Pose2d target;
    boolean isLobbing = false;

    if(alliance == Alliance.Blue) {
      if(RobotStatus.isLobbing()) {
        isLobbing = true;
        if(RobotStatus.getCurrentFieldPosition() == "RightLob") {
          target = new Pose2d(Constants.TrajectoryCalculations.kBlueLobX, Constants.TrajectoryCalculations.kRightLobY,
              robot.getRotation());
        } else {
          target = new Pose2d(Constants.TrajectoryCalculations.kBlueLobX, Constants.TrajectoryCalculations.kLeftLobY,
              robot.getRotation());
        }
      } else if(RobotStatus.isShooting()) {
        target = new Pose2d(Constants.FieldElements.kBlueHubX, Constants.FieldElements.kBlueHubY, robot.getRotation());
      } else {
        if(RobotStatus.wasLobbing()) {
          isLobbing = true;
          if(RobotStatus.getPreviousFieldPosition() == "RightLob") {
            target = new Pose2d(Constants.TrajectoryCalculations.kBlueLobX, Constants.TrajectoryCalculations.kRightLobY,
                robot.getRotation());
          } else {
            target = new Pose2d(Constants.TrajectoryCalculations.kBlueLobX, Constants.TrajectoryCalculations.kLeftLobY,
                robot.getRotation());
          }
        } else if(RobotStatus.wasShooting()) {
          target =
              new Pose2d(Constants.FieldElements.kBlueHubX, Constants.FieldElements.kBlueHubY, robot.getRotation());
        } else {
          target = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
        }
      }
    } else {
      if(RobotStatus.isLobbing()) {
        isLobbing = true;
        if(RobotStatus.getCurrentFieldPosition() == "RightLob") {
          target = new Pose2d(Constants.TrajectoryCalculations.kRedLobX, Constants.TrajectoryCalculations.kRightLobY,
              robot.getRotation());
        } else {
          target = new Pose2d(Constants.TrajectoryCalculations.kRedLobX, Constants.TrajectoryCalculations.kLeftLobY,
              robot.getRotation());
        }
      } else if(RobotStatus.isShooting()) {
        target = new Pose2d(Constants.FieldElements.kRedHubX, Constants.FieldElements.kRedHubY, robot.getRotation());
      } else {
        if(RobotStatus.wasLobbing()) {
          isLobbing = true;
          if(RobotStatus.getPreviousFieldPosition() == "RightLob") {
            target = new Pose2d(Constants.TrajectoryCalculations.kRedLobX, Constants.TrajectoryCalculations.kRightLobY,
                robot.getRotation());
          } else {
            target = new Pose2d(Constants.TrajectoryCalculations.kRedLobX, Constants.TrajectoryCalculations.kLeftLobY,
                robot.getRotation());
          }
        } else if(RobotStatus.wasShooting()) {
          target =
              new Pose2d(Constants.FieldElements.kRedHubX, Constants.FieldElements.kRedHubY, robot.getRotation());
        } else {
          target = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
        }
      }
    }
    return calculationsInMotion(robot, target, velocity, isLobbing);
  }
}
