// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

/** Add your docs here. */
public class ShooterAimingCalcs {

  static InterpolatingDoubleTreeMap m_distanceToFlywheelMap = new InterpolatingDoubleTreeMap();
  static InterpolatingDoubleTreeMap m_distanceToPitchMap = new InterpolatingDoubleTreeMap();
  static {
    //This will need to be derived experementally, currentlty based on theoretical values
    // Key is distance to target in metres
    // Value is flywheel speed in rps
    m_distanceToFlywheelMap.put(0.0, Constants.Shooter.kFlywheelMinVelocityRotsPerSec); // Change this to minimum flywheel speed
    m_distanceToFlywheelMap.put(1.5, 60.0);
    m_distanceToFlywheelMap.put(2.0, 63.0);
    m_distanceToFlywheelMap.put(2.5, 55.0);
    m_distanceToFlywheelMap.put(3.0, 60.0);
    m_distanceToFlywheelMap.put(3.5, 65.0);
    m_distanceToFlywheelMap.put(4.0, 70.0);
    m_distanceToFlywheelMap.put(4.5, 78.0);
    m_distanceToFlywheelMap.put(5.0, 68.0);
    m_distanceToFlywheelMap.put(5.5, 67.0);
    m_distanceToFlywheelMap.put(16.540988, Constants.Shooter.kFlywheelMaxVelocityRotsPerSec);

    //This will need to be derived experementally, currentlty based on theoretical values
    // Key is distance to target in metres
    // Value is pitch angle in degrees
    m_distanceToPitchMap.put(0.0, Constants.Shooter.kPitchMinAngleDeg);
    m_distanceToPitchMap.put(1.5, 20.8);
    m_distanceToPitchMap.put(2.0, 24.0);
    m_distanceToPitchMap.put(2.5, 27.5);
    m_distanceToPitchMap.put(3.0, 29.0);
    m_distanceToPitchMap.put(3.5, 30.0);
    m_distanceToPitchMap.put(4.0, 30.0);
    m_distanceToPitchMap.put(4.5, 31.0);
    m_distanceToPitchMap.put(5.0, 33.0);
    m_distanceToPitchMap.put(5.5, 33.0);
    m_distanceToPitchMap.put(16.540988, Constants.Shooter.kPitchMaxAngleDeg);
  }

  public double getInterpolatedFlywheelVelocity(double distance) {
    return m_distanceToFlywheelMap.get(distance);
  }

  public double getInterpolatedPitchAngle(double distance) {
    return m_distanceToPitchMap.get(distance);
  }

  public double getDistanceToHub(Pose2d robot) {
    Alliance alliance = GameData.getAlliance();
    Pose2d target;

    if(alliance == Alliance.Blue) {
      target = new Pose2d(Constants.FieldElements.kBlueHubX, Constants.FieldElements.kBlueHubY, new Rotation2d(0.0));
    } else {
      target = new Pose2d(Constants.FieldElements.kRedHubX, Constants.FieldElements.kRedHubY, new Rotation2d(0.0));
    }

    double distance = target.minus(robot).getTranslation().getNorm();
    return distance;
  }

  public double calculateShooterFlywheelRps(Pose2d current, Pose2d target) {
    return getInterpolatedFlywheelVelocity(target.minus(current).getTranslation().getNorm());
  }

  public double calculateShooterYawDegrees(Pose2d current, Pose2d target) {
    double robotYawAngle = current.getRotation().getDegrees(); //robot angle -180 to 180
    double targetYawAngle =
        Math.toDegrees(Math.atan2(target.getY() - current.getY(), target.getX() - current.getX()));
    // System.out.println((target.getY() - current.getY()) + ", " + (target.getX() - current.getX()));
    // System.out.println(Math.toDegrees(Math.atan2(target.getY() - current.getY(), target.getX() - current.getX())));
    // System.out.println(robotYawAngle + ", " + targetYawAngle);
    return (targetYawAngle - robotYawAngle);
  }

  public double calculateTimeOfFlight(Pose2d current, Pose2d target, double flywheelVelocity, double PitchAngleDeg) {
    double distanceX = target.minus(current).getTranslation().getNorm();
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
    dx = target.minus(current).getTranslation().getNorm();

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

    double distance = (target.minus(current)).getTranslation().getNorm(); //please test this is might break everything because i don't knw if subtracting teh rotations causes issues.
    // double[] pitchAngleDegAndFlywheelVelocity =
    //     calculateShooterPitchDegrees(current, target, calculateShooterFlywheelRps(current, target), isLobbing); don't need this anymore lol, it uses the pure math calcs whcih we didnt get to tune
    double pitchAngleDeg = getInterpolatedPitchAngle(distance);//pitchAngleDegAndFlywheelVelocity[0];
    double flywheelVelocity = getInterpolatedFlywheelVelocity(distance);//pitchAngleDegAndFlywheelVelocity[1];
    double targetX = target.getX();
    double targetY = target.getY();
    double tof = calculateTimeOfFlight(current, target, flywheelVelocity, pitchAngleDeg);
    Pose2d offsetPos;

    if(RobotStatus.isCompensateForMotion()) {
      for(int i = 0; i <= 3; i++) {
        targetX = (target.getX() + velocity.vxMetersPerSecond) * tof;
        targetY = (target.getY() + velocity.vyMetersPerSecond) * tof;
        distance = (new Pose2d(targetX, targetY, new Rotation2d(0.0)).minus(current)).getTranslation().getNorm();
        pitchAngleDeg = getInterpolatedPitchAngle(distance);
        flywheelVelocity = getInterpolatedFlywheelVelocity(distance);
        tof = calculateTimeOfFlight(current, new Pose2d(targetX, targetY, new Rotation2d(0.0)), flywheelVelocity,
            pitchAngleDeg);
      }
    }
    //double angularVelocityDegrees = Math.toDegrees(velocity.omegaRadiansPerSecond);
    // double tof = calculateTimeOfFlight(current, target, flywheelVelocity, pitchAngleDeg);
    // double offsetX = velocityX * tof; //realistically i dont see a better alternative to just using the current position to calculate the pitch of the shooter since it's necessary calculate the tof. It should be fine, since the value will be close enough to correct but we can always just add a fudge-facor based on our velocity in each direction 
    // double offsetY = velocityY * tof;
    //double offsetRot = angularVelocityDegrees * calculateTimeOfFlight(robot);
    offsetPos = new Pose2d(targetX, targetY, new Rotation2d(0.0)); // Pose2d offsetPos = new Pose2d(target.getX() + offsetX, target.getY() + offsetY, new Rotation2d(0.0));
    // double[] pitchAndVelocity =
    //     calculateShooterPitchDegrees(current, offsetPos, calculateShooterFlywheelRps(current, offsetPos), isLobbing);
    // MIGHT NEEDS TO GET THE NEW PITCH AND FLYWHEEL VELOCITY IF WE RE ADD THE ROBOT VELOCITY COMPENSATION
    double[] trajectoriesArray = {pitchAngleDeg, calculateShooterYawDegrees(current, offsetPos),
        flywheelVelocity, offsetPos.getX(), offsetPos.getY(), offsetPos.getRotation().getDegrees()};

    //System.out.println(Arrays.toString(trajectoriesArray));
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
        if(RobotStatus.getCurrentFieldPosition().equals("RightLob")) {
          target = new Pose2d(Constants.TrajectoryCalculations.kBlueLobX, Constants.TrajectoryCalculations.kRightLobY,
              new Rotation2d(0.0));
        } else {
          target = new Pose2d(Constants.TrajectoryCalculations.kBlueLobX, Constants.TrajectoryCalculations.kLeftLobY,
              new Rotation2d(0.0));
        }
      } else if(RobotStatus.isShooting()) {
        target = new Pose2d(Constants.FieldElements.kBlueHubX, Constants.FieldElements.kBlueHubY, robot.getRotation());
      } else {
        if(RobotStatus.wasLobbing()) {
          isLobbing = true;
          if(RobotStatus.getPreviousFieldPosition().equals("RightLob")) {
            target = new Pose2d(Constants.TrajectoryCalculations.kBlueLobX, Constants.TrajectoryCalculations.kRightLobY,
                new Rotation2d(0.0));
          } else {
            target = new Pose2d(Constants.TrajectoryCalculations.kBlueLobX, Constants.TrajectoryCalculations.kLeftLobY,
                new Rotation2d(0.0));
          }
        } else if(RobotStatus.wasShooting()) {
          target =
              new Pose2d(Constants.FieldElements.kBlueHubX, Constants.FieldElements.kBlueHubY, new Rotation2d(0.0));
        } else {
          target = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
        }
      }
    } else {
      if(RobotStatus.isLobbing()) {
        isLobbing = true;
        if(RobotStatus.getCurrentFieldPosition().equals("RightLob")) {
          target = new Pose2d(Constants.TrajectoryCalculations.kRedLobX, Constants.TrajectoryCalculations.kRightLobY,
              new Rotation2d(0.0));
        } else {
          target = new Pose2d(Constants.TrajectoryCalculations.kRedLobX, Constants.TrajectoryCalculations.kLeftLobY,
              new Rotation2d(0.0));
        }
      } else if(RobotStatus.isShooting()) {
        target = new Pose2d(Constants.FieldElements.kRedHubX, Constants.FieldElements.kRedHubY, new Rotation2d(0.0));
      } else {
        if(RobotStatus.wasLobbing()) {
          isLobbing = true;
          if(RobotStatus.getPreviousFieldPosition().equals("RightLob")) {
            target = new Pose2d(Constants.TrajectoryCalculations.kRedLobX, Constants.TrajectoryCalculations.kRightLobY,
                new Rotation2d(0.0));
          } else {
            target = new Pose2d(Constants.TrajectoryCalculations.kRedLobX, Constants.TrajectoryCalculations.kLeftLobY,
                new Rotation2d(0.0));
          }
        } else if(RobotStatus.wasShooting()) {
          target =
              new Pose2d(Constants.FieldElements.kRedHubX, Constants.FieldElements.kRedHubY, new Rotation2d(0.0));
        } else {
          target = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
        }
      }
    }
    return calculationsInMotion(robot, target, velocity, isLobbing);
  }
}
