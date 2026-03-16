// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Arrays;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.interpolation.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

/** Add your docs here. */
public class ShooterAimingCalcs {
  private double targetYawAngle;
  private double robotYawAngle;
  private double relativeTargetYawAngle;

  static InterpolatingDoubleTreeMap m_distanceToFlywheelMap = new InterpolatingDoubleTreeMap();
  static InterpolatingDoubleTreeMap m_distanceToPitchMap = new InterpolatingDoubleTreeMap();
  static {
    //This will need to be derived experementally, currentlty based on theoretical values
    // Key is distance to target in metres
    // Value is flywheel speed in rps
    m_distanceToFlywheelMap.put(0.0, 40.0); // Change this to minimum flywheel speed
    m_distanceToFlywheelMap.put(1.0, 40.0);
    m_distanceToFlywheelMap.put(1.5, 40.0);
    m_distanceToFlywheelMap.put(2.0, 44.0);
    m_distanceToFlywheelMap.put(2.5, 48.0);
    m_distanceToFlywheelMap.put(3.0, 52.0);
    m_distanceToFlywheelMap.put(3.5, 56.0);
    m_distanceToFlywheelMap.put(4.0, 60.0);
    m_distanceToFlywheelMap.put(4.5, 64.0);
    m_distanceToFlywheelMap.put(5.0, 68.0);
    m_distanceToFlywheelMap.put(6.0, 76.0);
    m_distanceToFlywheelMap.put(7.0, 84.0);
    m_distanceToFlywheelMap.put(8.0, 92.0);
    m_distanceToFlywheelMap.put(10.0, 100.0);
    m_distanceToFlywheelMap.put(12.0, 110.0);
    m_distanceToFlywheelMap.put(14.0, 115.0);
    m_distanceToFlywheelMap.put(16.540988, Constants.Shooter.kFlywheelMaxVelocityRotsPerSec);

    //This will need to be derived experementally, currentlty based on theoretical values
    // Key is distance to target in metres
    // Value is pitch angle in degrees
    m_distanceToPitchMap.put(0.0, Constants.Shooter.kPitchMaxAngleDeg);
    m_distanceToPitchMap.put(1.0, 49.6);
    m_distanceToPitchMap.put(1.5, 46.0);
    m_distanceToPitchMap.put(2.0, 44.0);
    m_distanceToPitchMap.put(2.5, 42.0);
    m_distanceToPitchMap.put(3.0, 40.0);
    m_distanceToPitchMap.put(3.5, 38.0);
    m_distanceToPitchMap.put(4.0, 36.0);
    m_distanceToPitchMap.put(4.5, 34.0);
    m_distanceToPitchMap.put(5.0, 32.0);
    m_distanceToPitchMap.put(6.0, 28.0);
    m_distanceToPitchMap.put(7.0, 24.0);
    m_distanceToPitchMap.put(8.0, 20.7);
    m_distanceToPitchMap.put(10.0, 20.7);
    m_distanceToPitchMap.put(12.0, 20.7);
    m_distanceToPitchMap.put(14.0, 20.7);
    m_distanceToPitchMap.put(16.540988, Constants.Shooter.kPitchMinAngleDeg);
  }

  public static double getInterpolatedFlywheelVelocity(double distance) {
    return m_distanceToFlywheelMap.get(distance);
  }

  public static double getInterpolatedPitchAngle(double distance) {
    return m_distanceToPitchMap.get(distance);
  }

  public double calculateShooterFlywheelRps(Pose2d robot) {
    double rps;
    if(RobotStatus.isLobbing()) {
      if(RobotStatus.getCurrentFieldPosition() == "RightLob") {
        rps = getInterpolatedFlywheelVelocity(getDistanceFromRightLob(robot).getNorm());
      } else {
        rps = getInterpolatedFlywheelVelocity(getDistanceFromLeftLob(robot).getNorm());
      }
    } else if(RobotStatus.isShooting()) {
      rps = getInterpolatedFlywheelVelocity(getDistanceFromHub(robot).getNorm());
    } else {
      if(RobotStatus.wasLobbing()) {
        if(RobotStatus.getPreviousFieldPosition() == "RightLob") {
          rps = getInterpolatedFlywheelVelocity(getDistanceFromRightLob(robot).getNorm());
        } else {
          rps = getInterpolatedFlywheelVelocity(getDistanceFromLeftLob(robot).getNorm());
        }
      } else if(RobotStatus.wasShooting()) {
        rps = getInterpolatedFlywheelVelocity(getDistanceFromHub(robot).getNorm());
      } else {
        rps = 0.0;
      }
    }
    return rps;
  }

  public double calculateShooterYawDegrees(Pose2d robot) {
    robotYawAngle = robot.getRotation().getDegrees() + 180; //robot angle 0 to 360
    if(RobotStatus.isLobbing()) {
      if(RobotStatus.getCurrentFieldPosition() == "RightLob") {
        targetYawAngle =
            Math.toDegrees(Math.atan2(getDistanceFromRightLob(robot).getY(), getDistanceFromRightLob(robot).getX()))
                + 180 + 315;//315 is turret starting rotational offset
      } else {
        targetYawAngle =
            Math.toDegrees(Math.atan2(getDistanceFromLeftLob(robot).getY(), getDistanceFromLeftLob(robot).getX())) + 180
                + 315;//315 is turret starting rotational offset
      }
    } else if(RobotStatus.isShooting()) {
      targetYawAngle =
          Math.toDegrees(Math.atan2(getDistanceFromHub(robot).getY(), getDistanceFromHub(robot).getX())) + 180 + 315;//315 is turret starting rotational offset
    } else {
      if(RobotStatus.wasLobbing()) {
        if(RobotStatus.getPreviousFieldPosition() == "RightLob") {
          targetYawAngle =
              Math.toDegrees(Math.atan2(getDistanceFromRightLob(robot).getY(), getDistanceFromRightLob(robot).getX()))
                  + 180 + 315;//315 is turret starting rotational offset
        } else {
          targetYawAngle =
              Math.toDegrees(Math.atan2(getDistanceFromLeftLob(robot).getY(), getDistanceFromLeftLob(robot).getX()))
                  + 180 + 315;//315 is turret starting rotational offset
        }
      } else if(RobotStatus.wasShooting()) {
        targetYawAngle =
            Math.toDegrees(Math.atan2(getDistanceFromHub(robot).getY(), getDistanceFromHub(robot).getX())) + 180 + 315;//315 is turret starting rotational offset
      } else {
        targetYawAngle = 0.0;
      }
    }

    relativeTargetYawAngle = (360 + (targetYawAngle - robotYawAngle)) % 360;
    return relativeTargetYawAngle;
  }

  public double calculateTimeOfFlight(Pose2d robot) {
    double distanceX = getDistanceFromHub(robot).getX();
    double velocityX =
        flywheelRpsToVelocity(calculateShooterFlywheelRps(robot))
            * Math.cos(Math.toRadians(calculateShooterPitchDegrees(robot)[0]));
    //System.out.println(calculateShooterPitchDegrees(robot)[0]);
    return distanceX / velocityX;
  }

  public double flywheelRpsToVelocity(double rps) {
    return rps * 1.0;
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

  public Translation2d getDistanceFromLeftLob(Pose2d robot) {
    Alliance alliance = GameData.getAlliance();
    Translation2d distToLob;
    Pose2d turretPos = robot.plus(new Transform2d(-0.1, 0.1, new Rotation2d(0.0)));

    if(alliance == Alliance.Blue) {
      distToLob = new Translation2d(Constants.TrajectoryCalculations.kBlueLobX - turretPos.getX(),
          Constants.TrajectoryCalculations.kLeftLobY - turretPos.getY());
    } else {
      distToLob = new Translation2d(Constants.TrajectoryCalculations.kRedLobX - turretPos.getX(),
          Constants.TrajectoryCalculations.kLeftLobY - turretPos.getY());
    }
    return distToLob;
  }

  public Translation2d getDistanceFromRightLob(Pose2d robot) {
    Alliance alliance = GameData.getAlliance();
    Translation2d distToLob;
    Pose2d turretPos = robot.plus(new Transform2d(-0.1, 0.1, new Rotation2d(0.0)));

    if(alliance == Alliance.Blue) {
      distToLob = new Translation2d(Constants.TrajectoryCalculations.kBlueLobX - turretPos.getX(),
          Constants.TrajectoryCalculations.kRightLobY - turretPos.getY());
    } else {
      distToLob = new Translation2d(Constants.TrajectoryCalculations.kRedLobX - turretPos.getX(),
          Constants.TrajectoryCalculations.kRightLobY - turretPos.getY());
    }
    return distToLob;
  }

  public double[] calculateShooterPitchDegrees(Pose2d robot) {
    // link to the visual trajectory calculation desmos graph
    // desmos.com/calculator/kd3svmcurr
    // link to the proof for each equation
    // desmos.com/calculator/er6sltycq6
    // 0 degrees output is horizontal, 90 is vertical pointing up

    double dx;
    double dy;
    double g = Constants.Shooter.kGravity;
    double v = flywheelRpsToVelocity(calculateShooterFlywheelRps(robot));
    double pitch;

    double uShoot = Constants.TrajectoryCalculations.interpolationShoot;
    double uLob = Constants.TrajectoryCalculations.interpolationLobber;

    if(RobotStatus.isLobbing()) {
      if(RobotStatus.getCurrentFieldPosition() == "RightLob") {
        //System.out.println("rl");
        dy = -Constants.TrajectoryCalculations.kShooterToFloorHeight;
        dx = getDistanceFromRightLob(robot).getNorm();
      } else {
        //System.out.println("ll");
        dy = -Constants.TrajectoryCalculations.kShooterToFloorHeight;
        dx = getDistanceFromLeftLob(robot).getNorm();
      }
    } else if(RobotStatus.isShooting()) {
      //System.out.println("s");
      dy = Constants.TrajectoryCalculations.shooterToHubHeight;
      dx = Math.hypot(getDistanceFromHub(robot).getX(), getDistanceFromHub(robot).getY());
    } else {
      if(RobotStatus.wasLobbing()) {
        if(RobotStatus.getPreviousFieldPosition() == "RightLob") {
          //System.out.println("rl");
          dy = -Constants.TrajectoryCalculations.kShooterToFloorHeight;
          dx = getDistanceFromRightLob(robot).getNorm();
        } else {
          //System.out.println("ll");
          dy = -Constants.TrajectoryCalculations.kShooterToFloorHeight;
          dx = getDistanceFromLeftLob(robot).getNorm();
        }
      } else if(RobotStatus.wasShooting()) {
        //System.out.println("s");
        dy = Constants.TrajectoryCalculations.shooterToHubHeight;
        dx = Math.hypot(getDistanceFromHub(robot).getX(), getDistanceFromHub(robot).getY());
      } else {
        //System.out.println("bad :(");
        dx = 0.0;
        dy = 0.0;
      }
    }


    //dx, dy, g, v, pitch

    double D = 1 - (2 * g * dy) / Math.pow(v, 2) - (Math.pow(g, 2) * Math.pow(dx, 2)) / Math.pow(v, 4);
    double T1 = (Math.pow(v, 2) / (g * dx)) * (1 + Math.sqrt(D)); // Higher angle
    double T2 = (Math.pow(v, 2) / (g * dx)) * (1 - Math.sqrt(D)); // Lower angle
    //System.out.println("T1: " + T1 + ", T2: " + T2 + ", D: " + D);

    double vMin = Math.sqrt(g * (dy + Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2))));
    double TMin = Math.pow(vMin, 2) / (g * dx);

    double angleLow = Math.toDegrees(Math.atan(T2));
    double angleHigh = Math.toDegrees(Math.atan(T1));
    double angleInterpShoot = Math.toDegrees(Math.atan(uShoot * T1 + (1 - uShoot) * T2));
    double angleInterpLob = Math.toDegrees(Math.atan(uLob * T1 + (1 - uLob) * T2));
    double angleVMin = Math.toDegrees(Math.atan(TMin));

    double finalVelocity = 0.0;

    if(RobotStatus.isLobbing()) {
      pitch = angleInterpLob;
      //System.out.println("angleInterplob:" + pitch);
    } else if(RobotStatus.isShooting()) {
      pitch =
          (dx < Constants.TrajectoryCalculations.piecewiseSwapCalculationDistance) ? angleInterpShoot : angleVMin;
      //System.out.println("shootOrVmin:" + pitch);
      finalVelocity = (dx < Constants.TrajectoryCalculations.piecewiseSwapCalculationDistance) ? 0.0 : vMin;
    } else {
      pitch = 0.0;
      //System.out.println("zero:" + pitch);
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

  public double[] calculationsInMotion(Pose2d robot, ChassisSpeeds velocity) {
    double velocityX = velocity.vxMetersPerSecond;
    double velocityY = velocity.vyMetersPerSecond;
    //double angularVelocityDegrees = Math.toDegrees(velocity.omegaRadiansPerSecond);
    double offsetX = velocityX * calculateTimeOfFlight(robot); //realistically i dont see a better alternative to just using the current position to calculate the pitch of the shooter since it's necessary calculate the tof. It should be fine, since the value will be close enough to correct but we can always just add a fudge-facor based on our velocity in each direction 
    double offsetY = velocityY * calculateTimeOfFlight(robot);
    //double offsetRot = angularVelocityDegrees * calculateTimeOfFlight(robot);
    Pose2d offsetPos = new Pose2d(robot.getX() - offsetX, robot.getY() - offsetY,
        robot.getRotation());
    double[] pitchAndVelocity = calculateShooterPitchDegrees(offsetPos);
    double[] trajectoriesArray = {pitchAndVelocity[0], calculateShooterYawDegrees(offsetPos),
        pitchAndVelocity[1], offsetPos.getX(), offsetPos.getY(), offsetPos.getRotation().getDegrees()};
    if(pitchAndVelocity[1] == 0.0) {
      trajectoriesArray[2] = calculateShooterFlywheelRps(offsetPos);
    }

    System.out.println(Arrays.toString(trajectoriesArray));
    //System.out.println(calculateTimeOfFlight(robot));
    return trajectoriesArray;
  }
}
