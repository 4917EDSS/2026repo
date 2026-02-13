// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

/** Add your docs here. */
public class CalculateShooterAiming {

  private boolean m_isLobbing = false;

  public double getYawAngle(Pose3d robot) {
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

    // calculate the shooter yaw angle, atan2 works in all quadrants, atan doesn't
    double angle = Math.toDegrees(Math.atan2(distY, distX));

    // get the robot's yaw
    double heading = shooterPoseinfield.toPose2d().getRotation().getDegrees();

    // subtract the robot heading from the shooter angle
    angle -= heading;

    return angle;
  }


  public double getTimeOfFlight(Double range, Double elevation, Double launchVelocity, Double launchAngle) {
    // Use Kinematic equation: y = vy * t - 0.5 * g * t^2
    // Rearanged: 0.5 * g * t^2 - vy * t + elevation = 0
    // Solve using the quaderatic formula 

    // Vertical component of velocity 
    double vy = launchVelocity * Math.sin(launchAngle);


    double a = 0.5 * Constants.Shooter.kGravity;
    double b = -vy;
    double c = elevation;
    double discriminant = b * b - 4 * a * c;

    if(discriminant < 0) {
      return -1;
    }

    // 2 solutions we want the positive one : forward in time
    double t1 = (-b + Math.sqrt(discriminant)) / (2 * a);
    double t2 = (-b - Math.sqrt(discriminant)) / (2 * a);

    return Math.max(t1, t2);

  }

  public double calculateAngleatTarget(double range, double elevation, double launchVelocity, Double launchAngle) {


    double timeOfFlight = getTimeOfFlight(range, elevation, launchVelocity, launchAngle);

    double vx = launchVelocity * Math.cos(launchAngle);

    if(timeOfFlight <= 0) {
      return Double.NaN;
    }

    double vy0 = launchVelocity * Math.sin(launchAngle);
    double vyAtTarget = vy0 * Constants.Shooter.kGravity * timeOfFlight;


    return Math.atan2(vyAtTarget, vx);
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

  public void setLobbingMode (boolean lobbingMode) {
    m_isLobbing = lobbingMode;
  }

  public BooleanSupplier getLobbingMode () {
    BooleanSupplier lobbingSupplier = () -> m_isLobbing;
    return lobbingSupplier;
  }

}
