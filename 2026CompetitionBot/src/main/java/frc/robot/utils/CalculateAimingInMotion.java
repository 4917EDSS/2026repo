package frc.robot.utils;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.ShooterSub;

public class CalculateAimingInMotion {

  public static Translation2d targetGoal(SwerveDriveState swerveDriveState) {

    Alliance alliance = GameData.getAlliance();
    Translation2d futureGoal;

    if(alliance == Alliance.Red) {
      futureGoal = new Translation2d(4.675, 4.035);
    } else {
      futureGoal = new Translation2d(11.856, 4.035);
    }

    Pose2d currentRobotPose = swerveDriveState.Pose;
    for(int i = 0; i < 5; i++) {
      double distance = currentRobotPose.getTranslation().minus(futureGoal).getNorm();
      double time = ShooterSub.ballFlightTime(distance);
      // Translation2d movement = swerveDriveState.Speeds

      //how long to shoot from current distance in seconds
      //that times current velocity  added to goal is new goal location
      //
    }
    return futureGoal;
  }

}
