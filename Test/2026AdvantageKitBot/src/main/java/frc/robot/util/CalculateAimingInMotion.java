package frc.robot.util;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.Shooter;

public class CalculateAimingInMotion {

  public static Translation2d targetGoal(SwerveDriveState swerveDriveState) {

    Alliance alliance = GameData.getAlliance();
    Translation2d futureGoal;

    if (alliance == Alliance.Red) {
      futureGoal =
          new Translation2d(Constants.FieldElements.kRedHubX, Constants.FieldElements.kRedHubY);
    } else {
      futureGoal =
          new Translation2d(Constants.FieldElements.kBlueHubX, Constants.FieldElements.kBlueHubY);
    }

    Pose2d currentRobotPose = swerveDriveState.Pose;
    for (int i = 0; i < 5; i++) {
      double distance = currentRobotPose.getTranslation().minus(futureGoal).getNorm();
      //double time = ShooterSub.ballFlightTime(distance);
      // Translation2d movement = swerveDriveState.Speeds

      // how long to shoot from current distance in seconds
      // that times current velocity  added to goal is new goal location
      //
    }
    return futureGoal;
  }
}
