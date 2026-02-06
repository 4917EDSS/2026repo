// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

/**
 * Command to drive the robot to a target pose using ProfiledPIDControllers. Uses separate
 * controllers for X, Y, and rotation (theta) for smooth, trapezoid-profiled motion.
 */
public class DriveToPoseCmd extends Command {
  private final Drive m_drivetrainSub;
  private final Pose2d m_targetPose;

  // Profiled PID Controllers for X, Y, and Theta
  private final ProfiledPIDController m_xController;
  private final ProfiledPIDController m_yController;
  private final ProfiledPIDController m_thetaController;

  // Tolerances for determining when we've reached the target
  private static final double X_TOLERANCE = 0.02; // meters
  private static final double Y_TOLERANCE = 0.02; // meters
  private static final double THETA_TOLERANCE = 2.0; // degrees

  /**
   * Creates a new DriveToPoseCmd.
   *
   * @param targetPose The target pose to drive to
   * @param drivetrainSub The drivetrain subsystem
   */
  public DriveToPoseCmd(Pose2d targetPose, Drive drivetrainSub) {
    m_targetPose = targetPose;
    m_drivetrainSub = drivetrainSub;

    // Configure X Controller
    // TrapezoidProfile.Constraints(max velocity m/s, max acceleration m/s^2)
    m_xController =
        new ProfiledPIDController(
            2.0, // kP - tune this value
            0.0, // kI
            0.0, // kD
            new TrapezoidProfile.Constraints(3.0, 2.0) // max vel, max accel
            );
    m_xController.setTolerance(X_TOLERANCE);

    // Configure Y Controller
    m_yController =
        new ProfiledPIDController(
            2.0, // kP - tune this value
            0.0, // kI
            0.0, // kD
            new TrapezoidProfile.Constraints(3.0, 2.0) // max vel, max accel
            );
    m_yController.setTolerance(Y_TOLERANCE);

    // Configure Theta Controller
    m_thetaController =
        new ProfiledPIDController(
            3.0, // kP - tune this value
            0.0, // kI
            0.0, // kD
            new TrapezoidProfile.Constraints(Math.PI * 2, Math.PI * 2) // rad/s, rad/s^2
            );
    m_thetaController.setTolerance(Math.toRadians(THETA_TOLERANCE));
    // Enable continuous input for theta (wraps around at -π to π)
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(m_drivetrainSub);
  }

  /** Alternative constructor with custom PID values and constraints. */
  public DriveToPoseCmd(
      Pose2d targetPose,
      Drive drivetrainSub,
      double xP,
      double xI,
      double xD,
      double maxXVel,
      double maxXAccel,
      double yP,
      double yI,
      double yD,
      double maxYVel,
      double maxYAccel,
      double thetaP,
      double thetaI,
      double thetaD,
      double maxThetaVel,
      double maxThetaAccel) {

    m_targetPose = targetPose;
    m_drivetrainSub = drivetrainSub;

    m_xController =
        new ProfiledPIDController(xP, xI, xD, new TrapezoidProfile.Constraints(maxXVel, maxXAccel));
    m_xController.setTolerance(X_TOLERANCE);

    m_yController =
        new ProfiledPIDController(yP, yI, yD, new TrapezoidProfile.Constraints(maxYVel, maxYAccel));
    m_yController.setTolerance(Y_TOLERANCE);

    m_thetaController =
        new ProfiledPIDController(
            thetaP, thetaI, thetaD, new TrapezoidProfile.Constraints(maxThetaVel, maxThetaAccel));
    m_thetaController.setTolerance(Math.toRadians(THETA_TOLERANCE));
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(m_drivetrainSub);
  }

  @Override
  public void initialize() {
    // Get current pose
    Pose2d currentPose = m_drivetrainSub.getPose();

    // Reset and set goals for all controllers
    m_xController.reset(currentPose.getX());
    m_xController.setGoal(m_targetPose.getX());

    m_yController.reset(currentPose.getY());
    m_yController.setGoal(m_targetPose.getY());

    m_thetaController.reset(currentPose.getRotation().getRadians());
    m_thetaController.setGoal(m_targetPose.getRotation().getRadians());

    Logger.recordOutput("DriveToPose/TargetPose", m_targetPose);
    Logger.recordOutput("DriveToPose/InitialPose", currentPose);
  }

  @Override
  public void execute() {
    // Get current pose
    Pose2d currentPose = m_drivetrainSub.getPose();

    // Calculate velocity outputs from profiled PID controllers
    double xVelocity = m_xController.calculate(currentPose.getX());
    double yVelocity = m_yController.calculate(currentPose.getY());
    double rotationalVelocity = m_thetaController.calculate(currentPose.getRotation().getRadians());

    ChassisSpeeds speeds = new ChassisSpeeds(xVelocity, yVelocity, rotationalVelocity);
    m_drivetrainSub.runVelocity(speeds);

    // Log data for AdvantageScope
    Logger.recordOutput("DriveToPose/CurrentPose", currentPose);
    Logger.recordOutput("DriveToPose/TargetPose", m_targetPose);

    // Log errors
    Logger.recordOutput("DriveToPose/XError", m_targetPose.getX() - currentPose.getX());
    Logger.recordOutput("DriveToPose/YError", m_targetPose.getY() - currentPose.getY());
    Logger.recordOutput(
        "DriveToPose/ThetaError",
        m_targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians());

    // Log commanded velocities
    Logger.recordOutput("DriveToPose/XVelocity", xVelocity);
    Logger.recordOutput("DriveToPose/YVelocity", yVelocity);
    Logger.recordOutput("DriveToPose/RotationalVelocity", rotationalVelocity);

    // Log setpoints from the profiled controllers
    Logger.recordOutput("DriveToPose/XSetpoint", m_xController.getSetpoint().position);
    Logger.recordOutput("DriveToPose/YSetpoint", m_yController.getSetpoint().position);
    Logger.recordOutput("DriveToPose/ThetaSetpoint", m_thetaController.getSetpoint().position);

    // Log goal status
    Logger.recordOutput("DriveToPose/XAtGoal", m_xController.atGoal());
    Logger.recordOutput("DriveToPose/YAtGoal", m_yController.atGoal());
    Logger.recordOutput("DriveToPose/ThetaAtGoal", m_thetaController.atGoal());
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain
    ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    m_drivetrainSub.runVelocity(speeds);

    Logger.recordOutput("DriveToPose/Interrupted", interrupted);
    Logger.recordOutput("DriveToPose/FinalPose", m_drivetrainSub.getPose());
  }

  @Override
  public boolean isFinished() {
    // Command is finished when all three controllers are at their goals
    boolean atGoal = m_xController.atGoal() && m_yController.atGoal() && m_thetaController.atGoal();

    Logger.recordOutput("DriveToPose/AtGoal", atGoal);

    return atGoal;
  }
}
