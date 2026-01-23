// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSub;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class DriveToPoseCmd extends Command {
  private DrivetrainSub m_drivetrainSub;
  private Pose2d m_targetPose;
  private Pose2d m_currentPose;
  private Transform2d m_error;

  private final SwerveRequest.FieldCentric m_autoDrive = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.Velocity).withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

  private final double m_driveP = 0.0;
  private final double m_rotP = 0.0;

  private final double m_feedforward = 0.0;
  private final double m_rotationalFeedForward = 0.0;

  /** Creates a new DriveToPoseCmd. */
  public DriveToPoseCmd(Pose2d targetPose, DrivetrainSub drivetrainSub) {
    m_targetPose = targetPose;
    m_drivetrainSub = drivetrainSub;

    addRequirements(m_drivetrainSub);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_error = new Transform2d();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_currentPose = m_drivetrainSub.getState().Pose;

    m_error = new Transform2d(m_targetPose.getX() - m_currentPose.getX(), m_targetPose.getY() - m_currentPose.getY(),
        m_targetPose.getRotation().minus(m_currentPose.getRotation()));

    Translation2d outputDrivePower = m_error.getTranslation().times(m_driveP);

    double outputRotPower = m_error.getRotation().getDegrees() * m_rotP;

    double xPower = outputDrivePower.getX() + m_feedforward;
    double yPower = outputDrivePower.getY() + m_feedforward;

    double rotPower = outputRotPower + m_rotationalFeedForward;

    m_drivetrainSub.setControl(m_autoDrive.withVelocityX(xPower).withVelocityY(yPower).withRotationalRate(rotPower));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSub.setControl(m_autoDrive.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((m_targetPose.getX() - m_currentPose.getX() < 0.0) && (m_targetPose.getY() - m_currentPose.getY() < 0.0)
        && (m_targetPose.getRotation().getDegrees() - m_currentPose.getRotation().getDegrees() < 0.0)) {
      return true;
    }
    return false;
  }
}
// Things left to do: Tuning for P values, tune feed forward, pick a margin of error for the "is finished", stop x, y, and roatation values when they reach their end position
