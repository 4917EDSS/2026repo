// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructTopic;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DrivetrainSub;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCmd extends Command {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("DriveState");
  private final SwerveRequest.RobotCentric testDrive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);
  DrivetrainSub m_drivetrainSub;
  double distanceToDrive;
  NetworkTableEntry currentPoseEntry;
  Pose2d currentPose;

  /** Creates a new driveCmd. */
  public DriveCmd(DrivetrainSub drivetrainSub) {
    m_drivetrainSub = drivetrainSub;
    addRequirements(m_drivetrainSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentPoseEntry = table.getEntry("pose");
    distanceToDrive = 15; //Subject to change
    m_drivetrainSub.setControl(testDrive.withVelocityX(99999999).withVelocityY(0).withRotationalRate(0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //currentPose2d.get
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSub.setControl(testDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;
  }
}
