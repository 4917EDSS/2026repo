// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.utils.ShooterAimingCalcs;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class AimCmd extends Command {
  private final DrivetrainSub m_drivetrainSub;
  private final ShooterSub m_shooterSub;
  private final ShooterAimingCalcs m_shooterAimingCalcs;

  /** Creates a new AimCmd. */
  public AimCmd(DrivetrainSub drivetrainSub, ShooterSub shooterSub, ShooterAimingCalcs shooterAimingCalcs) {
    m_drivetrainSub = drivetrainSub;
    m_shooterSub = shooterSub;
    m_shooterAimingCalcs = shooterAimingCalcs;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterSub.setTargetYawAngle(m_shooterAimingCalcs.calculateShooterYawDegrees(m_drivetrainSub.getPose()));
    m_shooterSub.setTargetPitchAngle(ShooterAimingCalcs
        .getInterpolatedPitchAngle(m_shooterAimingCalcs.getDistanceFromHub(m_drivetrainSub.getPose()).getNorm()));
    m_shooterSub.setTargetFlywheelVelocity(ShooterAimingCalcs
        .getInterpolatedFlywheelVelocity(m_shooterAimingCalcs.getDistanceFromHub(m_drivetrainSub.getPose()).getNorm()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
