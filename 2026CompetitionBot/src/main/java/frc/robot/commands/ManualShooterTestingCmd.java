// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSub;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class ManualShooterTestingCmd extends Command {
  ShooterSub m_shooterSub;
  double m_hoodAngle = 0.0;
  double m_flywheelRps = 0.0;
  double m_prevHoodAngle = 0.0;
  double m_prevFlywheelRps = 0.0;
  boolean m_kill = false;

  /** Creates a new ManualShooterTestingCmd. */
  public ManualShooterTestingCmd(ShooterSub shooterSub) {
    m_shooterSub = shooterSub;

    addRequirements(shooterSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("hood angle", m_hoodAngle);
    SmartDashboard.putNumber("flywheel rps", m_flywheelRps);
    SmartDashboard.putBoolean("kill", m_kill);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_hoodAngle = SmartDashboard.getNumber("hood angle", m_hoodAngle);
    m_flywheelRps = SmartDashboard.getNumber("hood angle", m_flywheelRps);
    m_kill = SmartDashboard.getBoolean("kill", m_kill);
    if(m_prevFlywheelRps != m_flywheelRps || m_prevHoodAngle != m_hoodAngle) {
      m_shooterSub.setTargetPitchAngle(SmartDashboard.getNumber("hood angle", m_hoodAngle));
      m_shooterSub.setTargetFlywheelVelocity(SmartDashboard.getNumber("flywheel rps", m_flywheelRps));
      m_prevFlywheelRps = m_flywheelRps;
      m_prevHoodAngle = m_hoodAngle;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_kill) {
      return true;
    }
    return false;
  }
}
