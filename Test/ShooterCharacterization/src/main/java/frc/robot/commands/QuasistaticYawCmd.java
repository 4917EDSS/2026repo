// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSub;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class QuasistaticYawCmd extends Command {
  private final boolean m_directionForward;
  private final double m_increaseInterval;
  private final double m_limit;
  private final ShooterSub m_shooterSub;

  private long m_lastUpdateTime;
  private double m_currentVoltage;

  /** Creates a new QuasistaticCmd. */
  public QuasistaticYawCmd(boolean directionForward, double increaseIntervalSec, double limit,
      ShooterSub shooterSub) {
    m_directionForward = directionForward;
    m_increaseInterval = increaseIntervalSec;
    m_limit = limit;
    m_shooterSub = shooterSub;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_directionForward) {
      m_currentVoltage = 1.0;
    } else {
      m_currentVoltage = -1.0;
    }
    m_shooterSub.setYawVoltage(m_currentVoltage);
    m_lastUpdateTime = RobotController.getFPGATime();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Check if the interval time has passed
    // Note that we use microseconds but the interval is passed in as seconds
    if((RobotController.getFPGATime() - m_lastUpdateTime) > (m_increaseInterval * 1000000)) {
      m_lastUpdateTime = RobotController.getFPGATime();
      if(m_directionForward) {
        m_currentVoltage += 1.0;
      } else {
        m_currentVoltage -= 1.0;
      }
      m_shooterSub.setYawVoltage(m_currentVoltage);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSub.setYawVoltage(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((m_directionForward && (m_shooterSub.getYawAngleDeg() > m_limit)) ||
        (!m_directionForward && (m_shooterSub.getYawAngleDeg() < m_limit))) {
      return true;
    }
    return false;
  }
}
