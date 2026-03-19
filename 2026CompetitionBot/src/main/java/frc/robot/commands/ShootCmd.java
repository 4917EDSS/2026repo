// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSub;
import frc.robot.subsystems.IntakeSub;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class ShootCmd extends Command {
  HopperSub m_hopperSub;
  IntakeSub m_intakeSub;
  Boolean m_isAtPitchLimit;
  Boolean m_isAtYawLimit;

  /** Creates a new ShooterSubCmd. */
  public ShootCmd(HopperSub hopperSub, IntakeSub intakeSub) {
    m_hopperSub = hopperSub;
    m_intakeSub = intakeSub;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hopperSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hopperSub.setEscalatorTargetVelocityRps(Constants.Hopper.kEscalatorFeedSpeedRps);
  }

  public void execute() {
    if(m_intakeSub.inDeploySafetyZone()) {
      m_hopperSub.disableEscalatorAutomation();
      m_hopperSub.setSingulatorVoltage(0.0);
      return;
    }

    m_hopperSub.setEscalatorTargetVelocityRps(Constants.Hopper.kEscalatorFeedSpeedRps);
    if(Math.abs(m_hopperSub.getEscalatorVelocityRotPerSec()
        - Constants.Hopper.kEscalatorFeedSpeedRps) <= Constants.Hopper.kEscalatorVelocityToleranceRotPerSec) {
      m_hopperSub.setSingulatorVoltage(Constants.Hopper.kSingulatorFeedVoltage);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hopperSub.disableShooting();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

