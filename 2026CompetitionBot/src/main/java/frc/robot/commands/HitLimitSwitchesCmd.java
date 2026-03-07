// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class ZeroAllSubsystemsCmd extends Command {
  IntakeSub m_intakeSub;
  ShooterSub m_shooterSub;
  Boolean m_isAtIntakeLimit = false;
  Boolean m_isAtPitchLimit = false;
  Boolean m_isAtYawLimit = false;

  public ZeroAllSubsystemsCmd(IntakeSub intakeSub, ShooterSub shooterSub) {
    m_intakeSub = intakeSub;
    m_shooterSub = shooterSub;


    addRequirements(intakeSub, shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // TODO: MAKE SURE THESE ALL GO IN THE CORRECT DIRECTION
    m_intakeSub.setDeployPower(0.1);

    m_shooterSub.setYawPower(-0.1);
    m_shooterSub.setPitchPower(-0.1);


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: PROBABLY ONLY CHECK ONE LIMIT, THE CORRECT ONE

    if(m_intakeSub.isAtInLimit()) {
      m_intakeSub.setDeployPower(0.0);
      m_isAtIntakeLimit = true;
    }

    if(m_shooterSub.isAtPitchLowerLimit() || m_shooterSub.isAtPitchUpperLimit()) {
      m_shooterSub.setPitchPower(0.0);
      m_isAtPitchLimit = true;
    }
    if(m_shooterSub.isAtYawAtCCWLimit() || m_shooterSub.isAtYawAtCWLimit()) {
      m_shooterSub.setPitchPower(0.0);
      m_isAtYawLimit = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_isAtIntakeLimit && m_isAtPitchLimit && m_isAtYawLimit);
  }
}
