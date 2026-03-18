// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class DisableShooterCmd extends Command {

  IntakeSub m_intakeSub;
  HopperSub m_hopperSub;
  ShooterSub m_shooterSub;

  /** Creates a new DisableShooterCmd. */
  public DisableShooterCmd(IntakeSub intakeSub, HopperSub hopperSub, ShooterSub shooterSub) {
    m_intakeSub = intakeSub;
    m_hopperSub = hopperSub;
    m_shooterSub = shooterSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeSub, m_hopperSub, m_shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSub.setBeltVoltage(0.0);
    m_hopperSub.disableSingulatorAutomation();
    m_shooterSub.disableFlywheelAutomation();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
