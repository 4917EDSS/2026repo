// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class EnableShooterCmd extends Command {
  /** Creates a new EnableShooterCmd. */
  IntakeSub m_intakeSub;
  HopperSub m_hopperSub;
  ShooterSub m_shooterSub;

  public EnableShooterCmd(IntakeSub intakeSub, HopperSub hopperSub, ShooterSub shooterSub) {
    m_intakeSub = intakeSub;
    m_hopperSub = hopperSub;
    m_shooterSub = shooterSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeSub, m_hopperSub, m_shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSub.setTargetFlywheelVelocity(Constants.Shooter.kFlywheelTargetVelocityRotsPerSec);

    //wait for flywheel to reach target velocity
    while(!m_shooterSub.isAtTargetFlywheelVelocity()) {

    }
    m_hopperSub.setEscalatorTargetVelocityRps(Constants.Hopper.kEscalatorFeedSpeedRps);
    //wait for escalator to reach target velocity
    while(!m_hopperSub.isEscalatorAtTargetVelocity()) {

    }
    m_hopperSub.setSingulatorVoltage(Constants.Hopper.kSingulatorMaxVoltage);
    m_intakeSub.setBeltVoltage(Constants.Intake.kBeltTargetVoltage);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hopperSub.disableSingulatorAutomation();
    m_hopperSub.disableEscalatorAutomation();
    m_shooterSub.disableFlywheelAutomation();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
