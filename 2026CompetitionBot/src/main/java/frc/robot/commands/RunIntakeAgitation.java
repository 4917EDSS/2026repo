// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSub;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunIntakeAgitation extends InstantCommand {
  IntakeSub m_intakeSub;
  CommandXboxController m_driverController;
  boolean m_withintake;
  double m_direction;
  double counter;

  public RunIntakeAgitation(IntakeSub intakeSub, CommandXboxController driverController) {
    m_intakeSub = intakeSub;
    m_driverController = driverController;
    addRequirements(m_intakeSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
  }

  @Override
  public void execute() {
    if(m_intakeSub.getDeployAngleDeg() < Constants.Intake.kDeployShakeMin || (m_direction < 0 && counter >= 50)) {
      m_direction = 1.0;
      counter = 0;
    } else if(m_intakeSub.getDeployAngleDeg() > Constants.Intake.kDeployOutAngleDeg) {
      // Bring the intake in needs more power
      m_direction = -2.0;
      counter = 0;
    } else {
      counter++;
    }
    if(Math.abs(m_driverController.getLeftX()) < 0.05 && Math.abs(m_driverController.getLeftY()) < 0.05
        && Math.abs(m_driverController.getRightX()) < 0.05) {
      m_intakeSub.setDeployVoltage(m_direction * 0.0);
      m_intakeSub.setBeltVoltage(Constants.Intake.kBeltAgitationVoltage);
    } else if(m_intakeSub.getDeployAngleDeg() > Constants.Intake.kDeployNearlyMax) {
      m_intakeSub.setDeployVoltage(0.0);
      m_intakeSub.setBeltVoltage(Constants.Intake.kBeltIntakeVoltage);
    } else {
      m_intakeSub.setDeployVoltage(0.0);
      m_intakeSub.setBeltVoltage(Constants.Intake.kBeltIntakeVoltage);
    }

  }

  @Override
  public void end(boolean interrupted) {
    m_intakeSub.setBeltVoltage(0.0);
    m_intakeSub.setDeployVoltage(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
