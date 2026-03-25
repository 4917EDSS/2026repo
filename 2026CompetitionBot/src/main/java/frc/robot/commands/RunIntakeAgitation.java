// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSub;
import frc.robot.utils.RobotStatus;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunIntakeAgitation extends InstantCommand {
  IntakeSub m_intakeSub;
  boolean m_withintake;
  double m_direction;

  public RunIntakeAgitation(IntakeSub intakeSub) {
    m_intakeSub = intakeSub;
    addRequirements(m_intakeSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_withintake = RobotStatus.isIntakeAgitating();

    if(m_withintake) {
      m_intakeSub.setBeltVoltage(Constants.Intake.kBeltTargetVoltage);
      if(m_intakeSub.getDeployAngleDeg() < Constants.Intake.kDeployShakeMin) {
        m_direction = 1.0;
      } else if(m_intakeSub.getDeployAngleDeg() > Constants.Intake.kDeployOutAngleDeg) {
        m_direction = -1.0;
      }
      m_intakeSub.setDeployVoltage(m_direction * 1.5);
    } else {
      m_intakeSub.setBeltVoltage(0.0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeSub.setBeltVoltage(0.0);
    m_intakeSub.setDeployVoltage(0.0);
  }

  @Override
  public boolean isFinished() {
    return !RobotStatus.isIntakeAgitating();
  }
}
