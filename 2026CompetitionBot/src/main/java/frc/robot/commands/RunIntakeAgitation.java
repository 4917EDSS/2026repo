// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.IntakeSub;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunIntakeAgitation extends InstantCommand {
  IntakeSub m_intakeSub;
  DrivetrainSub m_drivetrainSub;
  boolean m_withintake;
  double m_direction;

  public RunIntakeAgitation(IntakeSub intakeSub, DrivetrainSub drivetrainSub) {
    m_intakeSub = intakeSub;
    m_drivetrainSub = drivetrainSub;
    addRequirements(m_intakeSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSub.setBeltVoltage(Constants.Intake.kBeltTargetVoltage);
  }

  @Override
  public void execute() {
    if(m_intakeSub.getDeployAngleDeg() < Constants.Intake.kDeployShakeMin) {
      m_direction = 1.0;
    } else if(m_intakeSub.getDeployAngleDeg() > Constants.Intake.kDeployOutAngleDeg) {
      m_direction = -1.0;
    }
    if(Math.sqrt(Math.pow(m_drivetrainSub.getRobotRelativeSpeeds().vxMetersPerSecond, 2)
        + Math.pow(m_drivetrainSub.getRobotRelativeSpeeds().vyMetersPerSecond, 2)) < 0.25) {
      m_intakeSub.setDeployVoltage(m_direction * 1.5);
    } else if(m_intakeSub.getDeployAngleDeg() > Constants.Intake.kDeployOutAngleDeg) {
      m_intakeSub.setDeployVoltage(1.5);
    } else {
      m_intakeSub.setDeployVoltage(0.25);
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
