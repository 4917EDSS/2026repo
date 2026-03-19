// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSub;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class IntakeSetPositionCmd extends Command {
  private final double m_targetPositionDeg;
  private final double m_voltage;
  private final IntakeSub m_intakeSub;
  private double m_direction;

  /** Creates a new IntakeSetPositionCmd. */
  public IntakeSetPositionCmd(double targetPositionDeg, double voltage, IntakeSub intakeSub) {
    m_targetPositionDeg = targetPositionDeg;
    m_voltage = voltage;
    m_intakeSub = intakeSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_targetPositionDeg == Constants.Intake.kDeployInAngleDeg) {
      m_intakeSub.setBeltVoltage(0.0);
    } else {
      m_intakeSub.setBeltVoltage(Constants.Intake.kBeltTargetVoltage);
    }
    m_direction = 1.0;
    m_intakeSub.disableDeployAutomation();
    if(m_intakeSub.getDeployAngleDeg() >= m_targetPositionDeg) {
      m_direction = -1.0;
    }
    m_intakeSub.setDeployVoltage(m_direction * m_voltage);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSub.setDeployVoltage(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_intakeSub.getDeployAngleDeg() * m_direction >= m_targetPositionDeg * m_direction) {
      return true;
    } else {
      return false;
    }
  }
}
