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
public class IntakeToggleCmd extends Command {
  private final IntakeSub m_intakeSub;
  private Boolean m_deploy;

  /** Creates a new IntakeDeployCmd. */
  public IntakeToggleCmd(IntakeSub intakeSub) {
    m_intakeSub = intakeSub;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSub);
  }

  public IntakeToggleCmd(IntakeSub intakeSub, Boolean deploy) {
    this(intakeSub);
    m_deploy = deploy;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_deploy == null) {
      double targetAngleDeg = m_intakeSub.getTargetDeployAngleDeg();
      if(targetAngleDeg < Constants.Intake.kDeployInAngleDeg + 0.1
          && targetAngleDeg > Constants.Intake.kDeployInAngleDeg - 0.1) {
        m_deploy = true;
      } else {
        m_deploy = false;
      }
    }

    if(m_deploy) {
      // If our previous target angle was in, deploy the intake
      m_intakeSub.setTargetDeployAngle(Constants.Intake.kDeployOutAngleDeg);
      m_intakeSub.setBeltVoltage(Constants.Intake.kBeltVoltage);
    } else {
      // If our previous target angle was out, retract the intake
      m_intakeSub.setTargetDeployAngle(Constants.Intake.kDeployInAngleDeg);
      m_intakeSub.setBeltVoltage(0.0);
    }
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
