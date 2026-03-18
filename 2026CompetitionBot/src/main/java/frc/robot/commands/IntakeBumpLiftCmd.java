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
public class IntakeBumpLiftCmd extends Command {
  IntakeSub m_intakeSub;

  /** Creates a new IntakeBumpLiftCmd. */
  public IntakeBumpLiftCmd(IntakeSub intakeSub) {
    m_intakeSub = intakeSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSub.setTargetDeployAngle(Constants.Intake.kDeployBumpAngleDeg);
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
