// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.logging.Logger;
import frc.robot.subsystems.CanSub;
import frc.robot.subsystems.ClimbSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.HopperSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class KillAllCmd extends Command {
  private static Logger m_logger = Logger.getLogger(KillAllCmd.class.getName());
  ClimbSub m_climbSub;
  ShooterSub m_shooterSub;
  IntakeSub m_intakeSub;
  HopperSub m_hopperSub;

  /** Creates a new KillAllCmd. */
  public KillAllCmd(CanSub canSub, ClimbSub climbSub, DrivetrainSub drivetrainSub, HopperSub hopperSub,
      IntakeSub intakeSub, ShooterSub shooterSub) {

    m_climbSub = climbSub;
    m_shooterSub = shooterSub;
    m_intakeSub = intakeSub;
    m_hopperSub = hopperSub;

    addRequirements(canSub, climbSub, drivetrainSub, hopperSub, intakeSub, shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_logger.fine("KillAllCmd - Init");

    m_climbSub.setTargetRotateAngle(m_climbSub.getRotationAngleDeg());
    m_climbSub.setTargetDeployDistance(m_climbSub.getDeployDistanceM());
    m_intakeSub.setTargetDeployAngle(m_intakeSub.getDeployAngleDeg());
    m_shooterSub.setTargetPitchAngle(m_shooterSub.getPitchAngleDeg());
    m_shooterSub.setTargetYawAngle(m_shooterSub.getYawAngleDeg());
    m_hopperSub.setSingulatorPower(0.0);
    m_intakeSub.setBeltPower(0.0);


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_logger.fine("KillAllCmd - End" + (interrupted ? " (interrupted)" : ""));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
