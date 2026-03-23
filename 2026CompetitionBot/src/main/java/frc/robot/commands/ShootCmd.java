// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class ShootCmd extends Command {
  HopperSub m_hopperSub;
  IntakeSub m_intakeSub;
  ShooterSub m_shooterSub;
  Boolean m_isAtPitchLimit;
  Boolean m_isAtYawLimit;
  double m_direction;
  boolean m_withintake;

  public ShootCmd(HopperSub hopperSub, IntakeSub intakeSub, ShooterSub shooterSub) {
    this(hopperSub, intakeSub, shooterSub, true);
  }

  /** Creates a new ShooterSubCmd. */
  public ShootCmd(HopperSub hopperSub, IntakeSub intakeSub, ShooterSub shooterSub, boolean withintake) {

    m_hopperSub = hopperSub;
    m_intakeSub = intakeSub;
    m_shooterSub = shooterSub;
    m_withintake = withintake;
    // Use addRequirements() here to declare subsystem dependencies.
    if(withintake) {
      addRequirements(hopperSub, intakeSub);
    } else {
      addRequirements(hopperSub);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hopperSub.setEscalatorTargetVelocityRps(Constants.Hopper.kEscalatorFeedSpeedRps);
    if(m_withintake) {
      m_intakeSub.setBeltVoltage(Constants.Intake.kBeltTargetVoltage);
      m_direction = -1.0;
    }
  }

  public void execute() {
    if(m_intakeSub.inDeploySafetyZone()) {
      m_hopperSub.setSingulatorVoltage(0.0);
      return;
    }

    if(m_withintake) {
      if(m_intakeSub.getDeployAngleDeg() < Constants.Intake.kDeployShakeMin) {
        m_direction = 1.0;
      } else if(m_intakeSub.getDeployAngleDeg() > Constants.Intake.kDeployOutAngleDeg) {
        m_direction = -1.0;
      }
      m_intakeSub.setDeployVoltage(m_direction * 1.5);
    }
    // if(Math.abs(m_hopperSub.getEscalatorVelocityRotPerSec()
    //     - Constants.Hopper.kEscalatorFeedSpeedRps) <= Constants.Hopper.kEscalatorVelocityToleranceRotPerSec
    //     && !m_shooterSub.isInDeadZone()) {
    m_hopperSub.setSingulatorVoltage(Constants.Hopper.kSingulatorFeedVoltage);
    // } else {
    //   m_hopperSub.setSingulatorVoltage(0.0);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hopperSub.disableShooting();
    if(m_withintake) {
      m_intakeSub.setBeltVoltage(0.0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

