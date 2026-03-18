// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSub;
import frc.robot.subsystems.ShooterSub;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class ManualShooterTestingCmd extends Command {
  ShooterSub m_shooterSub;
  HopperSub m_hopperSub;

  boolean m_kill = false;

  /** Creates a new ManualShooterTestingCmd. */
  public ManualShooterTestingCmd(ShooterSub shooterSub, HopperSub hopperSub) {
    m_shooterSub = shooterSub;
    m_hopperSub = hopperSub;

    addRequirements(shooterSub, hopperSub);
    SmartDashboard.putNumber("hood angle", 0.0);
    SmartDashboard.putNumber("turret angle", 0.0);
    SmartDashboard.putNumber("flywheel rps", 100.0);

    SmartDashboard.putNumber("escalator rps", 30.0);

    SmartDashboard.putBoolean("kill", m_kill);


    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSub.setTargetPitchAngle(SmartDashboard.getNumber("hood angle", 0.0));
    m_shooterSub.setTargetYawAngle(SmartDashboard.getNumber("turret angle", 0.0));
    //System.out.println(SmartDashboard.getNumber("turret angle", 0.0));
    m_shooterSub.setTargetFlywheelVelocity(SmartDashboard.getNumber("flywheel rps", 0.0));
    m_hopperSub.setEscalatorTargetVelocity(SmartDashboard.getNumber("escalator rps", 0.0));
    // m_hopperSub.setSingulatorPower(0.75);
    m_kill = SmartDashboard.getBoolean("kill", m_kill);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSub.disableFlywheelAutomation();
    m_shooterSub.disablePitchAutomation();
    m_shooterSub.disableYawAutomation();
    m_hopperSub.disableShooting();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_kill) {
      return true;
    }
    return false;
  }
}
