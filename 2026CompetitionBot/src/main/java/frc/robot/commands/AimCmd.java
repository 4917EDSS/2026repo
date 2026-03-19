// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.utils.ShooterAimingCalcs;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class AimCmd extends Command {
  private DrivetrainSub m_drivetrainSub;
  private ShooterSub m_shooterSub;
  private IntakeSub m_intakeSub;
  private final ShooterAimingCalcs m_shooterAimingCalcs;

  /** Creates a new AimCmd. */
  public AimCmd(ShooterSub shooterSub, ShooterAimingCalcs shooterAimingCalcs, DrivetrainSub drivetrainSub) {
    m_drivetrainSub = drivetrainSub;
    m_shooterSub = shooterSub;
    m_shooterAimingCalcs = shooterAimingCalcs;
    addRequirements(m_shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    if(Math.abs(
        m_intakeSub.getDeployAngleDeg()
            - Constants.Intake.kDeployInAngleDeg) < Constants.Intake.kDeployInToleranceDeg) {
      m_shooterSub.disableFlywheelAutomation();
      m_shooterSub.disablePitchAutomation();
      m_shooterSub.disableYawAutomation();
    } else {
      m_shooterSub.setPitchYawFlywheelTarget(
          m_shooterAimingCalcs.setTargets(m_drivetrainSub.getTurretPose(), m_drivetrainSub.getRobotRelativeSpeeds()));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
