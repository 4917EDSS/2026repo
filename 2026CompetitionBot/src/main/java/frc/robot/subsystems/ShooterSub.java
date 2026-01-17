// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSub extends SubsystemBase {
  /** Creates a new ShooterSub. */
    
  
  private final TalonFX m_directionMotor = new TalonFX(Constants.CanIds.kDirectionMotor);
  private final TalonFX m_shooterMotor2 = new TalonFX(Constants.CanIds.kShooterMotor);
  private final TalonFX m_hoodMotor2 = new TalonFX(Constants.CanIds.kHoodMotor);

  private double m_targetAngle = 0;
  private double m_targetMotorspeed = 0;



  public ShooterSub() {}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
