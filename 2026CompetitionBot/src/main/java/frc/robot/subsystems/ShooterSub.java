// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSub extends SubsystemBase {
  /** Creates a new ShooterSub. */


  private final TalonFX m_directionMotor = new TalonFX(Constants.CanIds.kDirectionMotor);
  private final TalonFX m_shooterMotor2 = new TalonFX(Constants.CanIds.kShooterMotor);
  private final TalonFX m_hoodMotor2 = new TalonFX(Constants.CanIds.kHoodMotor);

  private double m_targetYawAngle = 0;
  private double m_targetPitchAngle = 0;
  private double m_targetFlywheelVelocity = 0;


  public ShooterSub() {}

  public static double ballFlightTime(double distance) {
    //TODO do parabola math
    return distance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void setTargetYawAngle(double angle) {
    // Not doing anything yet
    m_targetYawAngle = angle;

  }

  public void setTargetPitchAngle(double angle) {
    // Not doing anything yet
    m_targetPitchAngle = angle;

  }

  public void setTargetFlywheelVelocity(double velocity) {
    // Not doing anything yet
    m_targetFlywheelVelocity = velocity;
  }
}
