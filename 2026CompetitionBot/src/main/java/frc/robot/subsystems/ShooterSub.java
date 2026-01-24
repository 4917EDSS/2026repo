// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSub extends SubsystemBase {
  /** Creates a new ShooterSub. */


  private final TalonFX m_yawMotor = new TalonFX(Constants.CanIds.kYawMotor);
  private final TalonFX m_shooterMotor = new TalonFX(Constants.CanIds.kShooterMotor);
  private final TalonFX m_pitchMotor = new TalonFX(Constants.CanIds.kPitchMotor);

  private final Encoder m_yawAbsoluteEncoder =
      new Encoder(Constants.DioIds.kShooterYawAbsoluteEncoder1, Constants.DioIds.kShooterYawAbsoluteEncoder2);

  private final Encoder m_pitchAbsoluteEncoder =
      new Encoder(Constants.DioIds.kShooterPitchAbsoluteEncoder1, Constants.DioIds.kShooterPitchAbsoluteEncoder2);

  private double m_targetYawAngle = 0;
  private double m_targetPitchAngle = 0;
  private double m_targetFlywheelVelocity = 0;


  public ShooterSub() {

    TalonFXConfigurator talonFXConfigurator1 = m_yawMotor.getConfigurator();
    TalonFXConfigurator talonFXConfigurator2 = m_shooterMotor.getConfigurator();
    TalonFXConfigurator talonFXConfigurator3 = m_pitchMotor.getConfigurator();

    //This is how you set a current limit inside the motor (vs on the input power supply)
    //subject to change
    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 60; //limit in amps /TODO: determine reasonable limit
    limitConfigs.StatorCurrentLimitEnable = true;
    talonFXConfigurator1.apply(limitConfigs);
    talonFXConfigurator2.apply(limitConfigs);
    talonFXConfigurator3.apply(limitConfigs);

    // This is how you can set a deadband, invert the motor rotoation and set brake/coast
    MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    outputConfigs.DutyCycleNeutralDeadband = 0.02; // Ignore values below 2%
    outputConfigs.Inverted = InvertedValue.Clockwise_Positive; // Invert = Clockwise
    outputConfigs.NeutralMode = NeutralModeValue.Brake;
    talonFXConfigurator1.apply(outputConfigs);
    talonFXConfigurator2.apply(outputConfigs);
    talonFXConfigurator3.apply(outputConfigs);
  }

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

  public void resetYawEncoder() {
    m_yawAbsoluteEncoder.reset();
  }

  public void getYawEncoder() {
    m_yawAbsoluteEncoder.getDistance();
  }

  public void resetPitchEncoder() {
    m_pitchAbsoluteEncoder.reset();
  }

  public void getPitchEncoder() {
    m_pitchAbsoluteEncoder.getDistance();
  }
}
