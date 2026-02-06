// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;

import com.ctre.phoenix6.configs.TalonFXConfigurator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;


import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;

// Neo 550(yaw)
// Neo 550(pitch)
// Kraken X60 (flywheels) x 2

// 2 limits swithces for yaw left and right
// 2 limit switches for yaw up and down
// 2 encoders one for pitch and one for yaw
// Interal encoder for flyer velocity
public class ShooterSub extends SubsystemBase {

  // private final DigitalInput m_shooterIsAtYawCWLimit = new DigitalInput(Constants.DioIds.kShooterIsAtYawCWLimit);
  // private final DigitalInput m_shooterIsAtYawCCWLimit = new DigitalInput(Constants.DioIds.kShooterIsAtYawCCWLimit);
  // private final DigitalInput m_shooterIsAtPitchLowerLimit =
  //     new DigitalInput(Constants.DioIds.kShooterIsAtPitchLowerLimit);
  // private final DigitalInput m_shooterIsAtPitchUpperLimit =
  //     new DigitalInput(Constants.DioIds.kShooterIsAtPitchUpperLimit);

  /** Creates a new ShooterSub. */

  private final SparkMax m_yawMotor = new SparkMax(Constants.CanIds.kYawMotor, MotorType.kBrushless);
  private final SparkMax m_pitchMotor = new SparkMax(Constants.CanIds.kPitchMotor, MotorType.kBrushless);

  private final TalonFX m_shooterMotor1 = new TalonFX(Constants.CanIds.kShooterMotor1);
  private final TalonFX m_shooterMotor2 = new TalonFX(Constants.CanIds.kShooterMotor2);
  private StatusSignal<AngularVelocity> m_shooterVelcocitySignal;

  // private final Encoder m_yawAbsoluteEncoder =
  //     new Encoder(Constants.DioIds.kShooterYawAbsoluteEncoder1, Constants.DioIds.kShooterYawAbsoluteEncoder2);

  // private final Encoder m_pitchAbsoluteEncoder =
  //     new Encoder(Constants.DioIds.kShooterPitchAbsoluteEncoder1, Constants.DioIds.kShooterPitchAbsoluteEncoder2);

  private double m_targetYawAngle = 0;
  private double m_targetPitchAngle = 0;
  private double m_targetFlywheelVelocity = 0;

  //Pid Controls
  private double m_pitchKp = 0.02;
  private double m_pitchKi = 0.0;
  private double m_kD = 0.0;

  private final PIDController m_pitchPidController = new PIDController(m_pitchKp, m_pitchKi, m_kD);


  public ShooterSub() {
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig
        .inverted(true) // Set to true to invert the forward motor direction
        .smartCurrentLimit(60) // Current limit in amps
        .idleMode(IdleMode.kBrake).encoder
            .positionConversionFactor(0.0)
            .velocityConversionFactor(0.0);

    AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();
    encoderConfig.zeroOffset(0.0);
    motorConfig.apply(encoderConfig);
    m_yawMotor.configure(motorConfig, com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
    m_pitchMotor.configure(motorConfig, com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);


    m_pitchMotor.configure(motorConfig, com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);


    TalonFXConfigurator talonFXConfigurator = m_shooterMotor1.getConfigurator();
    //This is how you set a current limit inside the motor (vs on the input power supply)
    //subject to change
    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 60; //limit in amps /TODO: determine reasonable limit
    limitConfigs.StatorCurrentLimitEnable = true;
    talonFXConfigurator.apply(limitConfigs);


    // This is how you can set a deadband, invert the motor rotoation and set brake/coast
    MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    outputConfigs.DutyCycleNeutralDeadband = 0.02; // Ignore values below 2%
    outputConfigs.Inverted = InvertedValue.Clockwise_Positive; // Invert = Clockwise
    outputConfigs.NeutralMode = NeutralModeValue.Brake;
    talonFXConfigurator.apply(outputConfigs);

    //setting up internal encoder for TalonFX
    m_shooterVelcocitySignal = m_shooterMotor1.getVelocity();
  }


  public boolean shooterIsAtYawCWLimit() {
    return m_yawMotor.getForwardLimitSwitch().isPressed();
  }


  public boolean shooterIsAtYawCCWLimit() {
    return m_yawMotor.getReverseLimitSwitch().isPressed();
  }


  public boolean shooterIsAtPitchLowerLimit() {
    return m_pitchMotor.getForwardLimitSwitch().isPressed();
  }


  public boolean shooterIsAtPitchUpperLimit() {
    return m_pitchMotor.getReverseLimitSwitch().isPressed();
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
    m_yawMotor.getEncoder().setPosition(0);
  }

  public double getYawEncoder() {
    return m_yawMotor.getEncoder().getPosition();
  }

  public void resetPitchEncoder() {
    m_pitchMotor.getEncoder().setPosition(0);
  }

  public double getPitchEncoder() {
    return m_pitchMotor.getEncoder().getPosition();
  }

  private void runYawControl(boolean updateYawPower) {
    double activeAngle = m_targetYawAngle;
  }

  private void runPitchControl(boolean updatePitchPower) {
    double activeAngle = m_targetPitchAngle;
  }

  public double getFlywheelVelocity() {
    //returning in RPM
    return m_shooterVelcocitySignal.getValueAsDouble() * 60;
  }

  public boolean isAtTargetVelocity() {
    return getFlywheelVelocity() == m_targetFlywheelVelocity;
  }

  public boolean isAtTargetPitchAngle() {
    //TO DO, need to convert encoders position to angle
    return getPitchAngle() == m_targetPitchAngle;
  }

  public boolean isAtTargetYawAngle() {
    //TO DO, need to convert encoders position to angle
    return getYawAngle() == m_targetYawAngle;
  }

  public double getYawAngle() {
    return getYawEncoder() * 360;
  }

  public double getPitchAngle() {
    return getPitchEncoder() * 360;
  }

  private void setPitchPower(Double power) {
    m_pitchMotor.set(power);
  }

  private void runPitchAngleControl() {
    Double currentAngle = getPitchAngle();


    double pidPower = m_pitchPidController.calculate(currentAngle, m_targetPitchAngle);
    //TO DO create constant for this
    if(Math.abs(pidPower) > Constants.Shooter.kPitchMaxPower) {
      double sign = (pidPower >= 0.0) ? 1.0 : -1.0;
      pidPower = Constants.Shooter.kPitchMaxPower * sign;
    }


    setPitchPower(pidPower);
  }


}
