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
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Power;
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
  /** Creates a new ShooterSub. */

 private final SparkMax m_yawMotor = new SparkMax(Constants.CanIds.kYawMotor, MotorType.kBrushless);
  private final SparkMax m_pitchMotor = new SparkMax(Constants.CanIds.kPitchMotor , MotorType.kBrushless);
  private final TalonFX m_shooterMotor = new TalonFX(Constants.CanIds.kShooterMotor);


  private final Encoder m_yawAbsoluteEncoder =
      new Encoder(Constants.DioIds.kShooterYawAbsoluteEncoder1, Constants.DioIds.kShooterYawAbsoluteEncoder2);

  private final Encoder m_pitchAbsoluteEncoder =
      new Encoder(Constants.DioIds.kShooterPitchAbsoluteEncoder1, Constants.DioIds.kShooterPitchAbsoluteEncoder2);

  private double m_targetYawAngle = 0;
  private double m_targetPitchAngle = 0;
  private double m_targetFlywheelVelocity = 0;


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
    m_yawMotor.configure(motorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
         m_pitchMotor.configure(motorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    
    TalonFXConfigurator talonFXConfigurator = m_shooterMotor.getConfigurator();
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

  private void runYawControl(boolean updateYawPower) {
    double activeAngle = m_targetYawAngle;
  }
  
 private void runPitchControl(boolean updatePitchPower) {
    double activeAngle = m_targetPitchAngle;
  }

}
