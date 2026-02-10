// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperSub extends SubsystemBase {
  private final CanSub m_canSub;

  //IMPORTANT: The term singulator refers to the mechanism in the hopper which aligns the balls. 
  //The escalator forces the balls into the shooter.

  //request objects for velocity PID control
  final VelocityVoltage m_singulatorVelocityRequest = new VelocityVoltage(0).withSlot(0);
  final VelocityVoltage m_escalatorVelocityRequest = new VelocityVoltage(0).withSlot(0);

  private final TalonFX m_singulatorMotor = new TalonFX(Constants.CanIds.kHopperSingulatorMotor);
  private final SparkMax m_escalatorMotor = new SparkMax(Constants.CanIds.kHopperEscalatorMotor, MotorType.kBrushless);

  private boolean m_singulatorEnabled = false;
  private boolean m_escalatorEnabled = false;
  private double m_targetSingulatorVelocity = 0.0;
  private double m_targetEscalatorVelocity = 0.0;

  /** Creates a new HopperSub. */
  public HopperSub(CanSub canSub) {
    m_canSub = canSub;
    //singulator configurating
    // configuration is untested
    TalonFXConfigurator talonFXSingulatorConfigurator = m_singulatorMotor.getConfigurator();

    // Set encoder conversion factor
    FeedbackConfigs singulatorFeedbackConfigs = new FeedbackConfigs();
    singulatorFeedbackConfigs.SensorToMechanismRatio = Constants.Hopper.kSingulatorTicksToDegrees;
    talonFXSingulatorConfigurator.apply(singulatorFeedbackConfigs);

    //current limits configurations for singulator
    CurrentLimitsConfigs limitSingulatorConfigs = new CurrentLimitsConfigs();
    limitSingulatorConfigs.StatorCurrentLimit = 100; //this might be too high idk
    limitSingulatorConfigs.StatorCurrentLimitEnable = true;
    talonFXSingulatorConfigurator.apply(limitSingulatorConfigs);

    // PID configurations for singulator
    Slot0Configs slot0SingulatorConfigs = new Slot0Configs();
    slot0SingulatorConfigs.kS = 0.1; // Add 0.1 V output to overcome static friction
    slot0SingulatorConfigs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0SingulatorConfigs.kP = 0.11; // An error of 1 rps results in 0.11 V output
    slot0SingulatorConfigs.kI = 0; // no output for integrated error
    slot0SingulatorConfigs.kD = 0; // no output for error derivative
    talonFXSingulatorConfigurator.apply(slot0SingulatorConfigs);

    //motor configurations for singulator
    MotorOutputConfigs outputSingulatorConfigs = new MotorOutputConfigs();
    outputSingulatorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    outputSingulatorConfigs.NeutralMode = NeutralModeValue.Coast;
    talonFXSingulatorConfigurator.apply(outputSingulatorConfigs);


    //Escalator configurating
    // Motor Configs need to be tested
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig
        .inverted(false) // Set to true to invert the forward motor direction
        .smartCurrentLimit(100) // Current limit in amps
        .idleMode(IdleMode.kBrake).encoder
            .positionConversionFactor(1.0)
            .velocityConversionFactor(1.0);

    // Save the configuration to the motor
    // Only persist parameters when configuring the motor on start up as this
    // operation can be slow
    m_escalatorMotor.configure(motorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Singulating", m_singulatorEnabled);
    SmartDashboard.putNumber("Singulator Velocity", m_singulatorMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putBoolean("Escalating", m_escalatorEnabled);
    SmartDashboard.putNumber("Escalator Velocity", m_escalatorMotor.get());

    if(m_singulatorEnabled) {
      setSingulatorVelocity(Constants.Hopper.kSingulatorVelocity);
    }

    // TODO: For Spark max, need to run algorithm here
    if(m_escalatorEnabled) {
      setEscalatorVelocity(Constants.Hopper.kEscalatorVelocity);
    }
  }

  // TODO potentienly activate singulator and escalator at the same time
  public void enableSingulator() {
    m_singulatorEnabled = true;
  }

  public void disableSingulator() {
    m_singulatorEnabled = false;
  }

  public void setSingulatorPower(double power) {
    // Disable velocity control to stop interferring 
    disableSingulator();
    m_singulatorMotor.set(power);
  }

  //set singulator velocity with PID in RPS
  public void setSingulatorVelocity(double velocity) {
    m_targetSingulatorVelocity = velocity;
    //use pid control to set velocity
    m_singulatorMotor.setControl(m_singulatorVelocityRequest.withVelocity(velocity).withFeedForward(0.0));
  }

  public boolean isSingulatorEnabled() {
    return m_singulatorEnabled;
  }

  public double getSingulatorVelocity() {
    return m_singulatorMotor.getVelocity().getValueAsDouble();
  }

  public boolean isSingulatorAtTargetVelocity() {
    if(Constants.Hopper.kSingulatorVelocityTolerance > Math.abs(m_targetSingulatorVelocity - getSingulatorVelocity())) {
      return true;
    }
    return false;
  }


  public void enableEscalator() {
    m_escalatorEnabled = true;
  }

  public void disableEscalator() {
    m_escalatorEnabled = false;
  }

  public boolean isEscalatorEnabled() {
    return m_escalatorEnabled;
  }

  public void setEscalatorPower(double power) {
    // disable velocity control to stop interferring 
    disableEscalator();
    m_escalatorMotor.set(power);
  }

  //set Escalator velocity with PID in RPS
  public void setEscalatorVelocity(double velocity) {
    m_targetEscalatorVelocity = velocity;
    //use pid control to set velocity
    m_escalatorMotor.set(velocity); // TODO: Needs to be an algorithm that runs from Periodic
  }

  public double getEscalatorVelocity() {
    return m_escalatorMotor.get(); // TODO: Get velocity from internal encoder
  }


  public boolean isEscalatorAtTargetVelocity() {
    if(Constants.Hopper.kEscalatorVelocityTolerance > Math.abs(m_targetEscalatorVelocity - getEscalatorVelocity())) {
      return true;
    }
    return false;
  }

  public boolean isFull() {
    return m_canSub.isHopperFull(); // need sensor from cansub
  }

  public boolean isFuelInEscalator() {
    return m_canSub.isFuelInEscalator();
  }

  public boolean isEmpty() {
    return m_canSub.isHopperEmpty(); // need sensor from cansub
  }

}
