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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperSub extends SubsystemBase {

  //IMPORTANT: The term singulator refer to the mechanism in the hopper which forces balls into the shooter.
  //kraken x60 motor for singulator rotation (done)
  //2 ir sensors for hopper full and hopper empty (1 each)
  //internal encoder for singulator velocity


  //request objects for velocity PID control
  final VelocityVoltage m_singulatorVelocityRequest = new VelocityVoltage(0).withSlot(0);
  final VelocityVoltage m_escalatorVelocityRequest = new VelocityVoltage(0).withSlot(0);

  private final TalonFX m_singulatorMotor = new TalonFX(Constants.CanIds.kSingulatorMotor);
  private final TalonFX m_escalatorMotor = new TalonFX(Constants.CanIds.kEscalatorMotor);

  private Boolean singulatorEnabled = false;
  private boolean escalatorEnabled = false;
  private double targetSingulatorVelocity = 0.0;
  private double targetEscalatorVelocity = 0.0;
  CanSub m_canSub;

  /** Creates a new HopperSub. */
  public HopperSub(CanSub canSub) {
    m_canSub = canSub;
    //singulator configurating
    TalonFXConfigurator talonFXSingulatorConfigurator = m_singulatorMotor.getConfigurator();

    FeedbackConfigs singulatorFeedbackConfigs = new FeedbackConfigs();
    singulatorFeedbackConfigs.SensorToMechanismRatio = Constants.HopperConstants.kSingulatorTicksInMeter;
    talonFXSingulatorConfigurator.apply(singulatorFeedbackConfigs);

    //current limits configurations for singulator
    CurrentLimitsConfigs limitSingulatorConfigs = new CurrentLimitsConfigs();
    limitSingulatorConfigs.StatorCurrentLimit = 60; //this might be too high idk
    limitSingulatorConfigs.StatorCurrentLimitEnable = true;
    talonFXSingulatorConfigurator.apply(limitSingulatorConfigs);

    // PID configurations for singulator
    var slot0SingulatorConfigs = new Slot0Configs();
    slot0SingulatorConfigs.kS = 0.1; // Add 0.1 V output to overcome static friction
    slot0SingulatorConfigs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0SingulatorConfigs.kP = 0.11; // An error of 1 rps results in 0.11 V output
    slot0SingulatorConfigs.kI = 0; // no output for integrated error
    slot0SingulatorConfigs.kD = 0; // no output for error derivative
    talonFXSingulatorConfigurator.apply(slot0SingulatorConfigs);

    //motor configurations for singulator
    MotorOutputConfigs outputSingulatorConfigs = new MotorOutputConfigs();
    outputSingulatorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    outputSingulatorConfigs.NeutralMode = NeutralModeValue.Brake;
    talonFXSingulatorConfigurator.apply(outputSingulatorConfigs);


    //Escalator configurating
    TalonFXConfigurator talonFXEscalatorConfigurator = m_escalatorMotor.getConfigurator();

    FeedbackConfigs escalatorFeedbackConfigs = new FeedbackConfigs();
    escalatorFeedbackConfigs.SensorToMechanismRatio = Constants.HopperConstants.kEscalatorTicksInMeter;
    talonFXEscalatorConfigurator.apply(escalatorFeedbackConfigs);

    //current limit configurations for Escalator
    CurrentLimitsConfigs limitEscalatorConfigs = new CurrentLimitsConfigs();
    limitEscalatorConfigs.StatorCurrentLimit = 60; //this might be too high idk
    limitEscalatorConfigs.StatorCurrentLimitEnable = true;
    talonFXEscalatorConfigurator.apply(limitEscalatorConfigs);

    // PID configurations for Escalator
    var slot0EscalatorConfigs = new Slot0Configs();
    slot0EscalatorConfigs.kS = 0.1; // Add 0.1 V output to overcome static friction
    slot0EscalatorConfigs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0EscalatorConfigs.kP = 0.11; // An error of 1 rps results in 0.11 V output
    slot0EscalatorConfigs.kI = 0; // no output for integrated error
    slot0EscalatorConfigs.kD = 0; // no output for error derivative
    talonFXEscalatorConfigurator.apply(slot0EscalatorConfigs);

    //motor configurations for Escalator
    MotorOutputConfigs outputEscalatorConfigs = new MotorOutputConfigs();
    outputEscalatorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    outputEscalatorConfigs.NeutralMode = NeutralModeValue.Brake;
    talonFXEscalatorConfigurator.apply(outputEscalatorConfigs);
  }

  // TODO potentienly activate singulator and escalator at the same time
  public void enableSingulator() {
    singulatorEnabled = true;
  }

  public void disableSingulator() {
    singulatorEnabled = false;
  }

  public void setSingulatorPower(double power) {
    // Disable velocity control to stop interferring 
    disableSingulator();
    m_singulatorMotor.set(power);
  }

  //set singulator velocity with PID in RPS
  public void setSingulatorVelocity(double velocity) {
    targetSingulatorVelocity = velocity;
    //use pid control to set velocity
    m_singulatorMotor.setControl(m_singulatorVelocityRequest.withVelocity(velocity).withFeedForward(0.0));
  }

  public boolean isSingulatorEnabled() {
    return singulatorEnabled;
  }

  public double getSingulatorVelocity() {
    return m_singulatorMotor.getVelocity().getValueAsDouble();
  }

  public boolean isSingulatorAtTargetVelocity() {
    if(Constants.HopperConstants.kSingulatorVelocityTolerance > Math
        .abs(targetSingulatorVelocity - getSingulatorVelocity())) {
      return true;
    }
    return false;
  }


  public void enableEscalator() {
    escalatorEnabled = true;
  }

  public void disableEscalator() {
    escalatorEnabled = false;
  }

  public boolean isEscalatorEnabled() {
    return escalatorEnabled;
  }

  public void setEscalatorPower(double power) {
    // disable velocity control to stop interferring 
    disableEscalator();
    m_escalatorMotor.set(power);
  }

  //set Escalator velocity with PID in RPS
  public void setEscalatorVelocity(double velocity) {
    targetEscalatorVelocity = velocity;
    //use pid control to set velocity
    m_escalatorMotor.setControl(m_escalatorVelocityRequest.withVelocity(velocity).withFeedForward(0.0));
  }

  public double getEscalatorVelocity() {
    return m_escalatorMotor.getVelocity().getValueAsDouble();
  }


  public boolean isEscalatorAtTargetVelocity() {
    if(Constants.HopperConstants.kEscalatorVelocityTolerance > Math
        .abs(targetEscalatorVelocity - getEscalatorVelocity())) {
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Singulating", singulatorEnabled);
    SmartDashboard.putNumber("Singulator Velocity", m_escalatorMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putBoolean("Escalating", escalatorEnabled);
    SmartDashboard.putNumber("Escalator Velocity", m_escalatorMotor.getVelocity().getValueAsDouble());

    if(singulatorEnabled) {
      setSingulatorVelocity(Constants.HopperConstants.kSingulatorVelocity);
    }

    if(escalatorEnabled) {
      setEscalatorVelocity(Constants.HopperConstants.kEscalatorVelocity);
    }
  }
}
