// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
  final VelocityVoltage m_feederVelocityRequest = new VelocityVoltage(0).withSlot(0);

  private final TalonFX m_singulatorMotor = new TalonFX(Constants.CanIds.kSingulatorMotor);
  private final TalonFX m_feederMotor = new TalonFX(Constants.CanIds.kFeederMotor);

  private Boolean singulatorEnabled = false;
  private boolean feederEnabled = false;
  private double targetSingulatorVelocity = 0.0;
  private double targetFeederVelocity = 0.0;

  /** Creates a new HopperSub. */
  public HopperSub() {
    //singulator configurating
    TalonFXConfigurator talonFXSingulatorConfigurator = m_singulatorMotor.getConfigurator();

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


    //feeder configurating
    TalonFXConfigurator talonFXFeederConfigurator = m_feederMotor.getConfigurator();

    //current limit configurations for feeder
    CurrentLimitsConfigs limitFeederConfigs = new CurrentLimitsConfigs();
    limitFeederConfigs.StatorCurrentLimit = 60; //this might be too high idk
    limitFeederConfigs.StatorCurrentLimitEnable = true;
    talonFXFeederConfigurator.apply(limitFeederConfigs);

    // PID configurations for feeder
    var slot0FeederConfigs = new Slot0Configs();
    slot0FeederConfigs.kS = 0.1; // Add 0.1 V output to overcome static friction
    slot0FeederConfigs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0FeederConfigs.kP = 0.11; // An error of 1 rps results in 0.11 V output
    slot0FeederConfigs.kI = 0; // no output for integrated error
    slot0FeederConfigs.kD = 0; // no output for error derivative
    talonFXSingulatorConfigurator.apply(slot0FeederConfigs);

    //motor configurations for feeder
    MotorOutputConfigs outputFeederConfigs = new MotorOutputConfigs();
    outputFeederConfigs.Inverted = InvertedValue.Clockwise_Positive;
    outputFeederConfigs.NeutralMode = NeutralModeValue.Brake;
    talonFXFeederConfigurator.apply(outputFeederConfigs);
  }

  public void enableSingulator() {
    singulatorEnabled = true;
  }

  public void disableSingulator() {
    singulatorEnabled = false;
  }

  //set singulator velocity with PID in RPS
  public void setSingulatorVelocity(double velocity) {
    targetSingulatorVelocity = velocity;
    //use pid control to set velocity
    m_singulatorMotor.setControl(m_singulatorVelocityRequest.withVelocity(velocity).withFeedForward(0.0));
  }

  public double getSingulatorVelocity() {
    return m_singulatorMotor.getVelocity().getValueAsDouble();
  }

  public double getSingulatorPosition() {
    return m_singulatorMotor.getPosition().getValueAsDouble() / Constants.HopperConstants.kSingulatorTicksInMeter;
  }

  public boolean isSingulatorAtTargetVelocity() {
    if(Constants.HopperConstants.kSingulatorVelocityTolerance > Math
        .abs(targetSingulatorVelocity - getSingulatorVelocity())) {
      return true;
    }
    return false;
  }


  public void enableFeeder() {
    feederEnabled = true;
  }

  public void disableFeeder() {
    feederEnabled = false;
  }

  //set feeder velocity with PID in RPS
  public void setFeederVelocity(double velocity) {
    targetFeederVelocity = velocity;
    //use pid control to set velocity
    m_singulatorMotor.setControl(m_singulatorVelocityRequest.withVelocity(velocity).withFeedForward(0.0));
  }

  public double getFeederPosition() {
    return m_feederMotor.getPosition().getValueAsDouble() / Constants.HopperConstants.kFeederTicksInMeter;
  }

  public double getFeederVelocity() {
    return m_singulatorMotor.getVelocity().getValueAsDouble();
  }


  public boolean isFeederAtTargetVelocity() {
    if(Constants.HopperConstants.kFeederVelocityTolerance > Math.abs(targetFeederVelocity - getFeederVelocity())) {
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Singulating", singulatorEnabled);
    SmartDashboard.putNumber("Singulator Velocity", m_singulatorMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putBoolean("feeding", feederEnabled);
    SmartDashboard.putNumber("Feeder Velocity", m_feederMotor.getVelocity().getValueAsDouble());

    if(singulatorEnabled) {
      setSingulatorVelocity(Constants.HopperConstants.kSingulatorVelocity);
    }

    if(feederEnabled) {
      setFeederVelocity(Constants.HopperConstants.kFeederVelocity);
    }
  }
}
