// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperSub extends SubsystemBase {
  private static Logger m_logger = Logger.getLogger(ClimbSub.class.getName());

  // IMPORTANT: The term singulator refers to the mechanism in the hopper which aligns the balls. 
  // The escalator forces the balls into the shooter.
  private final TalonFX m_singulatorMotor = new TalonFX(Constants.CanIds.kHopperSingulatorMotor);
  // TODO: Change this to a kraken
  private final TalonFX m_escalatorMotor = new TalonFX(Constants.CanIds.kHopperEscalatorMotor);

  private final PIDController m_escalatorPid =
      new PIDController(Constants.Hopper.kEscalatorKP, Constants.Hopper.kEscalatorKI, Constants.Hopper.kEscalatorKD);
  private final SimpleMotorFeedforward m_escalatorFeedforward =
      new SimpleMotorFeedforward(Constants.Hopper.kEscalatorKS, Constants.Hopper.kEscalatorKV);

  private final CanSub m_canSub;

  // Request objects for velocity PID control
  private final VelocityVoltage m_singulatorVelocityRequest = new VelocityVoltage(0).withSlot(0);

  private boolean m_singulatorAutomationEnabled = false;
  private boolean m_escalatorAutomationEnabled = false;
  private double m_targetSingulatorVelocityRps = 0.0;
  private double m_targetEscalatorVelocityRps = 0.0;

  /** Creates a new HopperSub. */
  public HopperSub(CanSub canSub) {
    m_canSub = canSub;

    TalonFXConfigurator talonFxSingulatorConfigurator = m_singulatorMotor.getConfigurator();

    // Singulator configuration
    // Set encoder conversion factor
    FeedbackConfigs singulatorFeedbackConfigs = new FeedbackConfigs();
    singulatorFeedbackConfigs.SensorToMechanismRatio = Constants.Hopper.kSingulatorEncoderToRpsConversionFactor;
    talonFxSingulatorConfigurator.apply(singulatorFeedbackConfigs);

    // Current limits configurations for singulator
    CurrentLimitsConfigs limitSingulatorConfigs = new CurrentLimitsConfigs();
    limitSingulatorConfigs.StatorCurrentLimit = Constants.Hopper.kSingulatorMaxCurrent;
    limitSingulatorConfigs.StatorCurrentLimitEnable = true;
    talonFxSingulatorConfigurator.apply(limitSingulatorConfigs);

    // PID configurations for singulator
    Slot0Configs slot0SingulatorConfigs = new Slot0Configs();
    slot0SingulatorConfigs.kS = Constants.Hopper.kSingulatorKS; // Add voltage to overcome static friction
    slot0SingulatorConfigs.kV = Constants.Hopper.kSingulatorKV; // A velocity target of 1 rps results in X volts
    slot0SingulatorConfigs.kP = Constants.Hopper.kSingulatorKP;
    slot0SingulatorConfigs.kI = Constants.Hopper.kSingulatorKI;
    slot0SingulatorConfigs.kD = Constants.Hopper.kSingulatorKD;
    talonFxSingulatorConfigurator.apply(slot0SingulatorConfigs);

    // Motor configurations for singulator
    MotorOutputConfigs outputSingulatorConfigs = new MotorOutputConfigs();
    outputSingulatorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    outputSingulatorConfigs.NeutralMode = NeutralModeValue.Coast;
    talonFxSingulatorConfigurator.apply(outputSingulatorConfigs);

    // Escalator configuration
    // Motor configs

    TalonFXConfigurator talonFxEscalatorConfigurator = m_escalatorMotor.getConfigurator();

    CurrentLimitsConfigs limitEscalatorConfigs = new CurrentLimitsConfigs();
    limitEscalatorConfigs.StatorCurrentLimit = Constants.Hopper.kEscalatorMaxCurrent;
    limitEscalatorConfigs.StatorCurrentLimitEnable = true;
    talonFxEscalatorConfigurator.apply(limitEscalatorConfigs);

    Slot0Configs slot0EscalatorConfigs = new Slot0Configs();
    slot0EscalatorConfigs.kS = Constants.Hopper.kEscalatorKS;
    slot0EscalatorConfigs.kV = Constants.Hopper.kEscalatorKV;
    slot0EscalatorConfigs.kP = Constants.Hopper.kEscalatorKP;
    slot0EscalatorConfigs.kI = Constants.Hopper.kEscalatorKI;
    slot0EscalatorConfigs.kD = Constants.Hopper.kEscalatorKD;
    talonFxEscalatorConfigurator.apply(slot0EscalatorConfigs);

    MotorOutputConfigs outputEscalatorConfigs = new MotorOutputConfigs();
    outputEscalatorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    outputEscalatorConfigs.NeutralMode = NeutralModeValue.Brake;
    talonFxEscalatorConfigurator.apply(outputEscalatorConfigs);
  }

  public void init() {
    m_logger.info("Initializing HopperSub Subsystem");
    disableEscalatorAutomation();
    disableSingulatorAutomation();
    setEscalatorPower(0.0);
    setSingulatorPower(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Singulator Auto", m_singulatorAutomationEnabled);
    SmartDashboard.putNumber("Singulator Target", m_targetSingulatorVelocityRps);
    SmartDashboard.putNumber("Singulator Velocity", getSingulatorVelocityRps());
    SmartDashboard.putNumber("Singulator Power", m_singulatorMotor.get());
    SmartDashboard.putBoolean("Escalator Auto", m_escalatorAutomationEnabled);
    SmartDashboard.putNumber("Escalator Target", m_targetEscalatorVelocityRps);
    SmartDashboard.putNumber("Escalator Velocity", getEscalatorVelocityRps());
    SmartDashboard.putNumber("Escalator Power", m_escalatorMotor.get());

    runEscalatorVelocityControl(m_escalatorAutomationEnabled);
  }

  public void setSingulatorPower(double power) {
    // Disable TalonFX velocity control to stop interferring 
    disableSingulatorAutomation();
    m_singulatorMotor.set(power);
  }

  public void setEscalatorPower(double power) {
    m_escalatorMotor.set(power);
  }

  public double getSingulatorVelocityRps() {
    return m_singulatorMotor.getVelocity().getValueAsDouble();
  }

  public double getEscalatorPosition() {
    return m_escalatorMotor.getPosition().getValueAsDouble();
  }

  public double getEscalatorVelocityRps() {
    return m_escalatorMotor.getVelocity().getValueAsDouble();
  }

  public boolean isFull() {
    return m_canSub.isHopperFull();
  }

  public boolean isEmpty() {
    return m_canSub.isHopperEmpty();
  }

  public boolean isFuelInEscalator() {
    return m_canSub.isFuelInEscalator();
  }

  ////////////////////////////// Singulator automation //////////////////////////////
  public void enableSingulatorAutomation() {
    m_singulatorAutomationEnabled = true;
    // Use TalonFX's PID control to set velocity
    m_singulatorMotor.setControl(m_singulatorVelocityRequest.withVelocity(m_targetSingulatorVelocityRps)
        .withFeedForward(Constants.Hopper.kSingulatorKV));
  }

  public void disableSingulatorAutomation() {
    m_singulatorAutomationEnabled = false;
    m_singulatorMotor.setControl(new DutyCycleOut(0.0)); // Disable velocity control
  }

  // Set Singulator velocity with PID in RPS
  public void setSingulatorTargetVelocity(double velocityRps) {
    m_targetSingulatorVelocityRps = velocityRps;
    enableSingulatorAutomation();
  }

  public boolean isSingulatorAtTargetVelocity() {
    if(Math.abs(m_targetSingulatorVelocityRps
        - getSingulatorVelocityRps()) < Constants.Hopper.kSingulatorVelocityToleranceRps) {
      return true;
    }
    return false;
  }

  ////////////////////////////// Escalator automation //////////////////////////////
  public void enableEscalatorAutomation() {
    m_escalatorAutomationEnabled = true;
    runEscalatorVelocityControl(true);
  }

  public void disableEscalatorAutomation() {
    m_escalatorAutomationEnabled = false;
    setEscalatorPower(0.0);
  }

  // Set Escalator velocity with PID in RPS
  public void setEscalatorVelocity(double velocityRps) {
    m_targetEscalatorVelocityRps = velocityRps;
    // TODO: Use velocity control on TalonFX
    // Use pid control to set velocity
  }

  public boolean isEscalatorAtTargetVelocity() {
    if(Math
        .abs(m_targetEscalatorVelocityRps
            - getEscalatorVelocityRps()) < Constants.Hopper.kEscalatorVelocityToleranceRps) {
      return true;
    }
    return false;
  }

  private void runEscalatorVelocityControl(boolean setPower) {
    // TODO: Tune velocity multipliers 
    // What are velocity multipliers?  And they should go into Constants if needed.  [Eric]
    double feedForwardVelocity = m_escalatorFeedforward.calculate(Constants.Hopper.kEscalatorMaxVelocityRps * 0.1);
    double pidVelocity =
        m_escalatorPid.calculate(getEscalatorVelocityRps(), Constants.Hopper.kEscalatorMaxVelocityRps * 0.1);

    setEscalatorVelocity(feedForwardVelocity + pidVelocity);
  }
}
