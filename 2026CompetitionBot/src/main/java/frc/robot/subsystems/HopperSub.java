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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class HopperSub extends SubsystemBase {
  private static Logger m_logger = Logger.getLogger(ClimbSub.class.getName());

  // IMPORTANT: The term singulator refers to the mechanism in the hopper which aligns the balls. 
  // The escalator forces the balls into the shooter.
  private final TalonFX m_singulatorMotor = new TalonFX(Constants.CanIds.kHopperSingulatorMotor);
  private final TalonFX m_escalatorMotor = new TalonFX(Constants.CanIds.kHopperEscalatorMotor);

  private final SysIdRoutine m_singulatorSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Units.Volts.per(Units.Second).of(0.5),
          Units.Volts.of(2.0),
          Units.Seconds.of(8.0)),
      new SysIdRoutine.Mechanism(
          (voltage) -> runSingulatorSysIdVolts(voltage.in(Units.Volts)),
          (SysIdRoutineLog log) -> {
            log.motor("hopperSingulator")
                .voltage(Units.Volts.of(m_singulatorMotor.get() * RobotController.getBatteryVoltage()))
                .angularPosition(Units.Rotations.of(getSingulatorRot()))
                .angularVelocity(Units.RotationsPerSecond.of(getSingulatorVelocityRotPerSec()));
          },
          this));

  private final SysIdRoutine m_escalatorSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Units.Volts.per(Units.Second).of(0.5), // Ramp rate (V/s) is how fast the quasistatic test increases the voltage
          Units.Volts.of(2.0), // Step voltage (V) is the voltage used for the dynamic test (0V right to this voltage)
          Units.Seconds.of(8.0) //  Timeout (s) is the time at which the test quits (for safety purposes)
      ),
      new SysIdRoutine.Mechanism(
          (voltage) -> runEscalatorSysIdVolts(voltage.in(Units.Volts)), // Voltage Consumer is a method that sets the motor voltage to use for the next test step
          (SysIdRoutineLog log) -> { // Log consumer is a method that returns all of the data from the sensors that we need to collect
            log.motor("Hopper Escalator")
                .voltage(Units.Volts.of(m_escalatorMotor.get() * RobotController.getBatteryVoltage()))
                .angularPosition(Units.Rotations.of(getEscalatorPositionRot()))
                .angularVelocity(Units.RotationsPerSecond.of(getEscalatorVelocityRotPerSec()));
          },
          this));


  private final CanSub m_canSub;

  // Request objects for velocity PID control
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
    SmartDashboard.putNumber("Singulator Velocity", getSingulatorVelocityRotPerSec());
    SmartDashboard.putNumber("Singulator Power", m_singulatorMotor.get());
    SmartDashboard.putBoolean("Escalator Auto", m_escalatorAutomationEnabled);
    SmartDashboard.putNumber("Escalator Target", m_targetEscalatorVelocityRps);
    SmartDashboard.putNumber("Escalator Velocity", getEscalatorVelocityRotPerSec());
    SmartDashboard.putNumber("Escalator Power", m_escalatorMotor.get());
    SmartDashboard.putBoolean("isFull", isFull());

  }

  public void setSingulatorPower(double power) {
    // Disable TalonFX velocity control to stop interferring 
    disableSingulatorAutomation();
    m_singulatorMotor.set(power);
  }

  public void setSingulatorVoltage(double volts) {
    SmartDashboard.putNumber("Hop Sin Volts", volts);
    m_singulatorMotor.setVoltage(volts);
  }

  public void setEscalatorPower(double power) {
    m_escalatorMotor.set(power);
  }

  public void setEscalatorVoltage(double volts) {
    SmartDashboard.putNumber("Hop Esc Volts", volts);
    m_escalatorMotor.set(volts);
  }

  public double getSingulatorRot() {
    return m_singulatorMotor.getPosition().getValueAsDouble();
  }

  public double getSingulatorVelocityRotPerSec() {
    return m_singulatorMotor.getVelocity().getValueAsDouble();
  }


  public double getEscalatorPositionRot() {
    return m_escalatorMotor.getPosition().getValueAsDouble();
  }

  public double getEscalatorVelocityRotPerSec() {
    return m_escalatorMotor.getVelocity().getValueAsDouble();
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
    m_singulatorMotor.setControl(new VelocityVoltage(0.0).withSlot(0).withVelocity(m_targetSingulatorVelocityRps)
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
        - getSingulatorVelocityRotPerSec()) < Constants.Hopper.kSingulatorVelocityToleranceRotPerSec) {
      return true;
    }
    return false;
  }

  ////////////////////////////// Escalator automation //////////////////////////////
  public void enableEscalatorAutomation() {
    m_escalatorAutomationEnabled = true;
    // Use TalonFX's PID control to set velocity
    m_escalatorMotor.setControl(new VelocityVoltage(0.0).withSlot(0).withVelocity(m_targetEscalatorVelocityRps));
  }

  public void disableEscalatorAutomation() {
    m_escalatorAutomationEnabled = false;
    m_escalatorMotor.setControl(new DutyCycleOut(0.0)); // Disable velocity control
  }

  // Set Escalator velocity with PID in RPS
  public void setEscalatorVelocity(double velocityRps) {
    m_targetEscalatorVelocityRps = velocityRps;
    enableEscalatorAutomation();
  }

  public boolean isEscalatorAtTargetVelocity() {
    if(Math
        .abs(m_targetEscalatorVelocityRps
            - getEscalatorVelocityRotPerSec()) < Constants.Hopper.kEscalatorVelocityToleranceRotPerSec) {
      return true;
    }
    return false;
  }

  ////////////////////////////// Escalator SysId and Tests //////////////////////////////
  public void runEscalatorSysIdVolts(double volts) {
    volts = MathUtil.clamp(volts, -(Constants.Hopper.kEscalatorMaxPower * 12.0),
        (Constants.Hopper.kEscalatorMaxPower * 12.0));
    setEscalatorVoltage(volts);
    // TODO: Implement this
  }

  public Command escalatorSysIdQuasistaticCmd(SysIdRoutine.Direction dir) {
    return m_escalatorSysIdRoutine.quasistatic(dir);
  }

  public Command escalatorSysIdDynamicCmd(SysIdRoutine.Direction dir) {
    return m_escalatorSysIdRoutine.dynamic(dir);
  }

  ////////////////////////////// Singulator SysId and Tests //////////////////////////////
  public void runSingulatorSysIdVolts(double volts) {
    //check if we're at max power
    MathUtil.clamp(volts, -(Constants.Hopper.kSingulatorMaxPower * 12.0),
        (Constants.Hopper.kSingulatorMaxPower * 12.0));
    setSingulatorVoltage(volts);
  }

  public Command singulatorSysIdQuasistaticCmd(SysIdRoutine.Direction dir) {
    return m_singulatorSysIdRoutine.quasistatic(dir);
  }

  public Command singulatorSysIdDynamicCmd(SysIdRoutine.Direction dir) {
    return m_singulatorSysIdRoutine.dynamic(dir);
  }


}

