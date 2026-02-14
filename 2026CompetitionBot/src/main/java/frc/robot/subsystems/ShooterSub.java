// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ShooterSub extends SubsystemBase {
  private final SparkMax m_yawMotor = new SparkMax(Constants.CanIds.kShooterYawMotor, MotorType.kBrushless);
  private final SparkMax m_pitchMotor = new SparkMax(Constants.CanIds.kShooterPitchMotor, MotorType.kBrushless);
  private final TalonFX m_flywheelMotorL = new TalonFX(Constants.CanIds.kShooterFlywheelMotorL); // Make ABSOLUTELY sure its left
  private final TalonFX m_flywheelMotorR = new TalonFX(Constants.CanIds.kShooterFlywheelMotorR);
  // TODO: Add pitch upper limit switch (connected to SparkMax)

  private final PIDController m_yawPidController =
      new PIDController(Constants.Shooter.kYawKP, Constants.Shooter.kYawKI, Constants.Shooter.kYawKD);
  private final PIDController m_pitchPidController =
      new PIDController(Constants.Shooter.kPitchKP, Constants.Shooter.kPitchKI, Constants.Shooter.kPitchKD);

  private boolean m_flywheelAutomationEnabled = false;
  private boolean m_yawAutomationEnabled = false;
  private boolean m_pitchAutomationEnabled = false;
  private double m_targetYawAngleDeg = 0;
  private double m_targetPitchAngleDeg = 0;
  private double m_targetFlywheelVelocityRps = 0;

  private StatusSignal<AngularVelocity> m_shooterVelocitySignal;

  /** Creates a new ShooterSub. */
  public ShooterSub() { // Motor Configs need to be tested
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig
        .inverted(true) // Set to true to invert the forward motor direction
        .smartCurrentLimit((int) Constants.Shooter.kYawMaxCurrent) // Current limit in amps
        .idleMode(IdleMode.kBrake).encoder
            .positionConversionFactor(Constants.Shooter.kYawEncoderToDegConversionFactor)
            .velocityConversionFactor(1.0);
    m_yawMotor.configure(motorConfig, com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);

    motorConfig
        .inverted(true) // Set to true to invert the forward motor direction
        .smartCurrentLimit((int) Constants.Shooter.kYawMaxCurrent) // Current limit in amps
        .idleMode(IdleMode.kBrake).encoder
            .positionConversionFactor(Constants.Shooter.kPitchEncoderToDegConversionFactor)
            .velocityConversionFactor(1.0);
    m_pitchMotor.configure(motorConfig, com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);

    TalonFXConfigurator talonFXConfigurator1 = m_flywheelMotorL.getConfigurator();
    TalonFXConfigurator talonFXConfigurator2 = m_flywheelMotorR.getConfigurator();
    //This is how you set a current limit inside the motor (vs on the input power supply)
    //subject to change
    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = Constants.Shooter.kFlywheelMaxCurrent;
    limitConfigs.StatorCurrentLimitEnable = true;
    talonFXConfigurator1.apply(limitConfigs);
    talonFXConfigurator2.apply(limitConfigs);

    // This is how you can set a deadband, invert the motor rotoation and set brake/coast
    MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    outputConfigs.DutyCycleNeutralDeadband = 0.02; // Ignore values below 2%
    outputConfigs.Inverted = InvertedValue.Clockwise_Positive; // Invert = Clockwise
    outputConfigs.NeutralMode = NeutralModeValue.Coast;
    talonFXConfigurator1.apply(outputConfigs);

    outputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    talonFXConfigurator2.apply(outputConfigs);
    m_flywheelMotorR.setControl(new Follower(m_flywheelMotorL.getDeviceID(), MotorAlignmentValue.Opposed));

    // Setting up internal encoder for TalonFX
    m_shooterVelocitySignal = m_flywheelMotorL.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Shooter Yaw Auto", m_yawAutomationEnabled);
    SmartDashboard.putNumber("Shooter Yaw Target", m_targetYawAngleDeg);
    SmartDashboard.putNumber("Shooter Yaw Angle", getYawAngleDeg());
    SmartDashboard.putNumber("Shooter Yaw Power", m_yawMotor.get());

    SmartDashboard.putBoolean("Shooter Pitch Auto", m_pitchAutomationEnabled);
    SmartDashboard.putNumber("Shooter Pitch Target", m_targetPitchAngleDeg);
    SmartDashboard.putNumber("Shooter Pitch Angle", getPitchAngleDeg());
    SmartDashboard.putNumber("Shooter Pitch Power", m_pitchMotor.get());

    SmartDashboard.putNumber("Shooter Target Flywheel", m_targetFlywheelVelocityRps);
    SmartDashboard.putBoolean("Shooter Fly Auto", m_flywheelAutomationEnabled);
    SmartDashboard.putNumber("Shooter Fly Target", m_targetFlywheelVelocityRps);
    SmartDashboard.putNumber("Shooter Fly Velocity", getFlywheelVelocityRps());
    SmartDashboard.putNumber("Shooter Fly Power", m_flywheelMotorL.get());

    if(isAtPitchLowerLimit()) {
      resetPitchEncoder();
    }

    // TODO: Check if CW is positive, if so leave this, otherwise swap CCW with CW
    if(isAtYawAtCCWLimit()) {
      resetYawEncoder();
    }

    runYawControl(m_yawAutomationEnabled);
    runPitchControl(m_pitchAutomationEnabled);
    // TODO: This should run on the TalonFX, not here
    //runFlyhweelVelocityControl(m_flywheelAutomationEnabled);
  }

  public void setYawPower(double power) {
    m_yawMotor.set(power);
  }

  public void setPitchPower(double power) {
    m_pitchMotor.set(power);
  }

  public void setPitchAndYawPower(double pitch, double yaw) {
    setPitchPower(pitch);
    setYawPower(yaw);
  }

  public void setFlywheelPower(double power) {
    m_flywheelMotorL.set(power);
    // Motor 2 should follow motor 1
  }

  public double getYawAngleDeg() {
    return m_yawMotor.getEncoder().getPosition();
  }

  public double getPitchAngleDeg() {
    return m_pitchMotor.getEncoder().getPosition();
  }

  public double getFlywheelVelocityRps() {
    return m_flywheelMotorL.getVelocity().getValueAsDouble();
  }

  public void resetYawEncoder() {
    m_yawMotor.getEncoder().setPosition(0);
  }

  public void resetPitchEncoder() {
    m_pitchMotor.getEncoder().setPosition(0);
  }

  public boolean isAtYawAtCCWLimit() {
    return m_yawMotor.getReverseLimitSwitch().isPressed();
  }

  public boolean isAtYawAtCWLimit() {
    return m_yawMotor.getForwardLimitSwitch().isPressed();
  }

  public boolean isAtPitchLowerLimit() {
    return m_pitchMotor.getReverseLimitSwitch().isPressed();
  }

  public boolean isAtPitchUpperLimit() {
    return m_pitchMotor.getForwardLimitSwitch().isPressed();
  }

  ////////////////////////////// Yaw automation //////////////////////////////
  public void enableYawAutomation() {
    m_yawAutomationEnabled = true;
  }

  public void disableYawAutomation() {
    m_yawAutomationEnabled = false;
    setYawPower(0.0);
  }

  public void setTargetYawAngle(double angleDeg) {
    m_targetYawAngleDeg = angleDeg;
    runYawControl(true);
    enableYawAutomation();
  }

  public boolean isAtTargetYawAngle() {
    if(Math.abs(getYawAngleDeg() - m_targetYawAngleDeg) < Constants.Shooter.kYawTolerance) {
      return true;
    }
    return false;
  }

  // Set power based on difference between target and current yaw
  private void runYawControl(boolean setPower) {
    double currentAngle = getYawAngleDeg();

    double pidPower = m_yawPidController.calculate(currentAngle, m_targetYawAngleDeg);

    // Make sure we don't exceed our maxiumum allowed power
    if(Math.abs(pidPower) > Constants.Shooter.kYawMaxPower) {
      double sign = (pidPower >= 0.0) ? 1.0 : -1.0;
      pidPower = Constants.Shooter.kYawMaxPower * sign;
    }

    if(setPower) {
      setYawPower(pidPower);
    }
  }

  ////////////////////////////// Pitch automation //////////////////////////////
  public void enablePitchAutomation() {
    m_pitchAutomationEnabled = true;
  }

  public void disablePitchAutomation() {
    m_pitchAutomationEnabled = false;
    setPitchPower(0.0);
  }

  public void setTargetPitchAngle(double angleDeg) {
    m_targetPitchAngleDeg = angleDeg;
    runPitchControl(true);
    enablePitchAutomation();
  }

  public boolean isAtTargetPitchAngle() {
    if(Math.abs(getPitchAngleDeg() - m_targetPitchAngleDeg) < Constants.Shooter.kPitchTolerance) {
      return true;
    }
    return false;
  }

  // Set power based on difference between target and current pitch
  private void runPitchControl(boolean setPower) {
    double currentAngle = getPitchAngleDeg();

    double pidPower = m_pitchPidController.calculate(currentAngle, m_targetPitchAngleDeg);

    // Make sure we don't exceed our maxiumum allowed power
    if(Math.abs(pidPower) > Constants.Shooter.kPitchMaxPower) {
      double sign = (pidPower >= 0.0) ? 1.0 : -1.0;
      pidPower = Constants.Shooter.kPitchMaxPower * sign;
    }

    if(setPower) {
      setPitchPower(pidPower);
    }
  }

  ////////////////////////////// Flywheel automation //////////////////////////////
  public void enableFlyhweelAutomation() {
    m_flywheelAutomationEnabled = true;
  }

  public void disableFlyhweelAutomation() {
    m_flywheelAutomationEnabled = false;
    m_flywheelMotorL.setControl(new DutyCycleOut(0.0));
  }

  public void setTargetFlywheelVelocity(double velocityRps) {
    m_targetFlywheelVelocityRps = velocityRps;
    // TODO: Start TalonFX control
    enableFlyhweelAutomation();
  }

  public boolean isAtTargetFlywheelVelocity() {
    if(Math.abs(
        getFlywheelVelocityRps() - m_targetFlywheelVelocityRps) < Constants.Shooter.kFlywheelVelocityToleranceRps) {
      return true;
    }
    return false;
  }
}
