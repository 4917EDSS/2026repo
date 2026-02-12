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
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ClimbSub extends SubsystemBase {
  private final SparkMax m_deployMotor = new SparkMax(Constants.CanIds.kClimbDeployMotor, MotorType.kBrushless);
  private final TalonFX m_rotateMotor = new TalonFX(Constants.CanIds.kClimbRotateMotor);
  private final SparkLimitSwitch m_inboardLimit = m_deployMotor.getForwardLimitSwitch();
  private final SparkLimitSwitch m_outboardLimit = m_deployMotor.getReverseLimitSwitch();
  private final DigitalInput m_rotateCCWLimit = new DigitalInput(Constants.DioIds.kClimbCCWLimitSwitch);
  private final DigitalInput m_rotateCWLimit = new DigitalInput(Constants.DioIds.kClimbCWLimitSwitch);

  private boolean m_enableDeployAutomation = false;
  private boolean m_enableRotationAutomation = false;
  private double m_targetDeployDistanceMm = 0.0;
  private double m_targetRotationAngleDeg = Constants.Climb.kRotationInitialAngleDeg; // This could be different than the encoder-reset angle


  /** Creates a new ClimbSub. */
  public ClimbSub() {
    SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
    sparkMaxConfig
        .inverted(false)
        .smartCurrentLimit((int) Constants.Climb.kDeployMaxCurrent)
        .idleMode(IdleMode.kBrake).encoder
            .positionConversionFactor(Constants.Climb.kDeployEncoderToMmConversionFactor);
    m_deployMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    TalonFXConfigurator talonFXConfigurator = m_rotateMotor.getConfigurator();

    // This is how you set a current limit inside the motor (vs on the input power supply)
    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = Constants.Climb.kRotationMaxCurrent;
    limitConfigs.StatorCurrentLimitEnable = true;
    talonFXConfigurator.apply(limitConfigs);

    // This is how you can set a deadband, invert the motor rotoation and set brake/coast
    MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    outputConfigs.DutyCycleNeutralDeadband = 0.02; // Ignore values below 2%
    outputConfigs.Inverted = InvertedValue.CounterClockwise_Positive; // Invert = Clockwise
    outputConfigs.NeutralMode = NeutralModeValue.Brake;
    talonFXConfigurator.apply(outputConfigs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Cl Dep Auto", m_enableDeployAutomation);
    SmartDashboard.putNumber("Cl Dep Target", m_targetDeployDistanceMm);
    SmartDashboard.putNumber("Cl Dep Dist", getDeployDistanceMm());
    SmartDashboard.putNumber("Cl Dep Power", m_deployMotor.get());
    SmartDashboard.putBoolean("Cl Dep In", isAtDeployInLimit());
    SmartDashboard.putBoolean("Cl Dep Out", isAtDeployOutLimit());

    SmartDashboard.putBoolean("Cl Rot Auto", m_enableRotationAutomation);
    SmartDashboard.putNumber("Cl Rot Target", m_targetRotationAngleDeg);
    SmartDashboard.putNumber("Cl Rot Distance", getRotationAngleDeg());
    SmartDashboard.putNumber("Cl Rot Power", m_rotateMotor.get());
    SmartDashboard.putBoolean("Cl Rot CCW", isAtRotateCCWLimit());
    SmartDashboard.putBoolean("Cl Rot CW", isAtRotateCWLimit());

    // TODO: Add code to zero the distance encoder the first time it hits the IN limit
    // TODO: Add code to zero the rotate encoder the first time it hits the CCW limit

    // TODO: If we use a NEO for deploy, need to add the PID control.  But it looks like it might be
    // a Kraken so hold off on implementing this for now.
  }

  public void setDeployPower(double power) {
    m_deployMotor.set(power);
  }

  /**
   * Manually set the power of the climb rotate motor.
   * 
   * @param power power value -1.0 to 1.0
   */
  public void setRotatePower(double power) {
    m_rotateMotor.set(power);

  }

  public double getDeployDistanceMm() {
    return m_deployMotor.getEncoder().getPosition();
  }

  /**
   * Returns the current angular position of the climb arm
   * 
   * @return position in degrees
   */
  public double getRotationAngleDeg() {
    return m_rotateMotor.getPosition().getValueAsDouble();
  }

  /**
   * Sets the current angle as the zero angle
   */
  public void resetRotateEncoder() {
    m_rotateMotor.setPosition(0);
  }

  public boolean isAtDeployInLimit() {
    return m_inboardLimit.isPressed();
  }

  public boolean isAtDeployOutLimit() {
    return m_outboardLimit.isPressed();
  }

  public boolean isAtRotateCCWLimit() {
    return m_rotateCCWLimit.get();
  }

  public boolean isAtRotateCWLimit() {
    return m_rotateCWLimit.get();
  }

  ////////////////////////////// Deploy automation //////////////////////////////
  public void enableDeployAutomation() {
    m_enableDeployAutomation = true;
  }

  public void disableDeployAutomation() {
    m_enableDeployAutomation = false;
    setDeployPower(0.0);
  }

  public void setTargetDeployDistance(double distanceMm) {
    m_targetDeployDistanceMm = distanceMm;
    runDeployDistanceControl(true);
    enableDeployAutomation();
  }

  public boolean isAtTargetDistance() {
    // TODO:  Need to find the difference between the current and target angles and see if that is
    // smaller than the tolerance
    return (getDeployDistanceMm() > m_targetDeployDistanceMm); // maybe put in between +-error value
  }

  private void runDeployDistanceControl(boolean setPower) {
    // TODO: If using SparkMax, run PID control here.  If TalonFX, run it on that controller
  }

  ////////////////////////////// Rotation automation //////////////////////////////
  public void enableRotateAutomation() {
    m_enableRotationAutomation = true;
    // TODO: Start TalonFX control
  }

  public void disableRotateAutomation() {
    m_enableRotationAutomation = false;
    // TODO: Disable TalonFX velocity control (e.g. m_flywheelMotor1.setControl(new DutyCycleOut(0.0)))
  }

  public void setTargetRotateAngle(double angleDeg) {
    m_targetRotationAngleDeg = angleDeg;
    enableRotateAutomation();
  }

  public boolean isAtTargetRotateAngle() {
    // TODO:  Need to find the difference between the current and target angles and see if that is
    // smaller than the tolerance
    return false;
  }
}
