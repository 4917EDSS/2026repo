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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ClimbSub extends SubsystemBase {

  private final SparkMax m_deployMotor = new SparkMax(Constants.CanIds.kClimbDeployMotor, MotorType.kBrushless);
  private final TalonFX m_rotateMotor = new TalonFX(Constants.CanIds.kRotateMotor);
  private final SparkLimitSwitch m_inboardLimit = m_deployMotor.getForwardLimitSwitch();
  private final SparkLimitSwitch m_outboardLimit = m_deployMotor.getReverseLimitSwitch();

  private boolean m_activateClimb = false;
  private boolean m_climbDown = false;
  private double m_targetRotationAngle = Constants.Climb.kInitialRotationAngle;

  /** Creates a new ClimbSub. */
  public ClimbSub() {
    SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
    sparkMaxConfig
        .inverted(false)
        .smartCurrentLimit(100) // TODO: determine reasonable limit
        .idleMode(IdleMode.kBrake).encoder
            .positionConversionFactor(Constants.Climb.kDeployEncoderConversionFactor);

    m_deployMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    TalonFXConfigurator talonFXConfigurator = m_rotateMotor.getConfigurator();
    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 100; //limit in amps //TODO: determine reasonable limit
    limitConfigs.StatorCurrentLimitEnable = true;
    talonFXConfigurator.apply(limitConfigs);

    // This is how you can set a deadband, invert the motor rotoation and set brake/coast
    MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    outputConfigs.DutyCycleNeutralDeadband = 0.02;
    // Ignore values below 2%
    outputConfigs.Inverted = InvertedValue.Clockwise_Positive; // Invert = Clockwise
    outputConfigs.NeutralMode = NeutralModeValue.Brake;
    talonFXConfigurator.apply(outputConfigs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("Climb In Limit",isAtDeployInLimit());
    SmartDashboard.putBoolean("Climb Out Limit",isAtDeployOutLimit());
    SmartDashboard.putNumber("Climb Target Rotation", m_targetRotationAngle);

    // TODO: Enable if using Kraken and monitoring limits manually
    // Make sure to also fix which power and limits we're looking at
    // Stop if the climb deploy is moving and at the out limit
    // if(isAtDeployOutLimit() && getDeployPower() > 0) {  
    //   m_rotateMotor.set(0.0);
    // }
    //stop if the climb is at the in limit OR shoots below limit AND it is still moving
    // else if((isAtInLimit() || getRotationAngle() <= 0.0) && (getRotatePower() > 0)) {
    //   setRotatePower(0);
    // }

    // TODO:  Implement the following independently
    // - PID control of deploy (in here, because SparkMax)
    // - PID control of rotation (in TalonFX controller - Motion Magic)
    // - Monitoring any limit switches to turn off power (non-SparkMax-connected ones)
    // - Resetting of both encoders when hitting lower limit the first time
    if(m_activateClimb) {
      if(!isAtDeployOutLimit()) {
        setDeployPower(Constants.Climb.kDeployPower);
      } else if(getRotationAngle() - m_targetRotationAngle < Constants.Climb.kRotationTolerance) {
        setRotatePower(Constants.Climb.kRotationPower);
      } else if(m_climbDown) {
        if(!(getRotationAngle() < Constants.Climb.kRotationTolerance)) {
          setRotatePower(-Constants.Climb.kRotationPower);
        } else if(isAtDeployInLimit()) {
          setDeployPower(-Constants.Climb.kDeployPower);
        }
      }


    }
  }

  //returns if climb is at limit
  public boolean isAtDeployInLimit() {
    return m_inboardLimit.isPressed();
  }

  public boolean isAtDeployOutLimit() {
    return m_outboardLimit.isPressed();
  }

  /**
   * Manually set the power of the climb motor(s).
   * 
   * @param power power value -1.0 to 1.0
   */
  public void setRotatePower(double power) {
    m_rotateMotor.set(power);
  }

  public void setDeployPower(double power) {
    m_deployMotor.set(power);
  }

  public void setTargetAngle(double angle, double power) { // placeholder, needs kraken motion magic
    // TODO:  Set angle only and let Periodic handle the algorithm
    // Also, don't specify power
  }

  public void setTargetDeployDistance(double distance, double power) { // same, maybe fixed power
    // TODO:  Set distance only and let Periodic handle the algorithm
    // Also, don't specify power
  }

  /**
   * Sets the current angle as the zero angle
   */
  public void resetRotatePosition() {
    m_rotateMotor.setPosition(0);
  }

  public boolean isAtTargetDistance(double distance) {
    // TODO:  Just check the target distance minus the current distance (absolute) and compare
    // to a tolerance in Constants
    return (getDeployDistance() > distance); // TODO: Put in between +-error value
  }

  public boolean isAtRotateCCWLimit() {
    return false; // get cansub rotation limit switch
  }

  public boolean isAtRotateCWLimit() {
    return false; // get cansub rotation limit switch
  }

  /**
   * Returns the current angular position of the climb arm
   * 
   * @return position in degrees
   */
  public double getRotationAngle() {
    return m_rotateMotor.getPosition().getValueAsDouble() * 360; // TODO: Check if we have already converted to degrees
  }

  /**
   * Returns the current angular velocity of the climb arm
   * 
   * @return velocity in degrees per second
   */
  public double getVelocity() {
    return m_rotateMotor.getRotorVelocity().getValueAsDouble();
  }

  public double getDeployDistance() {
    return m_deployMotor.getEncoder().getPosition();
  }

  /**
   * Returns current power between -1 and 1
   * 
   * @return power
   */
  public double getRotatePower() {
    return m_rotateMotor.get();
  }

  /**
   * Returns how much current the motor is currently drawing
   * 
   * @return current in amps or -1.0 if motor can't measure current
   */
  public double getElectricalCurrent() {
    return m_rotateMotor.getStatorCurrent().getValueAsDouble();

  }

  public void climb() {
    m_activateClimb = true;
    m_climbDown = false;
  }

  public void climbdown() {
    m_activateClimb = true;
    m_climbDown = true;
  }

  public void StopClimb() {
    m_activateClimb = false;
    m_climbDown = false;
  }
}
