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

  private final TalonFX m_rotateMotor = new TalonFX(Constants.CanIds.kClimbRotateMotor);
  private final SparkMax m_deployMotor = new SparkMax(Constants.CanIds.kClimbDeployMotor, MotorType.kBrushless);
  private final SparkLimitSwitch m_inboardLimit = m_deployMotor.getForwardLimitSwitch();
  private final SparkLimitSwitch m_outboardLimit = m_deployMotor.getReverseLimitSwitch();
  private final DigitalInput m_rotateCCWLimit = new DigitalInput(Constants.DioIds.kClimbCCWLimitSwitch);
  private final DigitalInput m_rotateCWLimit = new DigitalInput(Constants.DioIds.kClimbCWLimitSwitch);


  private final double m_TargetRotationAngle;
  private boolean m_ActivateClimb = false;
  private boolean m_climbdown = false;

  /** Creates a new ClimbSub. */
  public ClimbSub() {
    TalonFXConfigurator talonFXConfigurator = m_rotateMotor.getConfigurator();
    SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();

    sparkMaxConfig
        .inverted(false)
        .smartCurrentLimit(100)
        .idleMode(IdleMode.kBrake).encoder
            .positionConversionFactor(Constants.Climb.kDeployEncoderToMmConversionFactor);

    m_deployMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //This is how you set a current limit inside the motor (vs on the input power supply)
    //subject to change
    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 60; //limit in amps /TODO: determine reasonable limit
    limitConfigs.StatorCurrentLimitEnable = true;
    talonFXConfigurator.apply(limitConfigs);

    m_TargetRotationAngle = Constants.Climb.kRotationInitialAngleDeg; //If this breaks anything switch to kFinalRotationAngle


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
    SmartDashboard.putBoolean("Climb In Limit", isAtInLimit());
    SmartDashboard.putBoolean("Climb Out Limit", isAtOutLimit());
    SmartDashboard.putNumber("Climb Target Rotation", m_TargetRotationAngle);
    //stop if the climb is moving and at the out limit
    if(isAtOutLimit() && getRotatePower() > 0) {
      m_rotateMotor.set(0.0);
    }


    // TODO: Enable if using Kraken and monitoring limits manually
    // Make sure to also fix which power and limits we're looking at
    // Stop if the climb deploy is moving and at the out limit
    // if(isAtDeployOutLimit() && getDeployPower() > 0) {  
    //   m_rotateMotor.set(0.0);
    // }
    //stop if the climb is at the in limit OR shoots below limit AND it is still moving
    else if((isAtInLimit() || getRotationAngle() <= 0.0) && (getRotatePower() > 0)) {
      setRotatePower(0);
    }

    if(m_ActivateClimb) {
      if(!isAtOutLimit()) {

        setDeployPower(Constants.Climb.kDeployMaxPower);
      } else if(getRotationAngle() - m_TargetRotationAngle < Constants.Climb.kRotationToleranceDeg) {
        setRotatePower(Constants.Climb.kRotationMaxPower);
      } else if(m_climbdown) {
        if(!(getRotationAngle() < Constants.Climb.kRotationToleranceDeg)) {
          setRotatePower(-Constants.Climb.kRotationMaxPower);
        } else if(isAtInLimit()) {
          setDeployPower(-Constants.Climb.kDeployMaxPower);
        }
      }


    }
  }

  //returns if climb is at limit
  public boolean isAtInLimit() {
    return m_inboardLimit.isPressed();
  }

  public boolean isAtOutLimit() {
    return m_outboardLimit.isPressed();
  }

  public boolean isAtCCWLimit() {
    return m_rotateCCWLimit.get();
  }

  public boolean isAtCWLimit() {
    return m_rotateCWLimit.get();
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
    while(getRotationAngle() < angle) {
      setRotatePower(power);
    }
  }

  public void setTargetDeployDistance(double distance, double power) { // same, maybe fixed power
    while(getDeployDistance() < distance) {
      m_deployMotor.set(power);
    }
  }

  /**
   * Sets the current angle as the zero angle
   */
  public void resetPosition() {
    m_rotateMotor.setPosition(0);
  }

  public boolean isAtTargetDistance(double distance) {
    return (getDeployDistance() > distance); // maybe put in between +-error value
  }

  public boolean isEncoderResetSwitchHit() {
    return false; // get cansub rotation limit switch
  }

  /**
   * Returns the current angular position of the climb arm
   * 
   * @return position in degrees
   */
  public double getRotationAngle() {
    return m_rotateMotor.getPosition().getValueAsDouble() * 360;
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
    m_ActivateClimb = true;
    m_climbdown = false;
  }

  public void climbdown() {
    m_ActivateClimb = true;
    m_climbdown = true;
  }

  public void StopClimb() {
    m_ActivateClimb = false;
    m_climbdown = false;
  }
}
