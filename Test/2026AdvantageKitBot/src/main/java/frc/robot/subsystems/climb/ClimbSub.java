// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSub extends SubsystemBase {

  private final TalonFX m_climbMotor = new TalonFX(Constants.CanIds.kClimbMotor);
  private final DigitalInput m_climbInLimit =
      new DigitalInput(Constants.DioIds.kClimbInLimitSwitch);
  private final DigitalInput m_climbOutLimit =
      new DigitalInput(Constants.DioIds.kClimbOutLimitSwitch);

  /** Creates a new ClimbSub. */
  public ClimbSub() {

    TalonFXConfigurator talonFXConfigurator = m_climbMotor.getConfigurator();

    // This is how you set a current limit inside the motor (vs on the input power supply)
    // subject to change
    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 60; // limit in amps /TODO: determine reasonable limit
    limitConfigs.StatorCurrentLimitEnable = true;
    talonFXConfigurator.apply(limitConfigs);

    // This is how you can set a deadband, invert the motor rotoation and set brake/coast
    MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    outputConfigs.DutyCycleNeutralDeadband = 0.02; // Ignore values below 2%
    outputConfigs.Inverted = InvertedValue.Clockwise_Positive; // Invert = Clockwise
    outputConfigs.NeutralMode = NeutralModeValue.Brake;
    talonFXConfigurator.apply(outputConfigs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // stop if the climb is moving and at the up limit
    if (isAtOutLimit() && getPower() > 0) {
      m_climbMotor.set(0.0);
    }
    // stop if the climb is at the bottom limit OR shoots below limit AND it is still moving
    else if ((isAtInLimit() || getPosition() <= 0.0) && (getPower() > 0)) {
      setPower(0);
    }
  }

  // returns if climb is at limit
  public boolean isAtInLimit() {
    return !m_climbInLimit.get();
  }

  public boolean isAtOutLimit() {
    return !m_climbOutLimit.get();
  }

  /**
   * Manually set the power of the climb motor(s).
   *
   * @param power power value -1.0 to 1.0
   */
  public void setPower(double power) {
    m_climbMotor.set(power);
  }

  /** Sets the current angle as the zero angle */
  public void resetPosition() {
    m_climbMotor.setPosition(0);
  }

  /**
   * Returns the current angular position of the climb arm
   *
   * @return position in degrees
   */
  public double getPosition() {
    return m_climbMotor.getPosition().getValueAsDouble();
  }

  /**
   * Returns the current angular velocity of the climb arm
   *
   * @return velocity in degrees per second
   */
  public double getVelocity() {
    return m_climbMotor.getRotorVelocity().getValueAsDouble();
  }

  /**
   * Returns current power between -1 and 1
   *
   * @return power
   */
  public double getPower() {
    return m_climbMotor.get();
  }

  /**
   * Returns how much current the motor is currently drawing
   *
   * @return current in amps or -1.0 if motor can't measure current
   */
  public double getElectricalCurrent() {
    return m_climbMotor.getStatorCurrent().getValueAsDouble();
  }
}
