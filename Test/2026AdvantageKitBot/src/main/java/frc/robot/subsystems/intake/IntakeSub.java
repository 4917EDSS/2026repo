// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.logging.Logger;

public class IntakeSub extends SubsystemBase {
  private static Logger m_logger = Logger.getLogger(IntakeSub.class.getName());
  private boolean m_intakeIsOn = false;
  private final TalonFX m_intakeMotor =
      new TalonFX(Constants.CanIds.kIntakeMotor); // To be changed later
  private final Encoder m_intakeAbsoluteEncoder =
      new Encoder(
          Constants.DioIds.kIntakeAbsoluteEncoder1, Constants.DioIds.kIntakeAbsoluteEncoder2);

  private final TalonFX m_IntakeArmMotor = new TalonFX(Constants.CanIds.kIntakeArmMotor);
  private final DigitalInput m_intakeInLimit =
      new DigitalInput(Constants.DioIds.kIntakeInLimitSwitch);
  private final DigitalInput m_intakeOutLimit =
      new DigitalInput(Constants.DioIds.kIntakeOutLimitSwitch);

  private double m_ArmPower = 0;

  /** Creates a new IntakeSub. */
  public IntakeSub() {
    m_intakeAbsoluteEncoder.setDistancePerPulse(0.0); // Converts encoder ticks to mm
    m_intakeAbsoluteEncoder.setReverseDirection(false);
    resetEncoder();

    TalonFXConfiguration config = new TalonFXConfiguration();

    /*
     * Sets the range when the arm is in down/up position.
     */
    // Define the range: 0 (min) to 20 (max) rotations
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 20.0;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    // Apply to the m_IntakeArmMotor
    m_IntakeArmMotor.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("intake status", m_intakeIsOn);
  }

  public void intake() {
    m_logger.info("intake");
    m_intakeIsOn = true;
  }

  public void setIntakePower(double power) {
    m_intakeMotor.set(power);
  }

  public void setIntakeArmPower(double power) {
    m_IntakeArmMotor.set(power);
  }

  public double getIntakeArmPower() {
    return m_IntakeArmMotor.get();
  }

  public boolean isIntakeAtInLimit() {
    return m_intakeInLimit.get();
  }

  public boolean isIntakeAtOutLimit() {
    return m_intakeOutLimit.get();
  }

  public void resetEncoder() {
    m_intakeAbsoluteEncoder.reset();
  }

  public void getIntakeEncoder() {
    m_intakeAbsoluteEncoder.getDistance();
  }
}
