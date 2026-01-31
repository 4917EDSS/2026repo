// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.logging.Logger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

// 2 Neo 550s for deployment
// 1 Vortex (sparkmax) for belts
// 2 limit switches (in and out)
// 1 position encoder

public class IntakeSub extends SubsystemBase {
  private static Logger m_logger = Logger.getLogger(IntakeSub.class.getName());
  private boolean m_intakeIsOn = false;
  private final SparkMax m_beltMotor = new SparkMax(Constants.CanIds.kIntakeMotor, MotorType.kBrushless);
  private final Encoder m_intakeAbsoluteEncoder =
      new Encoder(Constants.DioIds.kIntakeAbsoluteEncoder1, Constants.DioIds.kIntakeAbsoluteEncoder2);
  private final SparkMax m_pivotMotor = new SparkMax(Constants.CanIds.kIntakeMotor, MotorType.kBrushless);


  private final SparkMax m_deployMotor1 = new SparkMax(Constants.CanIds.kDeployMotor1, MotorType.kBrushless);
  private final SparkMax m_deployMotor2 = new SparkMax(Constants.CanIds.kDeployMotor2, MotorType.kBrushless);
  private final DigitalInput m_intakeInLimit = new DigitalInput(Constants.DioIds.kIntakeInLimitSwitch);
  private final DigitalInput m_intakeOutLimit = new DigitalInput(Constants.DioIds.kIntakeOutLimitSwitch);

  private double m_armPower = 0;
  private final double kRotationToDegrees = 270.0 / 3; // Not the final conversion values


  /** Creates a new IntakeSub. */
  public IntakeSub() {
    m_intakeAbsoluteEncoder.setDistancePerPulse(0.0); // Converts encoder ticks to mm 
    m_intakeAbsoluteEncoder.setReverseDirection(false);
    resetEncoder();

    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig
        .inverted(true) // Set to true to invert the forward motor direction
        .smartCurrentLimit(60) // Current limit in amps
        .idleMode(IdleMode.kBrake).encoder
            .positionConversionFactor(0)
            .velocityConversionFactor(0);

    AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();
    encoderConfig.zeroOffset(0);
    motorConfig.apply(encoderConfig);

    // Save the configuration to the motor
    // Only persist parameters when configuring the motor on start up as this
    // operation can be slow
    m_beltMotor.configure(motorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    m_deployMotor1.configure(motorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    m_deployMotor2.configure(motorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    m_pivotMotor.getIntakeEncoder().SetPositionConversionFactor(kRotationToDegrees);

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
    m_beltMotor.set(power);
  }

  public void setIntakeArmPower(double power) {
    m_deployMotor1.set(power);
  }

  public double getIntakeArmPower() {
    return m_deployMotor1.get();
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

  public double getCurrentAngle() {
    return m_pivotMotor.getIntakeEncoder().getPosition();
  }

}
