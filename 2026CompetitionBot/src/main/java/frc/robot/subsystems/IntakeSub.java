// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.logging.Logger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeSub extends SubsystemBase {
  private static Logger m_logger = Logger.getLogger(IntakeSub.class.getName());
  private boolean m_intakeison = false;
  //private final TalonFX m_IntakeMotor = new TalonFX(Constants.CanIds.kIntakeMotor); // To be changed later

  private final SparkMax m_intakeMotor = new SparkMax(3, MotorType.kBrushless);

  private final TalonFX m_IntakeArmMotor = new TalonFX(Constants.CanIds.kIntakeArmMotor);
  private double m_ArmPower = 0;
  

  /** Creates a new IntakeSub. */
  public IntakeSub() {
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
    //m_IntakeArmMotor.getConfigurator().apply(config);



    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig
        .inverted(true) // Set to true to invert the forward motor direction
        .smartCurrentLimit(60) // Current limit in amps
        .idleMode(IdleMode.kBrake);

    m_intakeMotor.configure(motorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("intake status", m_intakeison);
  }

  public void intake() {
    m_logger.info("intake");
    m_intakeison = true;
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

  public void pullArmUp() {
    m_logger.info("Arms are up");
    setIntakeArmPower(-5);
  }

  public void pullArmDown() {
    m_logger.info("Arms are down");
    setIntakeArmPower(5);
  }
}
