// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// 2 Neo 550s for deployment
// 1 Vortex (sparkmax) for belts
// 2 limit switches (in and out)
// 1 position encoder

public class IntakeSub extends SubsystemBase {
  private static Logger m_logger = Logger.getLogger(IntakeSub.class.getName());

  private final SparkFlex m_beltMotor = new SparkFlex(Constants.CanIds.kIntakeMotor, MotorType.kBrushless);
  private final SparkMax m_deployMotorL = new SparkMax(Constants.CanIds.kDeployMotorL, MotorType.kBrushless);
  private final SparkMax m_deployMotorR = new SparkMax(Constants.CanIds.kDeployMotorR, MotorType.kBrushless); // Run in tandem
  private final DigitalInput m_deployInLimit = new DigitalInput(Constants.Intake.DioIds.kDeployInLimit);
  private final DigitalInput m_deployOutLimit = new DigitalInput(Constants.Intake.DioIds.kDeployOutLimit);

  private boolean m_isIntakeOn = false;
  private boolean m_isIntakeEncoderSet = false;
  private double m_targetDeployAngle = 0.0;
  private double m_deployKP = 0.022;
  private double m_deployKI = 0.0;
  private double m_deployKD = 0.0;

  private final PIDController m_deployPid = new PIDController(m_deployKP, m_deployKI, m_deployKD);

  /** Creates a new IntakeSub. */
  public IntakeSub() { // Motor Configs need to be tested
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig
        .inverted(false) // Set to true to invert the forward motor direction
        .smartCurrentLimit(100) // Current limit in amps
        .idleMode(IdleMode.kCoast).encoder
            .positionConversionFactor(Constants.Intake.kRotationToDegrees)
            .velocityConversionFactor(1.0);

    // Save the configuration to the motor
    // Only persist parameters when configuring the motor on start up as this
    // operation can be slow
    m_deployMotorL.configure(motorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_deployMotorR.configure(motorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    motorConfig.encoder.positionConversionFactor(1.0);
    m_beltMotor.configure(motorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("intake status", m_isIntakeOn);

    runDeployAngleControl(true);

    if(!m_isIntakeEncoderSet) {
      // Reset encoder if we're at the in limit and we've never set the encoder
      if(isAtInLimit()) {
        m_isIntakeEncoderSet = true;
        resetDeployEncoder(Constants.Intake.kInAngle);
      }
    }
  }

  public void setTargetDeployAngle(double angle) {
    m_targetDeployAngle = angle;
    m_isIntakeOn = true;
    runDeployAngleControl(true);
  }

  public boolean isAtTargetDeployAngle() {
    return getDeployAngle() == m_targetDeployAngle;
  }

  public void setBeltPower(double power) {
    m_beltMotor.set(power);
  }

  public void setDeployPower(double power) {
    m_deployMotorL.set(power);
  }

  public boolean isAtInLimit() {
    return m_deployInLimit.get();
  }

  public boolean isAtOutLimit() {
    return m_deployOutLimit.get();
  }

  public void resetDeployEncoder(double resetAngle) {
    m_deployMotorL.getEncoder().setPosition(resetAngle);
  }

  public double getDeployAngle() {
    return m_deployMotorL.getEncoder().getPosition();
  }


  public void runDeployAngleControl(boolean runDeployAngleControl) {
    // m_logger.info("intake");
    double currentAngle = getDeployAngle();

    double pidPower = m_deployPid.calculate(currentAngle, m_targetDeployAngle);

    if(Math.abs(pidPower) > Constants.Intake.kDeployMaxPower) {
      double sign = (pidPower >= 0.0) ? 1.0 : -1.0;
      pidPower = Constants.Intake.kDeployMaxPower * sign;
    }

    // If we are at the out limit, set our kP to a very small value so that it will retract if it gets hit
    // TODO: Choose an accurate value for this
    if(isAtTargetDeployAngle()) {
      pidPower = 0.001;
    }
    if(isAtInLimit() && pidPower < 0.0) {
      pidPower = 0.0;

    } else if(isAtOutLimit() && pidPower > 0.001) { // TODO: Make value a constant
      pidPower = 0.001;
    }

    if(runDeployAngleControl) {
      setDeployPower(pidPower);
    }
  }
}
