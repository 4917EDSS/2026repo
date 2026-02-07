// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

// 2 Neo 550s for deployment
// 1 Vortex (sparkmax) for belts
// 2 limit switches (in and out)
// 1 position encoder

public class IntakeSub extends SubsystemBase {
  private static Logger m_logger = Logger.getLogger(IntakeSub.class.getName());
  private boolean m_isIntakeOn = false;
  private final SparkMax m_intakeMotor = new SparkMax(Constants.CanIds.kIntakeMotor, MotorType.kBrushless);
  private final Encoder m_intakeAbsoluteEncoder =
      new Encoder(Constants.DioIds.kIntakeAbsoluteEncoder1, Constants.DioIds.kIntakeAbsoluteEncoder2);
  //private final SparkMax m_pivotMotor = new SparkMax(Constants.CanIds.kIntakeMotor, MotorType.kBrushless);


  private final SparkMax m_deployMotorL = new SparkMax(Constants.CanIds.kDeployMotorL, MotorType.kBrushless);
  private final SparkMax m_deployMotorR = new SparkMax(Constants.CanIds.kDeployMotorR, MotorType.kBrushless); // Run in tandem
  private final DigitalInput m_encoderResetSwitch = new DigitalInput(Constants.Intake.DioIds.kIntakeEncoderResetSwitch);

  private double m_armPower = 0.0;
  private boolean m_runPositonControl = false;
  private double m_targetDeployAngle = 0.0;
  private double m_deployKP = 0.022;
  private double m_deployKI = 0.0;
  private double m_deployKD = 0.0;
  private final PIDController m_deployPid = new PIDController(m_deployKP, m_deployKI, m_deployKD);
  // Not the final conversion values

  private boolean m_isIntakeEncoderSet = false;


  /** Creates a new IntakeSub. */
  public IntakeSub() { // Motor Configs need to be tested
    m_intakeAbsoluteEncoder.setDistancePerPulse(0.0); // Converts encoder ticks to mm 
    m_intakeAbsoluteEncoder.setReverseDirection(false);
    resetEncoder();

    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig
        .inverted(false) // Set to true to invert the forward motor direction
        .smartCurrentLimit(60) // Current limit in amps
        .idleMode(IdleMode.kBrake).encoder
            .positionConversionFactor(Constants.Intake.kRotationToDegrees)
            .velocityConversionFactor(0);

    AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();
    encoderConfig.zeroOffset(0);
    motorConfig.apply(encoderConfig);

    // Save the configuration to the motor
    // Only persist parameters when configuring the motor on start up as this
    // operation can be slow
    m_intakeMotor.configure(motorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_deployMotorL.configure(motorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_deployMotorR.configure(motorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("intake status", m_isIntakeOn);

    runIntake(true);

    if(!m_isIntakeEncoderSet) {
      // Adds a counter to the encoder reset switch so that we don't reset position by
      // accident
      if(encoderResetSwitchHit()) {
        m_isIntakeEncoderSet = true;
        setAngle(Constants.Intake.kInAngle);
      }
    }
  }

  public boolean encoderResetSwitchHit() {
    return !m_encoderResetSwitch.get();
  }

  public void setTargetDeployAngle(double angle) {
    // Not doing anything yet
    m_targetDeployAngle = angle;
    runIntake(true);
  }

  public void setBeltPower(double power) {
    m_intakeMotor.set(power);
  }

  public void setDeployPower(double power) {
    m_deployMotorL.set(power);
  }

  public boolean isAtInLimit() {
    return m_deployMotorL.getReverseLimitSwitch().isPressed();
  }

  public boolean isAtOutLimit() {
    return m_deployMotorL.getForwardLimitSwitch().isPressed();
  }

  public void resetEncoder() {
    m_intakeAbsoluteEncoder.reset();
  }

  public void getIntakeEncoder() {
    m_intakeAbsoluteEncoder.getDistance();
  }

  public double getCurrentAngle() {
    return m_deployMotorL.getEncoder().getPosition(); // need converson
  }

  //public double getCurrentAngle() {
  //return m_pivotMotor.getEncoder().getPosition();
  //}

  public void setAngle(double m_TargetAngle) {
    // m_pivotMotor 
    //to do later
  }

  private void runAngleControl(boolean updatePower) {
    double activeAngle = m_targetDeployAngle;
    // if holding set low power 
    double pidPower = m_deployPid.calculate(getCurrentAngle(), activeAngle);
    // setPower(PidPower)
    // to do
  }

  public void runIntake(boolean runIntake) {
    m_logger.info("intake");
    m_isIntakeOn = true;
    double currentAngle = getCurrentAngle();

    double pidPower = m_deployPid.calculate(currentAngle, m_targetDeployAngle);

    if(Math.abs(pidPower) > Constants.Intake.kDeployMaxPower) {
      double sign = (pidPower >= 0.0) ? 1.0 : -1.0;
      pidPower = Constants.Intake.kDeployMaxPower * sign;
    }
    if(runIntake) {
      setDeployPower(pidPower);
    }
  }
}
