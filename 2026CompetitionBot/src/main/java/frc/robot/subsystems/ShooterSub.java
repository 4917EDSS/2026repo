// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


/** Creates a new ShooterSub. */
public class ShooterSub extends SubsystemBase {
  private final SparkMax m_yawMotor = new SparkMax(Constants.CanIds.kShooterYawMotor, MotorType.kBrushless);
  private final SparkMax m_pitchMotor = new SparkMax(Constants.CanIds.kShooterPitchMotor, MotorType.kBrushless);
  private final TalonFX m_flywheelMotor1 = new TalonFX(Constants.CanIds.kShooterFlywheelMotor1);
  private final TalonFX m_flywheelMotor2 = new TalonFX(Constants.CanIds.kShooterFlywheelMotor2);
  // TODO: Add yaw CCW and CW limit switches (connected to SparkMax)
  // TODO: Add pitch lower and upper limit switches (connected to SparkMax)

  private final PIDController m_yawPidController =
      new PIDController(Constants.Shooter.kYawKP, Constants.Shooter.kYawKI, Constants.Shooter.kYawKD);
  private final PIDController m_pitchPidController =
      new PIDController(Constants.Shooter.kPitchKP, Constants.Shooter.kPitchKI, Constants.Shooter.kPitchKD);
  // TODO:  Run flywheel velocity control on the TalonFX
  private final SimpleMotorFeedforward m_flyWheelFeedforward =
      new SimpleMotorFeedforward(Constants.Shooter.kFlywheelKS, Constants.Shooter.kFlywheelKV);
  private final PIDController m_flyWheelPID =
      new PIDController(Constants.Shooter.kFlywheelKP, Constants.Shooter.kFlywheelKI, Constants.Shooter.kFlywheelKD);

  private boolean m_flywheelAutomationEnabled = false;
  private boolean m_yawAutomationEnabled = false;
  private boolean m_pitchAutomationEnabled = false;
  private double m_targetYawAngleDeg = 0;
  private double m_targetPitchAngleDeg = 0;
  private double m_targetFlywheelVelocityRps = 0;

  private StatusSignal<AngularVelocity> m_shooterVelocitySignal;


  public ShooterSub() { // Motor Configs need to be tested
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig
        .inverted(false) // Set to true to invert the forward motor direction
        .smartCurrentLimit(Constants.Shooter.ky) // Current limit in amps
        .idleMode(IdleMode.kBrake).encoder
            .positionConversionFactor(0.0)
            .velocityConversionFactor(0.0);

    AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();
    encoderConfig.zeroOffset(0.0);
    motorConfig.apply(encoderConfig);
    m_yawMotor.configure(motorConfig, com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
    m_pitchMotor.configure(motorConfig, com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);


    m_pitchMotor.configure(motorConfig, com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);


    TalonFXConfigurator talonFXConfigurator = m_flywheelMotor1.getConfigurator();
    TalonFXConfigurator talonFXConfigurator2 = m_flywheelMotor2.getConfigurator();
    //This is how you set a current limit inside the motor (vs on the input power supply)
    //subject to change
    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 100; //limit in amps /TODO: determine reasonable limit
    limitConfigs.StatorCurrentLimitEnable = true;
    talonFXConfigurator.apply(limitConfigs);
    talonFXConfigurator2.apply(limitConfigs);


    // This is how you can set a deadband, invert the motor rotoation and set brake/coast
    MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    outputConfigs.DutyCycleNeutralDeadband = 0.02; // Ignore values below 2%
    outputConfigs.Inverted = InvertedValue.Clockwise_Positive; // Invert = Clockwise
    outputConfigs.NeutralMode = NeutralModeValue.Brake;
    talonFXConfigurator.apply(outputConfigs);

    outputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    talonFXConfigurator2.apply(outputConfigs);

    m_flywheelMotor2.setControl(new Follower(m_flywheelMotor1.getDeviceID(), MotorAlignmentValue.Opposed));

    //setting up internal encoder for TalonFX
    m_shooterVelocitySignal = m_flywheelMotor1.getVelocity();
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Shooter Target Yaw", m_targetYawAngleDeg);
    SmartDashboard.putNumber("Shooter Target Pitch", m_targetPitchAngleDeg);
    SmartDashboard.putNumber("Shooter Target Velocity", m_targetFlywheelVelocityRps);


    // This method will be called once per scheduler run

    runYawControl(m_yawAutomationEnabled);
    runPitchControl(m_pitchAutomationEnabled);
    runFlyhweelVelocityControl(m_flywheelAutomationEnabled);
    // runFlywheelBangBang(m_runVelocityControl);
  }

  public boolean isAtYawAtCWLimit() {
    return m_yawMotor.getForwardLimitSwitch().isPressed();
  }


  public boolean isAtYawAtCCWLimit() {
    return m_yawMotor.getReverseLimitSwitch().isPressed();
  }


  public boolean isAtPitchLowerLimit() {
    return m_pitchMotor.getForwardLimitSwitch().isPressed();
  }


  public boolean isAtPitchUpperLimit() {
    return m_pitchMotor.getReverseLimitSwitch().isPressed();
  }

  public void setFlywheelVoltage(double power) {
    m_flywheelMotor1.set(power);
  }

  public void setTargetYawAngle(double angle) {
    // Not doing anything yet
    m_targetYawAngleDeg = angle;

  }

  public void setTargetPitchAngle(double angle) {
    // Not doing anything yet
    m_targetPitchAngleDeg = angle;

  }

  public void setTargetFlywheelVelocity(double velocity) {
    // Not doing anything yet
    m_targetFlywheelVelocityRps = velocity;
  }

  public void resetYawEncoder() {
    m_yawMotor.getEncoder().setPosition(0);
  }

  public double getYawEncoder() {
    return m_yawMotor.getEncoder().getPosition();
  }

  public void resetPitchEncoder() {
    m_pitchMotor.getEncoder().setPosition(0);
  }

  public double getPitchEncoder() {
    return m_pitchMotor.getEncoder().getPosition();
  }


  public double getFlywheelVelocity() {
    //returning in RPM
    return m_shooterVelocitySignal.getValueAsDouble() / 60;
  }

  public boolean isAtTargetFlywheelVelocity() {
    return getFlywheelVelocity() == m_targetFlywheelVelocityRps;
  }

  public boolean isAtTargetPitchAngle() {
    //TO DO, need to convert encoders position to angle
    return getPitchAngle() == m_targetPitchAngleDeg;
  }

  public boolean isAtTargetYawAngle() {
    //TO DO, need to convert encoders position to angle
    return getYawAngle() == m_targetYawAngleDeg;
  }

  public double getYawAngle() {
    return getYawEncoder() * 360; //Is this value correct?
  }

  public double getPitchAngle() {
    return getPitchEncoder() * 360; //Is this value correct?
  }

  public void setYawPower(Double power) {
    m_yawMotor.set(power);
  }

  public void setPitchPower(Double power) {
    m_pitchMotor.set(power);
  }

  public void setFlywheelPower(double power) {
    m_flywheelMotor1.set(power);
  }

  //set current power based on target for yaw
  private void runYawControl(boolean runYawControl) {
    double currentAngle = getYawAngle();

    double pidPower = m_yawPidController.calculate(currentAngle, m_targetYawAngleDeg);
    //TO DO create constant for this
    if(Math.abs(pidPower) > Constants.Shooter.kYawMaxPower) {
      double sign = (pidPower >= 0.0) ? 1.0 : -1.0;
      pidPower = Constants.Shooter.kYawMaxPower * sign;
    }


    setYawPower(pidPower);
  }

  //set current power based on target for pitch
  private void runPitchControl(boolean runPitchControl) {
    Double currentAngle = getPitchAngle();

    double pidPower = m_pitchPidController.calculate(currentAngle, m_targetPitchAngleDeg);
    //TO DO create constant for this
    if(Math.abs(pidPower) > Constants.Shooter.kPitchMaxPower) {
      double sign = (pidPower >= 0.0) ? 1.0 : -1.0;
      pidPower = Constants.Shooter.kPitchMaxPower * sign;
    }


    setPitchPower(pidPower);
  }

  public void enableFlyhweelVelocityControl(boolean run) {
    m_flywheelAutomationEnabled = run;
  }

  //set current power based on target for flywheel velocity
  private void runFlyhweelVelocityControl(boolean run) {
    // Flywheel needs to spin at set velocity prior to m_pivotSub.spinBothFeeders being executed. 
    if(m_flywheelAutomationEnabled) {
      double feedForwardVoltage = m_flyWheelFeedforward.calculate(m_targetFlywheelVelocityRps, 0.0);
      // So far, we don't need the PID control.  Feedforward is doing well on its own
      double pidVoltage = m_flyWheelPID.calculate(getFlywheelVelocity(), m_targetFlywheelVelocityRps);

      setFlywheelPower(feedForwardVoltage + pidVoltage);
    } else {
      setFlywheelPower(0.0);
    }
  }

  private void runFlywheelBangBang(boolean run) {
    // if the current flywheel velocity is less than the target flywheel velocity, speed it up proportionally, if its greater, slow it down
    if(m_flywheelAutomationEnabled) {
      if(getFlywheelVelocity() < m_targetFlywheelVelocityRps) {
        setFlywheelPower(Math
            .min(m_targetPower + ((m_targetFlywheelVelocityRps - getFlywheelVelocity()) / m_targetFlywheelVelocityRps),
                1.0));
      } else if(getFlywheelVelocity() > m_targetFlywheelVelocityRps) {
        setFlywheelPower(
            Math.max(((getFlywheelVelocity() - m_targetFlywheelVelocityRps) / getFlywheelVelocity()), 0.0));
      } else {
        setFlywheelPower(m_targetFlywheelVelocityRps);
      }
    }
  }

}
