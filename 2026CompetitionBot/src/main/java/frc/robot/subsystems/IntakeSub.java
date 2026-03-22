// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class IntakeSub extends SubsystemBase {
  private static Logger m_logger = Logger.getLogger(IntakeSub.class.getName());


  private final SparkFlex m_beltMotor = new SparkFlex(Constants.CanIds.kIntakeBeltMotor, MotorType.kBrushless);
  private final TalonFX m_deployMotor = new TalonFX(Constants.CanIds.kIntakeDeployMotor);
  private final DutyCycleEncoder m_encoder =
      new DutyCycleEncoder(new DigitalInput(Constants.DioIds.kIntakeEncoder), 360, 338.0);

  private final PIDController m_deployPidController =
      new PIDController(Constants.Intake.kDeployKP, Constants.Intake.kDeployKI, Constants.Intake.kDeployKD);

  private boolean m_deployAutomationEnabled = false;
  private double m_targetDeployAngleDeg = 0.0;
  private double m_deployKS;
  private double m_deployKG;

  /** Creates a new IntakeSub. */
  public IntakeSub() { // Motor Configs need to be tested
    // m_deployPidController.setTolerance(Constants.Intake.kDeployToleranceDeg);
    // SparkMaxConfig deployMotorConfig = new SparkMaxConfig();
    // deployMotorConfig
    //     .inverted(false) // Set to true to invert the forward motor direction
    //     .smartCurrentLimit(100) // Current limit in amps
    //     .idleMode(IdleMode.kCoast).encoder
    //         .positionConversionFactor(Constants.Intake.kDeployEncoderToDegConversionFactor)
    //         .velocityConversionFactor(Constants.Intake.kDeployEncoderToDegConversionFactor / 60);


    TalonFXConfigurator talonFXConfigurator = m_deployMotor.getConfigurator();
    //This is how you set a current limit inside the motor (vs on the input power supply)
    //subject to change
    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = Constants.Intake.kDeployMaxCurrent;
    limitConfigs.StatorCurrentLimitEnable = true;
    talonFXConfigurator.apply(limitConfigs);

    FeedbackConfigs DeployFeedbackConfigs = new FeedbackConfigs();
    DeployFeedbackConfigs.SensorToMechanismRatio = Constants.Intake.kDeployEncoderToDegConversionFactor;
    talonFXConfigurator.apply(DeployFeedbackConfigs);

    // This is how you can set a deadband, invert the motor rotoation and set brake/coast
    MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    outputConfigs.DutyCycleNeutralDeadband = 0.02; // Ignore values below 2%
    outputConfigs.Inverted = InvertedValue.Clockwise_Positive; // Invert = Clockwise
    outputConfigs.NeutralMode = NeutralModeValue.Coast;
    talonFXConfigurator.apply(outputConfigs);

    outputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

    // Save the configuration to the motor
    // Only persist parameters when configuring the motor on start up as this
    // operation can be slow

    SparkMaxConfig beltMotorConfig = new SparkMaxConfig();
    beltMotorConfig
        .inverted(false) // Set to true to invert the forward motor direction
        .smartCurrentLimit(60) // Current limit in amps
        .idleMode(IdleMode.kCoast);

    m_beltMotor.configure(beltMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void init() {
    m_logger.info("Initializing IntakeSub Subsystem");
    disableDeployAutomation();
    setDeployVoltage(0.0);
    m_beltMotor.set(0.0);
    SmartDashboard.putNumber("totalVolts", 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Intake Auto", m_deployAutomationEnabled);
    SmartDashboard.putNumber("Intake Target Angle", m_targetDeployAngleDeg);
    SmartDashboard.putNumber("Intake Current Angle", getDeployAngleDeg());
    SmartDashboard.putNumber("Int Belt Current", m_beltMotor.getOutputCurrent());

    // Run the deploy-angle PID but only set the motor power if automation is currently enabled
    runDeployAngleControl(m_deployAutomationEnabled);
  }

  public void setBeltVoltage(double volts) {
    m_beltMotor.setVoltage(volts);
    SmartDashboard.putNumber("Int Belt Tar Volts", volts);
  }

  public void setDeployVoltage(double volts) {
    SmartDashboard.putNumber("Int Dep Volts", volts);
    m_deployMotor.setVoltage(volts);
  }

  public double getDeployAngleDeg() {
    double encoderValue = m_encoder.get();
    if(encoderValue > 180) {
      encoderValue = (360 - encoderValue) * -1;
    }
    return encoderValue;
  }

  public double getTargetDeployAngleDeg() {
    return m_targetDeployAngleDeg;
  }

  public void resetDeployEncoder(double resetAngleDeg) {
    m_deployMotor.setPosition(resetAngleDeg);
  }

  public boolean inDeploySafetyZone() {
    if(Math.abs(getDeployAngleDeg() - Constants.Intake.kDeployInAngleDeg) < Constants.Intake.kDeployInToleranceDeg) {
      return true;
    }
    return false;
  }

  ////////////////////////////// Deploy automation //////////////////////////////
  public void enableDeployAutomation() {
    m_deployAutomationEnabled = true;
  }

  public void disableDeployAutomation() {
    m_deployAutomationEnabled = false;
    setDeployVoltage(0.0);
  }

  public void setTargetDeployAngle(double angleDeg) {
    m_targetDeployAngleDeg = angleDeg;
    m_deployPidController.setSetpoint(angleDeg);
    runDeployAngleControl(true);
    enableDeployAutomation();
  }

  public boolean isAtTargetDeployAngle() {
    // If the yaw encoder isn't reset, then we can never be at our goal since we don't know where we are
    return m_deployPidController.atSetpoint();
  }

  // Set power based on difference between target and current angle
  private void runDeployAngleControl(boolean setPower) {
    double currentAngle = getDeployAngleDeg();
    double pidVolts = m_deployPidController.calculate(currentAngle);
    double kSVolts = m_deployKS;
    if(currentAngle - m_targetDeployAngleDeg > 0) {
      kSVolts *= -1;
    }
    double kGVolts = m_deployKG * Math.cos(-currentAngle / 180 * Math.PI);
    double totalVolts = pidVolts + kGVolts + kSVolts;
    SmartDashboard.putNumber("ffVolts", kSVolts + kGVolts);
    SmartDashboard.putNumber("totalVolts", totalVolts);

    // Make sure we don't exceed our maxiumum allowed power (in volts, up to 12V)
    MathUtil.clamp(totalVolts, -Constants.Intake.kDeployMaxVoltage, Constants.Intake.kDeployMaxVoltage);

    // Sets 'safety zones' so that we don't bash into our limits
    // if(currentAngle <= Constants.Intake.kDeployInAngleDeg + Constants.Intake.kDeploySafetyZoneSize
    //     && totalVolts < -Constants.Intake.kDeploySafetyPower) {
    //   totalVolts = -Constants.Intake.kDeploySafetyPower * 12;
    // } else if(currentAngle >= Constants.Intake.kDeployOutAngleDeg - Constants.Intake.kDeploySafetyZoneSize
    //     && totalVolts > Constants.Intake.kDeploySafetyPower) {
    //   totalVolts = Constants.Intake.kDeploySafetyPower * 12;
    // }

    // If our power is negative and we are at the in limit, set the voltage to 0
    // if(isAtInLimit() && totalVolts < 0.0) {
    //totalVolts = 0.0;
    // }

    // We may need to apply a small amount of power to hold the intake in and out

    if(setPower) {
      setDeployVoltage(totalVolts);
    }
  }

  ////////////////////////////// Deploy SysId and Test //////////////////////////////
  private final SysIdRoutine m_deploySysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Units.Volts.per(Units.Seconds).of(0.5),
          Units.Volts.of(2.0),
          Units.Seconds.of(8.0)),
      new SysIdRoutine.Mechanism(
          (voltage) -> runDeploySysIdVolts(voltage.in(Units.Volts)),
          (SysIdRoutineLog log) -> {
            log.motor("intakeDeploy")
                .voltage(Units.Volts.of(m_deployMotor.get() * RobotController.getBatteryVoltage()))
                .angularPosition(Units.Degrees.of(getDeployAngleDeg()));
          },
          this));

  public void runDeploySysIdVolts(double volts) {

    // Make sure we don't exceed our maxiumum allowed power (relative to 12.0 volts)
    volts = MathUtil.clamp(volts, -(Constants.Intake.kDeployMaxVoltage), Constants.Intake.kDeployMaxVoltage);

    setDeployVoltage(volts);
  }

  public Command deploySysIdQuasistatiCmdc(SysIdRoutine.Direction dir) {
    return m_deploySysIdRoutine.quasistatic(dir);
  }

  public Command deploySysIdDynamicCmd(SysIdRoutine.Direction dir) {
    return m_deploySysIdRoutine.dynamic(dir);
  }

  // Tuning
  public void setDeployTuningConstants(double kS, double kG, double kP, double kI, double kD) {
    m_deployKS = kS;
    m_deployKG = kG;
    m_deployPidController.setP(kP);
    m_deployPidController.setI(kI);
    m_deployPidController.setD(kD);
    System.out.println("Intake" + kS + "," + kG + "," + kP + "," + kI + "," + kD + ",");
  }
}
