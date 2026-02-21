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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.wpilibj.DigitalInput;
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
  private final SparkMax m_deployMotorL = new SparkMax(Constants.CanIds.kIntakeDeployMotorL, MotorType.kBrushless);
  private final SparkMax m_deployMotorR = new SparkMax(Constants.CanIds.kIntakeDeployMotorR, MotorType.kBrushless); // Run in tandem
  private final DigitalInput m_deployInLimit = new DigitalInput(Constants.DioIds.kIntakeDeployInLimit);
  private final DigitalInput m_deployOutLimit = new DigitalInput(Constants.DioIds.kIntakeDeployOutLimit);

  private final SysIdRoutine m_deploySysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Units.Volts.per(Units.Seconds).of(0.5),
          Units.Volts.of(2.0),
          Units.Seconds.of(8.0)),
      new SysIdRoutine.Mechanism(
          (voltage) -> runDeploySysIdVolts(voltage.in(Units.Volts)),
          (SysIdRoutineLog log) -> {
            log.motor("intakeDeploy")
                .voltage(Units.Volts.of(m_deployMotorL.getAppliedOutput() * RobotController.getBatteryVoltage()))
                .angularPosition(Units.Degrees.of(getDeployAngleDeg()))
                .angularVelocity(Units.DegreesPerSecond.of(getDeployVelocityDegPerSec()));
          },
          this));

  private final ArmFeedforward m_deployFeedforward =
      new ArmFeedforward(Constants.Intake.kDeployKS, Constants.Intake.kDeployKG, Constants.Intake.kDeployKV);
  private final TrapezoidProfile.Constraints m_deployProfileConstraints = new TrapezoidProfile.Constraints(
      Constants.Intake.kDeployMaxVelocityDegPerSec, Constants.Intake.kDeployMaxAccelerationDegPerSec);
  private final ProfiledPIDController m_deployPidController =
      new ProfiledPIDController(Constants.Intake.kDeployKP, Constants.Intake.kDeployKI, Constants.Intake.kDeployKD,
          m_deployProfileConstraints);

  private boolean m_deployAutomationEnabled = false;
  private boolean m_isIntakeEncoderSet = false;
  private double m_targetDeployAngleDeg = 0.0;


  /** Creates a new IntakeSub. */
  public IntakeSub() { // Motor Configs need to be tested
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig
        .inverted(false) // Set to true to invert the forward motor direction
        .smartCurrentLimit(100) // Current limit in amps
        .idleMode(IdleMode.kCoast).encoder
            .positionConversionFactor(Constants.Intake.kDeployEncoderToDegConversionFactor)
            .velocityConversionFactor(1.0); // Should not be reading deploy velocity (outside of possible kD)

    // Save the configuration to the motor
    // Only persist parameters when configuring the motor on start up as this
    // operation can be slow
    m_deployMotorL.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_deployMotorR.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    motorConfig.encoder.positionConversionFactor(1.0); // Don't care about the belt position.  Vortex is 1:1 gearing.
    m_beltMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void init() {
    m_logger.info("Initializing IntakeSub Subsystem");
    disableDeployAutomation();
    setDeployPower(0.0);
    m_beltMotor.set(0.0);
    m_isIntakeEncoderSet = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Intake Auto", m_deployAutomationEnabled);
    SmartDashboard.putBoolean("Intake In Limit", isAtInLimit());
    SmartDashboard.putBoolean("Intake Out Limit", isAtOutLimit());
    SmartDashboard.putBoolean("Intake Enc Set", m_isIntakeEncoderSet);
    SmartDashboard.putNumber("Intake Target Angle", m_targetDeployAngleDeg);

    // Check if the relative encoder has been zeroed yet or not
    if(!m_isIntakeEncoderSet) {
      // Reset encoder if we're at the in limit and we've never set the encoder
      if(isAtInLimit()) {
        m_isIntakeEncoderSet = true;
        resetDeployEncoder(Constants.Intake.kDeployInAngleDeg);
      }
    }

    // Run the deploy-angle PID but only set the motor power if automation is currently enabled
    runDeployAngleControl(m_deployAutomationEnabled);
  }

  public void setBeltPower(double power) {
    m_beltMotor.set(power);
  }

  public void setDeployPower(double power) {
    SmartDashboard.putNumber("Intake Deploy Power", m_deployMotorL.get());
    m_deployMotorL.set(power);
  }

  public void setDeployVoltage(double volts) {
    SmartDashboard.putNumber("Int Dep Volts", volts);
    m_deployMotorL.setVoltage(volts);
  }

  public double getDeployAngleDeg() {
    return m_deployMotorL.getEncoder().getPosition();
  }

  public double getDeployVelocityDegPerSec() {
    return m_deployMotorL.getEncoder().getVelocity();
  }

  public void resetDeployEncoder(double resetAngleDeg) {
    m_deployMotorL.getEncoder().setPosition(resetAngleDeg);
  }

  public boolean isAtInLimit() {
    return m_deployInLimit.get();
  }

  public boolean isAtOutLimit() {
    return m_deployOutLimit.get();
  }

  ////////////////////////////// Deploy automation //////////////////////////////
  public void enableDeployAutomation() {
    m_deployAutomationEnabled = true;
  }

  public void disableDeployAutomation() {
    m_deployAutomationEnabled = false;
    setDeployPower(0.0);
  }

  public void setTargetDeployAngle(double angleDeg) {
    m_targetDeployAngleDeg = angleDeg;
    m_deployPidController.setGoal(angleDeg);
    runDeployAngleControl(true);
    enableDeployAutomation();
  }

  public boolean isAtTargetDeployAngle() {
    // If the yaw encoder isn't reset, then we can never be at our goal since we don't know where we are
    return m_isIntakeEncoderSet && m_deployPidController.atGoal();
  }

  // Set power based on difference between target and current yaw
  private void runDeployAngleControl(boolean setPower) {
    // Can't run automated control if encoder position is unknown
    if(!m_isIntakeEncoderSet) {
      return;
    }

    double currentAngle = getDeployAngleDeg();
    double pidVolts = m_deployPidController.calculate(currentAngle);
    TrapezoidProfile.State setPoint = m_deployPidController.getSetpoint();
    double ffVolts =
        m_deployFeedforward.calculate(Math.toRadians(Constants.Intake.kDeployOutAngleDeg - 90), setPoint.velocity);
    double totalVolts = pidVolts + ffVolts;

    // Make sure we don't exceed our maxiumum allowed power (in volts, up to 12V)
    MathUtil.clamp(totalVolts, -(Constants.Intake.kDeployMaxPower * 12), Constants.Intake.kDeployMaxPower * 12);

    // We may need to apply a small amount of power to hold the intake in and out

    if(setPower) {
      setDeployVoltage(totalVolts);
    }
  }

  ////////////////////////////// Deploy SysId and Test //////////////////////////////
  public void runDeploySysIdVolts(double volts) {
    // Make sure we're not pushing past the limits
    if(((volts > 0) && isAtOutLimit()) || ((volts < 0) && isAtInLimit())) {
      setDeployVoltage(0.0);
      return;
    }

    // Make sure we don't exceed our maxiumum allowed power (relative to 12.0 volts)
    volts = MathUtil.clamp(volts, -(Constants.Intake.kDeployMaxPower * 12.0), Constants.Intake.kDeployMaxPower * 12.0);

    setDeployVoltage(volts);
  }

  public Command deploySysIdQuasistatiCmdc(SysIdRoutine.Direction dir) {
    return m_deploySysIdRoutine.quasistatic(dir);
  }

  public Command deploySysIdDynamicCmd(SysIdRoutine.Direction dir) {
    return m_deploySysIdRoutine.dynamic(dir);
  }
}
