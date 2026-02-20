// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;


public class ShooterSub extends SubsystemBase {
  private static Logger m_logger = Logger.getLogger(ShooterSub.class.getName());

  private final SparkMax m_yawMotor = new SparkMax(Constants.CanIds.kShooterYawMotor, MotorType.kBrushless);
  private final SparkMax m_pitchMotor = new SparkMax(Constants.CanIds.kShooterPitchMotor, MotorType.kBrushless);
  private final TalonFX m_flywheelMotorL = new TalonFX(Constants.CanIds.kShooterFlywheelMotorL); // Make ABSOLUTELY sure its left
  private final TalonFX m_flywheelMotorR = new TalonFX(Constants.CanIds.kShooterFlywheelMotorR);

  private final SysIdRoutine m_yawSysIdRoutine = new SysIdRoutine(
      // SysIDRoutine takes a Config object (test parameters) and a Mechanism object (how to move motors and read sensors)
      new SysIdRoutine.Config(
          Units.Volts.per(Units.Second).of(0.25), // Ramp rate (V/s) is how fast the quasistatic test increases the voltage
          Units.Volts.of(3.0), // Step voltage (V) is the voltage used for the dynamic test (0V right to this voltage)
          Units.Seconds.of(8.0) //  Timeout (s) is the time at which the test quits (for safety purposes)
      ),
      new SysIdRoutine.Mechanism(
          (voltage) -> runYawSysIdVolts(voltage.in(Units.Volts)), // Voltage Consumer is a method that sets the motor voltage to use for the next test step
          (SysIdRoutineLog log) -> { // Log consumer is a method that returns all of the data from the sensors that we need to collect
            log.motor("shooterYaw")
                .voltage(Units.Volts.of(m_yawMotor.getAppliedOutput() * RobotController.getBatteryVoltage()))
                .angularPosition(Units.Radians.of(getYawAngleDeg()))
                .angularVelocity(Units.RadiansPerSecond.of(getYawVelocityDegPerSec()));
          },
          this // Subsystem we are testing
      ));

  private final SysIdRoutine m_flywheelSysIdRoutine = new SysIdRoutine(
      // SysIDRoutine takes a Config object (test parameters) and a Mechanism object (how to move motors and read sensors)
      new SysIdRoutine.Config(
          Units.Volts.per(Units.Second).of(0.25), // Ramp rate (V/s) is how fast the quasistatic test increases the voltage
          Units.Volts.of(3.0), // Step voltage (V) is the voltage used for the dynamic test (0V right to this voltage)
          Units.Seconds.of(8.0) //  Timeout (s) is the time at which the test quits (for safety purposes, since you should release the button before then)
      ),
      new SysIdRoutine.Mechanism(
          (voltage) -> runFlywheelSysIdVolts(voltage.in(Units.Volts)), // Voltage Consumer is a method that sets the motor voltage to use for the next test step
          (SysIdRoutineLog log) -> { // Log consumer is a method that returns all of the data from the sensors that we need to collect
            log.motor("shooterFlywheel")
                .voltage(Units.Volts.of(m_flywheelMotorL.get() * RobotController.getBatteryVoltage()))
                .angularPosition(Units.Radians.of(getFlywheelPositionRot()))
                .angularVelocity(Units.RadiansPerSecond.of(getFlywheelVelocityRps()));
          },
          this // Subsystem we are testing
      ));

  private final SimpleMotorFeedforward m_yawFeedforward =
      new SimpleMotorFeedforward(Constants.Shooter.kYawKS, Constants.Shooter.kYawKV);
  private final TrapezoidProfile.Constraints m_yawProfileConstraints = new TrapezoidProfile.Constraints(
      Constants.Shooter.kYawMaxVelocityDegPerSec, Constants.Shooter.kYawMaxAccelerationDegPerSec);
  private final ProfiledPIDController m_yawPidController =
      new ProfiledPIDController(Constants.Shooter.kYawKP, Constants.Shooter.kYawKI, Constants.Shooter.kYawKD,
          m_yawProfileConstraints);
  private final PIDController m_pitchPidController =
      new PIDController(Constants.Shooter.kPitchKP, Constants.Shooter.kPitchKI, Constants.Shooter.kPitchKD);

  private boolean m_flywheelAutomationEnabled = false;
  private boolean m_yawAutomationEnabled = false;
  private boolean m_pitchAutomationEnabled = false;
  private double m_targetYawAngleDeg = 0;
  private double m_targetPitchAngleDeg = 0;
  private double m_targetFlywheelVelocityRps = 0;

  private boolean m_pitchHasBeenReset = false;
  private boolean m_yawHasBeenReset = false;

  /** Creates a new ShooterSub. */
  public ShooterSub() { // Motor Configs need to be tested
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig
        .inverted(true) // Set to true to invert the forward motor direction
        .smartCurrentLimit((int) Constants.Shooter.kYawMaxCurrent) // Current limit in amps
        .idleMode(IdleMode.kBrake).encoder
            .positionConversionFactor(Constants.Shooter.kYawEncoderToDegConversionFactor)
            .velocityConversionFactor(1.0);
    m_yawMotor.configure(motorConfig, com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);

    motorConfig
        .inverted(true) // Set to true to invert the forward motor direction
        .smartCurrentLimit((int) Constants.Shooter.kYawMaxCurrent) // Current limit in amps
        .idleMode(IdleMode.kBrake).encoder
            .positionConversionFactor(Constants.Shooter.kPitchEncoderToDegConversionFactor)
            .velocityConversionFactor(1.0);
    m_pitchMotor.configure(motorConfig, com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);

    TalonFXConfigurator talonFXConfigurator1 = m_flywheelMotorL.getConfigurator();
    TalonFXConfigurator talonFXConfigurator2 = m_flywheelMotorR.getConfigurator();
    //This is how you set a current limit inside the motor (vs on the input power supply)
    //subject to change
    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = Constants.Shooter.kFlywheelMaxCurrent;
    limitConfigs.StatorCurrentLimitEnable = true;
    talonFXConfigurator1.apply(limitConfigs);
    talonFXConfigurator2.apply(limitConfigs);

    FeedbackConfigs flywheelFeedbackConfigs = new FeedbackConfigs();
    flywheelFeedbackConfigs.SensorToMechanismRatio = Constants.Shooter.kFlywheelEncoderToRpsConversionFactor;
    talonFXConfigurator1.apply(flywheelFeedbackConfigs);
    talonFXConfigurator2.apply(flywheelFeedbackConfigs);

    // This is how you can set a deadband, invert the motor rotoation and set brake/coast
    MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    outputConfigs.DutyCycleNeutralDeadband = 0.02; // Ignore values below 2%
    outputConfigs.Inverted = InvertedValue.Clockwise_Positive; // Invert = Clockwise
    outputConfigs.NeutralMode = NeutralModeValue.Coast;
    talonFXConfigurator1.apply(outputConfigs);

    outputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    talonFXConfigurator2.apply(outputConfigs);
    m_flywheelMotorR.setControl(new Follower(m_flywheelMotorL.getDeviceID(), MotorAlignmentValue.Opposed));

    m_yawPidController.setTolerance(Constants.Shooter.kYawTolerance);

    init();
  }

  public void init() {
    m_logger.info("Initializing ShooterSub Subsystem");
    disableFlyhweelAutomation();
    disablePitchAutomation();
    disableYawAutomation();
    m_pitchHasBeenReset = false;
    m_yawHasBeenReset = false;
    setFlywheelPower(0.0);
    setPitchPower(0.0);
    setYawPower(0.0);
    setYawVoltage(0.0); // Doing this too to post the voltage to the dashboard
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Sht Yaw Auto", m_yawAutomationEnabled);
    SmartDashboard.putNumber("Sht Yaw Target", m_targetYawAngleDeg);
    SmartDashboard.putNumber("Sht Yaw Angle", getYawAngleDeg());
    SmartDashboard.putNumber("Sht Yaw Vel", getYawVelocityDegPerSec());
    SmartDashboard.putBoolean("Sht Yaw CCW", isAtYawAtCCWLimit());
    SmartDashboard.putBoolean("Sht Yaw CW", isAtYawAtCWLimit());
    SmartDashboard.putBoolean("Sht Yaw Enc Set", m_yawHasBeenReset);
    // Yaw power sent to dashboard in setPower

    SmartDashboard.putBoolean("Sht Ptc Auto", m_pitchAutomationEnabled);
    SmartDashboard.putNumber("Sht Ptc Target", m_targetPitchAngleDeg);
    SmartDashboard.putNumber("Sht Ptc Angle", getPitchAngleDeg());
    SmartDashboard.putBoolean("Sht Ptc Up Lmt", isAtPitchUpperLimit());
    SmartDashboard.putBoolean("Sht Ptc Down Lmt", isAtPitchLowerLimit());
    // Pitch power sent to dashboard in setPower

    SmartDashboard.putBoolean("Sht Fly Auto", m_flywheelAutomationEnabled);
    SmartDashboard.putNumber("Sht Fly Target", m_targetFlywheelVelocityRps);
    SmartDashboard.putNumber("Sht Fly Velocity", getFlywheelVelocityRps());
    SmartDashboard.putNumber("Sht Fly Power", m_flywheelMotorL.get());
    SmartDashboard.putNumber("Sht Fly Pos", getFlywheelPositionRot());

    if(!m_pitchHasBeenReset && isAtPitchLowerLimit()) {
      resetPitchEncoder();
      m_pitchHasBeenReset = true;
    }

    if(!m_yawHasBeenReset && isAtYawAtCWLimit()) {
      resetYawEncoder();
      m_yawHasBeenReset = true;
    }

    runYawControl(m_yawAutomationEnabled);
    runPitchControl(m_pitchAutomationEnabled);
    // Flywheel control is run on the TalonFX
  }

  public void setYawPower(double power) {
    SmartDashboard.putNumber("Sht Yaw Power", power);
    m_yawMotor.set(power);
  }

  public void setYawVoltage(double volts) {
    SmartDashboard.putNumber("Sht Yaw Volts", volts);
    m_yawMotor.setVoltage(volts);
  }

  public void setPitchPower(double power) {
    SmartDashboard.putNumber("Sht Ptc Power", power);
    m_pitchMotor.set(power);
  }

  public void setPitchVoltage(double volts) {
    SmartDashboard.putNumber("Sht Ptc Volts", volts);
    m_pitchMotor.setVoltage(volts);
  }

  public void setPitchAndYawPower(double pitch, double yaw) {
    setPitchPower(pitch);
    setYawPower(yaw);
  }

  public void setFlywheelPower(double power) {
    m_flywheelMotorL.set(power);
    // Motor 2 should follow motor 1
  }

  public void setFlywheelVoltage(double volts) {
    m_flywheelMotorL.setVoltage(volts);
    // Motor 2 should follow motor 1
  }

  public double getYawAngleDeg() {
    return m_yawMotor.getEncoder().getPosition();
  }

  public double getYawVelocityDegPerSec() {
    return m_yawMotor.getEncoder().getVelocity();
  }

  public double getPitchAngleDeg() {
    return m_pitchMotor.getEncoder().getPosition();
  }

  public double getFlywheelPositionRot() {
    return m_flywheelMotorL.getPosition().getValueAsDouble();
  }

  public double getFlywheelVelocityRps() {
    return m_flywheelMotorL.getVelocity().getValueAsDouble();
  }

  public void resetYawEncoder() {
    m_yawMotor.getEncoder().setPosition(0);
  }

  public void resetPitchEncoder() {
    m_pitchMotor.getEncoder().setPosition(0);
  }

  public boolean isAtYawAtCCWLimit() {
    return m_yawMotor.getForwardLimitSwitch().isPressed();
  }

  public boolean isAtYawAtCWLimit() {
    return m_yawMotor.getReverseLimitSwitch().isPressed();
  }

  public boolean isAtPitchLowerLimit() {
    return m_pitchMotor.getReverseLimitSwitch().isPressed();
  }

  public boolean isAtPitchUpperLimit() {
    return m_pitchMotor.getForwardLimitSwitch().isPressed();
  }

  ////////////////////////////// Yaw automation //////////////////////////////
  public void enableYawAutomation() {
    m_yawAutomationEnabled = true;
  }

  public void disableYawAutomation() {
    m_yawAutomationEnabled = false;
    setYawPower(0.0);
  }

  public void setTargetYawAngle(double angleDeg) {
    m_targetYawAngleDeg = angleDeg;
    m_yawPidController.setGoal(angleDeg);
    runYawControl(true);
    enableYawAutomation();
  }

  public boolean isAtTargetYawAngle() {
    // If the yaw encoder isn't reset, then we can never be at our goal since we don't know where we are
    return m_yawHasBeenReset && m_yawPidController.atGoal();
  }

  // Set power based on difference between target and current yaw
  private void runYawControl(boolean setPower) {
    // Can run automated control if encoder position is unknown
    if(!m_yawHasBeenReset) {
      return;
    }

    double currentAngle = getYawAngleDeg();
    double pidVolts = m_yawPidController.calculate(currentAngle);
    TrapezoidProfile.State setPoint = m_yawPidController.getSetpoint();
    double ffVolts = m_yawFeedforward.calculate(setPoint.velocity);
    double totalVolts = pidVolts + ffVolts;

    // Make sure we don't exceed our maxiumum allowed power (in volts, up to 12V)
    // TODO: Consider using this method instead:  totalVolts = MathUtil.clamp(totalVolts, -Constants.Shooter.kYawMaxPower, Constants.Shooter.kYawMaxPower);
    if(Math.abs(totalVolts) > (Constants.Shooter.kYawMaxPower * 12.0)) {
      double sign = (totalVolts >= 0.0) ? 1.0 : -1.0;
      totalVolts = Constants.Shooter.kYawMaxPower * 12.0 * sign;
    }

    if(setPower) {
      setYawVoltage(totalVolts);
    }
  }

  ////////////////////////////// Pitch automation //////////////////////////////
  public void enablePitchAutomation() {
    m_pitchAutomationEnabled = true;
  }

  public void disablePitchAutomation() {
    m_pitchAutomationEnabled = false;
    setPitchPower(0.0);
  }

  public void setTargetPitchAngle(double angleDeg) {
    m_targetPitchAngleDeg = angleDeg;
    runPitchControl(true);
    enablePitchAutomation();
  }

  public boolean isAtTargetPitchAngle() {
    if(Math.abs(getPitchAngleDeg() - m_targetPitchAngleDeg) < Constants.Shooter.kPitchTolerance) {
      return true;
    }
    return false;
  }

  // Set power based on difference between target and current pitch
  private void runPitchControl(boolean setPower) {
    double currentAngle = getPitchAngleDeg();

    double pidPower = m_pitchPidController.calculate(currentAngle, m_targetPitchAngleDeg);

    // Make sure we don't exceed our maxiumum allowed power
    if(Math.abs(pidPower) > Constants.Shooter.kPitchMaxPower) {
      double sign = (pidPower >= 0.0) ? 1.0 : -1.0;
      pidPower = Constants.Shooter.kPitchMaxPower * sign;
    }

    if(setPower) {
      setPitchPower(pidPower);
    }
  }

  ////////////////////////////// Flywheel automation //////////////////////////////
  public void enableFlyhweelAutomation() {
    m_flywheelAutomationEnabled = true;
    // Using voltage control is more reliable since it takes into account the current battery voltage
    // Run it on the TalonFX instead of on the roboRIO
    // The intial velocity is set to 0.0 but then immediately overridden in withVelocity()
    // Slot 0 is just one of many available configuration slots on the TalonFX (so you can switch between them, if necessary)
    m_flywheelMotorL.setControl(new VelocityVoltage(0.0).withSlot(0).withVelocity(m_targetFlywheelVelocityRps));
  }

  public void disableFlyhweelAutomation() {
    m_flywheelAutomationEnabled = false;
    m_flywheelMotorL.setControl(new DutyCycleOut(0.0));
  }

  public void setTargetFlywheelVelocity(double velocityRps) {
    m_targetFlywheelVelocityRps = velocityRps;
    enableFlyhweelAutomation();
  }

  public boolean isAtTargetFlywheelVelocity() {
    if(Math.abs(
        getFlywheelVelocityRps() - m_targetFlywheelVelocityRps) < Constants.Shooter.kFlywheelVelocityToleranceRps) {
      return true;
    }
    return false;
  }


  ////////////////////////////// SysId and Test //////////////////////////////
  public void runYawSysIdVolts(double volts) {
    // Make sure we're pushing past the limits
    if(((volts > 0) && isAtYawAtCCWLimit()) || ((volts < 0) && isAtYawAtCWLimit())) {
      setYawVoltage(0.0);
      return;
    }

    // Make sure we don't exceed our maxiumum allowed power (relative to 12.0 volts)
    if(Math.abs(volts) > (Constants.Shooter.kYawMaxPower * 12.0)) {
      double sign = (volts >= 0.0) ? 1.0 : -1.0;
      volts = Constants.Shooter.kPitchMaxPower * 12.0 * sign;
    }

    setYawVoltage(volts);
  }

  public Command yawSysIdQuasistatic(SysIdRoutine.Direction dir) {
    return m_yawSysIdRoutine.quasistatic(dir);
  }

  public Command yawSysIdDynamic(SysIdRoutine.Direction dir) {
    return m_yawSysIdRoutine.dynamic(dir);
  }

  public void runFlywheelSysIdVolts(double volts) {
    // Make sure we don't exceed our maxiumum allowed power (relative to 12.0 volts)
    if(Math.abs(volts) > (Constants.Shooter.kFlywheelMaxPower * 12.0)) {
      double sign = (volts >= 0.0) ? 1.0 : -1.0;
      volts = Constants.Shooter.kFlywheelMaxPower * 12.0 * sign;
    }

    setFlywheelVoltage(volts);
  }

  public Command flywheelSysIdQuasistatic(SysIdRoutine.Direction dir) {
    return m_flywheelSysIdRoutine.quasistatic(dir);
  }

  public Command flywheelSysIdDynamic(SysIdRoutine.Direction dir) {
    return m_flywheelSysIdRoutine.dynamic(dir);
  }
}
