// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
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
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.utils.ShooterAimingCalcs;


public class ShooterSub extends SubsystemBase {
  private static Logger m_logger = Logger.getLogger(ShooterSub.class.getName());
  ShooterAimingCalcs m_shooterAimingCalcs;
  DrivetrainSub m_drivetrainSub;

  private final SparkMax m_yawMotor = new SparkMax(Constants.CanIds.kShooterYawMotor, MotorType.kBrushless);
  private final SparkMax m_pitchMotor = new SparkMax(Constants.CanIds.kShooterPitchMotor, MotorType.kBrushless);
  private final TalonFX m_flywheelMotorL = new TalonFX(Constants.CanIds.kShooterFlywheelMotorL); // Make ABSOLUTELY sure its left
  private final TalonFX m_flywheelMotorR = new TalonFX(Constants.CanIds.kShooterFlywheelMotorR);

  // private final SparkAbsoluteEncoder m_yawAbsoluteEncoder = m_yawMotor.getAbsoluteEncoder();

  // Part of old 
  // private final SimpleMotorFeedforward m_yawFeedforward =
  //     new SimpleMotorFeedforward(Constants.Shooter.kYawKS, Constants.Shooter.kYawKV);
  // private final TrapezoidProfile.Constraints m_yawProfileConstraints = new TrapezoidProfile.Constraints(
  //     Constants.Shooter.kYawMaxVelocityDegPerSec, Constants.Shooter.kYawMaxAccelerationDegPerSec);
  private final PIDController m_yawPidController =
      new PIDController(Constants.Shooter.kYawKP, Constants.Shooter.kYawKI, Constants.Shooter.kYawKD);

  // private final ArmFeedforward m_pitchFeedforward =
  //     new ArmFeedforward(Constants.Shooter.kPitchKS, Constants.Shooter.kPitchKG, Constants.Shooter.kPitchKV);
  // private final TrapezoidProfile.Constraints m_pitchProfileConstraints = new TrapezoidProfile.Constraints(
  //     Constants.Shooter.kPitchMaxVelocityDegPerSec, Constants.Shooter.kPitchMaxAccelerationDegPerSec);
  private final PIDController m_pitchPidController =
      new PIDController(Constants.Shooter.kPitchKP, Constants.Shooter.kPitchKI, Constants.Shooter.kPitchKD);

  private boolean m_flywheelAutomationEnabled = false;
  private boolean m_yawAutomationEnabled = false;
  private boolean m_pitchAutomationEnabled = false;
  private double m_targetYawAngleDeg = 0;
  private double m_targetPitchAngleDeg = 0;
  private double m_targetFlywheelVelocityRotsPerSec = 0;

  private boolean m_pitchHasBeenReset = false;
  private boolean m_yawHasBeenReset = false;
  private int m_yawSwitchHitCounter = 0;
  private double m_pitchKS = Constants.Shooter.kPitchKS;
  private double m_pitchKG = Constants.Shooter.kPitchKG;
  private double m_yawKS = Constants.Shooter.kYawKS;

  // private double m_lastYawEncoderRots = 0.0;
  // private double m_currentYawEncoderRots = 0.0;
  // private double m_deltaYaw = 0.0;
  // private int m_yawRotationCount = 10;
  private boolean m_inDeadZone;

  /** Creates a new ShooterSub. */
  public ShooterSub(ShooterAimingCalcs shooterAimingCalcs, DrivetrainSub drivetrainSub) { // Motor Configs need to be tested
    m_shooterAimingCalcs = shooterAimingCalcs;
    m_drivetrainSub = drivetrainSub;
    SparkMaxConfig yawMotorConfig = new SparkMaxConfig();
    yawMotorConfig
        .inverted(true) // Set to true to invert the forward motor direction
        .smartCurrentLimit((int) Constants.Shooter.kYawMaxCurrent) // Current limit in amps
        .idleMode(IdleMode.kBrake).encoder
            .positionConversionFactor(Constants.Shooter.kYawRelativeEncoderConversion);
    yawMotorConfig.apply(new LimitSwitchConfig().forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor)
        .reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor));

    // Yaw absolute encoder configuration
    // yawMotorConfig.absoluteEncoder.positionConversionFactor(1.0);
    // yawMotorConfig.absoluteEncoder.zeroOffset(Constants.Shooter.kYawEncoderOffset);
    // yawMotorConfig.absoluteEncoder.inverted(true);
    // yawMotorConfig.absoluteEncoder.zeroCentered(true);

    m_yawMotor.configure(yawMotorConfig, com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);

    SparkMaxConfig pitchMotorConfig = new SparkMaxConfig();
    pitchMotorConfig
        .inverted(true) // Set to true to invert the forward motor direction
        .smartCurrentLimit((int) Constants.Shooter.kPitchMaxCurrent) // Current limit in amps
        .idleMode(IdleMode.kBrake).encoder
            .positionConversionFactor(Constants.Shooter.kPitchEncoderToDegConversionFactor);
    pitchMotorConfig.apply(new LimitSwitchConfig().forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor)
        .reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor));

    m_pitchMotor.configure(pitchMotorConfig, com.revrobotics.ResetMode.kResetSafeParameters,
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
    flywheelFeedbackConfigs.SensorToMechanismRatio = Constants.Shooter.kFlywheelEncoderToRotsPerSecConversionFactor;
    talonFXConfigurator1.apply(flywheelFeedbackConfigs);
    talonFXConfigurator2.apply(flywheelFeedbackConfigs);

    // Setup the flywheel velocity control
    Slot0Configs slot0FlywheelConfigs = new Slot0Configs();
    slot0FlywheelConfigs.kS = Constants.Shooter.kFlywheelKS;
    slot0FlywheelConfigs.kV = Constants.Shooter.kFlywheelKV;
    slot0FlywheelConfigs.kP = Constants.Shooter.kFlywheelKP;
    slot0FlywheelConfigs.kI = Constants.Shooter.kFlywheelKI;
    slot0FlywheelConfigs.kD = Constants.Shooter.kFlywheelKD;
    talonFXConfigurator1.apply(slot0FlywheelConfigs);

    // This is how you can set a deadband, invert the motor rotoation and set brake/coast
    MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    outputConfigs.DutyCycleNeutralDeadband = 0.02; // Ignore values below 2%
    outputConfigs.Inverted = InvertedValue.Clockwise_Positive; // Invert = Clockwise
    outputConfigs.NeutralMode = NeutralModeValue.Coast;
    //outputConfigs.ControlTimesyncFreqHz = 500; idk could be right we'll see

    talonFXConfigurator1.apply(outputConfigs);

    outputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    talonFXConfigurator2.apply(outputConfigs);
    m_flywheelMotorR.setControl(new Follower(m_flywheelMotorL.getDeviceID(), MotorAlignmentValue.Opposed).withUpdateFreqHz(1000));

    m_yawPidController.setTolerance(Constants.Shooter.kYawTolerance);

    init();
  }

  public void init() {
    m_logger.info("Initializing ShooterSub Subsystem");
    disableFlywheelAutomation();
    disablePitchAutomation();
    disableYawAutomation();
    m_pitchHasBeenReset = false;
    m_yawHasBeenReset = false;
    setFlywheelVoltage(0.0);
    setPitchVoltage(0.0);
    setYawVoltage(0.0);
    m_inDeadZone = false;

    SmartDashboard.putNumber("Set Ptc Pos", 0.0);
    SmartDashboard.putNumber("Set Sht Fly Vel Rps", 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Sht Yaw Auto", m_yawAutomationEnabled);
    SmartDashboard.putNumber("Sht Yaw Angle", getYawAngleDeg());
    SmartDashboard.putNumber("Sht Yaw Vel", getYawVelocityDegPerSec());
    SmartDashboard.putBoolean("Sht Yaw CCW", isAtYawAtCCWLimit());
    SmartDashboard.putBoolean("Sht Yaw CW", isAtYawAtCWLimit());
    SmartDashboard.putBoolean("Sht Yaw Enc Set", m_yawHasBeenReset);
    SmartDashboard.putNumber("Sht Yaw Target", m_targetYawAngleDeg);
    SmartDashboard.putNumber("Sht Yaw Power", m_yawMotor.get());
    SmartDashboard.putNumber("Sht Yaw Amps", m_yawMotor.getOutputCurrent());
    //SmartDashboard.putNumber("Sht Yaw Enc Rot", m_currentYawEncoderRots);
    //SmartDashboard.putNumber("Yaw Rot Count", m_yawRotationCount);

    // Yaw power sent to dashboard in setPower

    SmartDashboard.putBoolean("Sht Ptc Auto", m_pitchAutomationEnabled);
    SmartDashboard.putNumber("Sht Ptc Target", m_targetPitchAngleDeg);
    SmartDashboard.putNumber("Sht Ptc Angle", getPitchAngleDeg());
    SmartDashboard.putBoolean("Sht Ptc Enc Set", m_pitchHasBeenReset);
    SmartDashboard.putBoolean("Sht Ptc Up Lmt", isAtPitchUpperLimit());
    SmartDashboard.putBoolean("Sht Ptc Down Lmt", isAtPitchLowerLimit());
    SmartDashboard.putNumber("Shot Ptc Amps", m_pitchMotor.getOutputCurrent());
    // Pitch power sent to dashboard in setPower

    SmartDashboard.putBoolean("Sht Fly Auto", m_flywheelAutomationEnabled);
    SmartDashboard.putNumber("Sht Fly Target", m_targetFlywheelVelocityRotsPerSec);
    SmartDashboard.putNumber("Sht Fly Velocity", getFlywheelVelocityRotsPerSec());
    SmartDashboard.putNumber("Sht Fly Vel Mps", getFlywheelVelocityMetersPerSec());
    SmartDashboard.putNumber("Sht Fly Power", m_flywheelMotorL.get());
    SmartDashboard.putNumber("Sht Fly Pos", getFlywheelPositionRot());
    SmartDashboard.putNumber("Sht Fly Voltage", m_flywheelMotorL.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Sht Fly Amps L", m_flywheelMotorL.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Sht Fly Amps R", m_flywheelMotorR.getSupplyCurrent().getValueAsDouble());

    if(!m_pitchHasBeenReset && isAtPitchLowerLimit()) {
      resetPitchEncoder();
      m_pitchHasBeenReset = true;
    }

    if(!m_yawHasBeenReset && isAtYawAtCCWLimit()) {
      m_yawSwitchHitCounter += 1;

      if(m_yawSwitchHitCounter > 0) {
        resetYawEncoder();
        m_yawSwitchHitCounter = 0;
        m_yawHasBeenReset = true;
      }
    } //else if(isAtYawAtCCWLimit() && (getYawAngleDeg() > 361 || getYawAngleDeg() < 359)) {
    //   m_yawSwitchHitCounter += 1;
    //   if(m_yawSwitchHitCounter > 0) {
    //     resetYawEncoder();
    //     m_yawSwitchHitCounter = 0;
    //   }
    // } else {
    //   m_yawSwitchHitCounter = 0;
    // }

    runYawControl(m_yawAutomationEnabled);
    runPitchControl(m_pitchAutomationEnabled);
    // flywheel control done on talonfx

    // m_currentYawEncoderRots = m_yawAbsoluteEncoder.getPosition();
    // m_deltaYaw = m_currentYawEncoderRots - m_lastYawEncoderRots;
    // // Detect yaw wraparound
    // if(m_deltaYaw > 0.75) {

    //   m_yawRotationCount--; // wrapped backward
    // } else if(m_deltaYaw < -0.75) {
    //   m_yawRotationCount++; // wrapped forward
    // }

    // m_lastYawEncoderRots = m_currentYawEncoderRots;
  }

  public void setFlywheelTuningConstants(double kS, double kV, double kP, double kI, double kD) {
    TalonFXConfigurator talonFXConfigurator1 = m_flywheelMotorL.getConfigurator();
    Slot0Configs slot0FlywheelConfigs = new Slot0Configs();
    slot0FlywheelConfigs.kS = kS;
    slot0FlywheelConfigs.kV = kV;
    slot0FlywheelConfigs.kP = kP;
    slot0FlywheelConfigs.kI = kI;
    slot0FlywheelConfigs.kD = kD;
    talonFXConfigurator1.apply(slot0FlywheelConfigs);
    System.out.println("Shooter" + kS + "," + kV + "," + kP + "," + kI + "," + kD + ",");

  }

  public void setPitchTuningConstants(double kS, double kG, double kP, double kI, double kD) {
    m_pitchKS = kS;
    // m_pitchFeedforward.setKv(kV);
    m_pitchKG = kG;
    m_pitchPidController.setPID(kP, kI, kD);
    System.out.println("pitch" + "," + "," + kP + "," + kI + "," + kD);
  }

  // public void setYawTuningConstants(double kS, double kP, double kI, double kD) {
  //   m_yawKS = kS;
  //   m_yawPidController.setPID(kP, kI, kD);
  //   System.out.println("yaw" + kS + "," + kP + "," + kI + "," + kD + ",");
  // }


  public void setYawVoltage(double volts) {
    m_yawMotor.setVoltage(volts);
  }

  public void setPitchVoltage(double volts) {
    SmartDashboard.putNumber("Sht Ptc Vlt", volts);
    m_pitchMotor.setVoltage(volts);
  }

  public void setFlywheelVoltage(double volts) {
    m_flywheelMotorL.setVoltage(volts);
    // Motor 2 should follow motor 1
  }

  public double getYawAngleDeg() {
    return m_yawMotor.getEncoder().getPosition();//((m_yawRotationCount + m_currentYawEncoderRots + 0.5) * Constants.Shooter.kYawEncoderToDegConversionFactor); //adjust from -0.5 to 0.5 -> 0.0, to 1.0
  }

  public double getYawVelocityDegPerSec() {
    return m_yawMotor.getEncoder().getVelocity();
  }

  public double getPitchAngleDeg() {
    return m_pitchMotor.getEncoder().getPosition();
  }

  public double getPitchVelocityDegPerSec() {
    return m_pitchMotor.getEncoder().getVelocity();
  }

  public double getFlywheelPositionRot() {
    return m_flywheelMotorL.getPosition().getValueAsDouble();
  }

  public double getFlywheelVelocityRotsPerSec() {
    return m_flywheelMotorL.getVelocity().getValueAsDouble();
  }

  public double getFlywheelVelocityMetersPerSec() {
    return m_flywheelMotorL.getVelocity().getValueAsDouble()
        * Constants.Shooter.kFlywheelRotsPerSecToMpsConversionFactor;
  }

  public void resetYawEncoder() {
    m_yawMotor.getEncoder().setPosition(360.0);

    // double currentRots = m_yawAbsoluteEncoder.getPosition();

    // // Checks if the current rotation is far from 0, if so sets our rotation count to 3
    // // This stops us from accidentaly missing entire rotations
    // if(currentRots > 0.25) {
    //   m_yawRotationCount = 3;
    // } else {
    //   m_yawRotationCount = 4;
    // }
    // m_lastYawEncoderRots = currentRots;
    // m_currentYawEncoderRots = currentRots;
  }

  // This is used to run the hit limit switches command and reset the yaw encoder
  public void unsetYawEncoder() {
    m_yawHasBeenReset = false;
  }

  public void resetPitchEncoder() {
    m_pitchMotor.getEncoder().setPosition(Constants.Shooter.kPitchMinAngleDeg);
  }

  public boolean isAtYawAtCCWLimit() {
    return m_yawMotor.getForwardLimitSwitch().isPressed();
  }

  public boolean isAtYawAtCWLimit() {
    return m_yawMotor.getReverseLimitSwitch().isPressed();
  }

  public boolean isInDeadZone() {
    return m_inDeadZone;
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
    setYawVoltage(0.0);
  }

  public void setTargetYawAngle(double angleDeg) {
    SmartDashboard.putNumber("angledeg", angleDeg);
    angleDeg = (angleDeg + 360) % 360.0;
    if((Constants.Shooter.kYawDeadzoneMin < angleDeg && angleDeg < Constants.Shooter.kYawDeadzoneMax)
        || angleDeg > Constants.Shooter.kYawDeadzoneMinWrapparound) {
      angleDeg = 205.0;
      m_inDeadZone = true;
    } else {
      m_inDeadZone = false;
    }
    m_targetYawAngleDeg = angleDeg;
    m_yawPidController.setSetpoint(angleDeg);
    runYawControl(true);
    enableYawAutomation();
  }

  public boolean isAtTargetYawAngle() {
    // If the yaw encoder isn't reset, then we can never be at our goal since we don't know where we are
    return m_yawHasBeenReset && m_yawPidController.atSetpoint();
  }

  // Set power based on difference between target and current yaw
  private void runYawControl(boolean setPower) {
    // Can run automated control if encoder position is unknown
    if(!m_yawHasBeenReset) {
      return;
    }

    double currentAngle = getYawAngleDeg();
    double pidVolts = m_yawPidController.calculate(currentAngle);
    double ffVolts = m_yawKS * Math.signum(pidVolts);
    double totalVolts = pidVolts;
    if(!m_yawPidController.atSetpoint()) {
      totalVolts += ffVolts;
    }
    SmartDashboard.putNumber("pidvolts", pidVolts);

    // Make sure we don't exceed our maxiumum allowed power (in volts, up to 12V)
    totalVolts = MathUtil.clamp(totalVolts, -Constants.Shooter.kYawMaxVoltage, Constants.Shooter.kYawMaxVoltage);

    //Ensure motor doesn't get undervolted by setting voltage to zero if less than ks
    if(Math.abs(totalVolts) < m_yawKS) {
      totalVolts = 0.0;
    }

    SmartDashboard.putNumber("totalVoltsYaw", totalVolts);

    if(setPower && !Double.isNaN(totalVolts)) {
      setYawVoltage(totalVolts);
    }
  }

  ////////////////////////////// Pitch automation //////////////////////////////
  public void enablePitchAutomation() {
    m_pitchAutomationEnabled = true;
  }

  public void disablePitchAutomation() {
    m_pitchAutomationEnabled = false;
    setPitchVoltage(0.0);
  }

  public void setTargetPitchAngle(double angleDeg) {
    m_targetPitchAngleDeg =
        MathUtil.clamp(angleDeg, Constants.Shooter.kPitchMinAngleDeg + 2, Constants.Shooter.kPitchMaxAngleDeg - 2);
    // m_pitchPidController.reset();
    m_pitchPidController.setSetpoint(m_targetPitchAngleDeg);
    runPitchControl(true);
    enablePitchAutomation();
  }

  public boolean isAtTargetPitchAngle() {
    return m_pitchHasBeenReset && m_pitchPidController.atSetpoint();
  }

  // Set power based on difference between target and current pitch
  private void runPitchControl(boolean setPower) {
    // Can run automated control if encoder position is unknown
    if(!m_pitchHasBeenReset) {
      return;
    }

    double currentAngle = getPitchAngleDeg();
    double pidVolts = m_pitchPidController.calculate(currentAngle);
    double kGVolts = m_pitchKG * Math.cos(-currentAngle / 180 * Math.PI);
    double ffVolts = m_pitchKS * Math.signum(pidVolts) + kGVolts;
    double totalVolts = pidVolts + ffVolts;


    // Make sure we don't exceed our maxiumum allowed power (in volts, up to 12V)
    totalVolts =
        MathUtil.clamp(totalVolts, -Constants.Shooter.kPitchMaxVoltage, Constants.Shooter.kPitchMaxVoltage);

    if(setPower && !Double.isNaN(totalVolts)) {
      setPitchVoltage(totalVolts);
    }
  }

  ////////////////////////////// Flywheel automation //////////////////////////////
  private void enableFlyhweelAutomation() {
    m_flywheelAutomationEnabled = true;
    if(m_targetFlywheelVelocityRotsPerSec < Constants.Shooter.kFlywheelMinVelocityRotsPerSec) {
      m_flywheelMotorL.setControl(new DutyCycleOut(0.0));
    } else {
      m_flywheelMotorL
          .setControl(new VelocityVoltage(0.0).withSlot(0).withVelocity(m_targetFlywheelVelocityRotsPerSec)
              .withUpdateFreqHz(1000));
    }
  }

  public void disableFlywheelAutomation() {
    m_flywheelAutomationEnabled = false;
    m_flywheelMotorL.setControl(new DutyCycleOut(0.0));
  }

  public void setTargetFlywheelVelocity(double velocityRotsPerSec) {
    m_targetFlywheelVelocityRotsPerSec = MathUtil.clamp(velocityRotsPerSec,
        Constants.Shooter.kFlywheelMinVelocityRotsPerSec, Constants.Shooter.kFlywheelMaxVelocityRotsPerSec);
    enableFlyhweelAutomation();
  }

  public boolean isAtTargetFlywheelVelocity() {
    if(Math.abs(
        getFlywheelVelocityRotsPerSec()
            - m_targetFlywheelVelocityRotsPerSec) < Constants.Shooter.kFlywheelVelocityToleranceRotsPerSec) {
      return true;
    }
    return false;
  }


  ////////////////////////////// Yaw SysId //////////////////////////////
  private final SysIdRoutine m_yawSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Units.Volts.per(Units.Second).of(0.5),
          Units.Volts.of(3.0),
          Units.Seconds.of(8.0)),
      new SysIdRoutine.Mechanism(
          (voltage) -> runYawSysIdVolts(voltage.in(Units.Volts)),
          (SysIdRoutineLog log) -> {
            log.motor("shooterYaw")
                .voltage(Units.Volts.of(m_yawMotor.getAppliedOutput() * RobotController.getBatteryVoltage()))
                .angularPosition(Units.Degrees.of(getYawAngleDeg()))
                .angularVelocity(Units.DegreesPerSecond.of(getYawVelocityDegPerSec()));
          },
          this));

  public void runYawSysIdVolts(double volts) {
    //if we hit a limit switch then don't set volts
    if(isAtYawAtCCWLimit() && volts > 0 || isAtYawAtCWLimit() && volts < 0) {
      return;
    }
    //set the volts
    setYawVoltage(
        MathUtil.clamp(volts, -(Constants.Shooter.kYawMaxVoltage), (Constants.Shooter.kYawMaxVoltage)));
  }

  public Command yawSysIdQuasistatic(SysIdRoutine.Direction dir) {
    return m_yawSysIdRoutine.quasistatic(dir);
  }

  public Command yawSysIdDynamic(SysIdRoutine.Direction dir) {
    return m_yawSysIdRoutine.dynamic(dir);
  }


  ////////////////////////////// Pitch SysId //////////////////////////////
  private final SysIdRoutine m_pitchSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Units.Volts.per(Units.Second).of(0.5),
          Units.Volts.of(2.0),
          Units.Seconds.of(8.0)),
      new SysIdRoutine.Mechanism(
          (voltage) -> runPitchSysIdVolts(voltage.in(Units.Volts)),
          (SysIdRoutineLog log) -> {
            log.motor("shooterPtc")
                .voltage(Units.Volts.of(m_pitchMotor.getAppliedOutput() * RobotController.getBatteryVoltage()))
                .angularPosition(Units.Degrees.of(getPitchAngleDeg()))
                .angularVelocity(Units.DegreesPerSecond.of(getPitchVelocityDegPerSec()));
          },
          this));


  public void runPitchSysIdVolts(double volts) {
    //if we hit a limit switch then don't set volts
    if(isAtPitchUpperLimit() && volts > 0 || isAtPitchLowerLimit() && volts < 0) {
      return;
    }
    //set the volts
    setPitchVoltage(
        MathUtil.clamp(volts, -(Constants.Shooter.kPitchMaxVoltage), (Constants.Shooter.kPitchMaxVoltage)));
  }

  public Command pitchSysIdQuasistatic(SysIdRoutine.Direction dir) {
    return m_pitchSysIdRoutine.quasistatic(dir);
  }

  public Command pitchSysIdDynamic(SysIdRoutine.Direction dir) {
    return m_pitchSysIdRoutine.dynamic(dir);
  }


  ////////////////////////////// Flywheel SysId //////////////////////////////

  private final SysIdRoutine m_flywheelSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Units.Volts.per(Units.Second).of(0.5),
          Units.Volts.of(2.0),
          Units.Seconds.of(8.0)),
      new SysIdRoutine.Mechanism(
          (voltage) -> runFlywheelSysIdVolts(voltage.in(Units.Volts)),
          (SysIdRoutineLog log) -> {
            log.motor("shooterFlywheel")
                .voltage(Units.Volts.of(m_flywheelMotorL.get() * RobotController.getBatteryVoltage()))
                .angularPosition(Units.Rotations.of(getFlywheelPositionRot()))
                .angularVelocity(Units.RotationsPerSecond.of(getFlywheelVelocityRotsPerSec()));
          },
          this //subsystem we are testing
      ));

  public void runFlywheelSysIdVolts(double volts) {
    // Make sure we don't exceed our maxiumum allowed power (relative to 12.0 volts)


    // Check if we're at max power
    volts =
        MathUtil.clamp(volts, -(Constants.Shooter.kFlywheelMaxVoltage),
            (Constants.Shooter.kFlywheelMaxVoltage));
    setFlywheelVoltage(volts);
  }

  public Command flywheelSysIdQuasistaticCmd(SysIdRoutine.Direction dir) {
    return m_flywheelSysIdRoutine.quasistatic(dir);
  }

  public Command flywheelSysIdDynamicCmd(SysIdRoutine.Direction dir) {
    return m_flywheelSysIdRoutine.dynamic(dir);
  }

  //RUN ALL CONTROL ALGORTHMS
  public void setPitchYawFlywheelTarget(double[] trajectoriesArray) {
    // setTargetPitchAngle(m_shooterAimingCalcs
    //     .getInterpolatedPitchAngle(m_shooterAimingCalcs.getDistanceToHub(m_drivetrainSub.getPose())));
    setTargetPitchAngle(trajectoriesArray[0]);
    setTargetYawAngle(trajectoriesArray[1]);
    // setTargetFlywheelVelocity(m_shooterAimingCalcs.getInterpolatedFlywheelVelocity(
    //     m_shooterAimingCalcs.getDistanceToHub(m_drivetrainSub.getPose())));
    setTargetFlywheelVelocity(trajectoriesArray[2]);
  }

  public void endPitchYawFlywheel() {
    setTargetPitchAngle(getPitchAngleDeg());
    setTargetYawAngle(getYawAngleDeg());
    disableFlywheelAutomation();
  }
}
