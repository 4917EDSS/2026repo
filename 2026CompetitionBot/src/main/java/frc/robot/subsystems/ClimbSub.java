// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;


public class ClimbSub extends SubsystemBase {
  private static Logger m_logger = Logger.getLogger(ClimbSub.class.getName());

  private final TalonFX m_deployMotor = new TalonFX(Constants.CanIds.kClimbDeployMotor);
  private final TalonFX m_rotateMotor = new TalonFX(Constants.CanIds.kClimbRotateMotor);
  private final DigitalInput m_inboardLimit = new DigitalInput(Constants.DioIds.kClimbInLimitSwitch);
  private final DigitalInput m_outboardLimit = new DigitalInput(Constants.DioIds.kClimbOutLimitSwitch);
  private final DigitalInput m_rotateCCWLimit = new DigitalInput(Constants.DioIds.kClimbCCWLimitSwitch);
  private final DigitalInput m_rotateCWLimit = new DigitalInput(Constants.DioIds.kClimbCWLimitSwitch);

  private final SysIdRoutine m_deploySysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Units.Volts.per(Units.Second).of(0.5),
          Units.Volts.of(2.0),
          Units.Seconds.of(8.0)),
      new SysIdRoutine.Mechanism(
          (voltage) -> runDeploySysIdVolts(voltage.in(Units.Volts)),
          (SysIdRoutineLog log) -> {
            log.motor("climbDeploy")
                .voltage(Units.Volts.of(m_deployMotor.get() * RobotController.getBatteryVoltage()))
                .linearPosition(Units.Meters.of(getDeployDistanceM()))
                .linearVelocity(Units.MetersPerSecond.of(getDeployVelocityDegPerSec()));
          },
          this));

  private boolean m_enableDeployAutomation = false;
  private boolean m_enableRotationAutomation = false;
  private boolean m_deployEncoderSet = false;
  private boolean m_rotateEncoderSet = false;
  private double m_targetDeployDistanceM = 0.0;
  private double m_targetRotationAngleDeg = Constants.Climb.kRotationInitialAngleDeg; // This could be different than the encoder-reset angle

  /** Creates a new ClimbSub. */
  public ClimbSub() {
    TalonFXConfigurator talonFXConfiguratorDeploy = m_deployMotor.getConfigurator();
    TalonFXConfigurator talonFXConfiguratorRotate = m_rotateMotor.getConfigurator();

    // TODO Add constants for this
    Slot0Configs slot0DeployConfigs = new Slot0Configs();
    slot0DeployConfigs.kS = Constants.Climb.kDeployKS;
    slot0DeployConfigs.kV = Constants.Climb.kDeployKV;
    slot0DeployConfigs.kA = Constants.Climb.kDeployKA;
    slot0DeployConfigs.kP = Constants.Climb.kDeployKP;
    slot0DeployConfigs.kI = Constants.Climb.kDeployKI;
    slot0DeployConfigs.kD = Constants.Climb.kDeployKD;
    talonFXConfiguratorDeploy.apply(slot0DeployConfigs);

    MotionMagicConfigs mmcDeploy = new MotionMagicConfigs();
    mmcDeploy.MotionMagicCruiseVelocity = Constants.Climb.kDeployMaxVelocityMPerSec;
    mmcDeploy.MotionMagicAcceleration = Constants.Climb.kDeployMaxAccelerationMPerSec;
    talonFXConfiguratorDeploy.apply(mmcDeploy);

    // This is how you set a current limit inside the motor (vs on the input power supply)
    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = Constants.Climb.kRotationMaxCurrent;
    limitConfigs.StatorCurrentLimitEnable = true;
    talonFXConfiguratorRotate.apply(limitConfigs);
    limitConfigs.StatorCurrentLimit = Constants.Climb.kDeployMaxCurrent;
    talonFXConfiguratorDeploy.apply(limitConfigs);

    // This is how you can set a deadband, invert the motor rotoation and set brake/coast
    MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    outputConfigs.DutyCycleNeutralDeadband = 0.02; // Ignore values below 2%
    outputConfigs.Inverted = InvertedValue.CounterClockwise_Positive; // Invert = Clockwise
    outputConfigs.NeutralMode = NeutralModeValue.Brake;
    talonFXConfiguratorRotate.apply(outputConfigs);
    talonFXConfiguratorDeploy.apply(outputConfigs);
  }

  public void init() {
    m_logger.info("Initializing ClimbSub Subsystem");
    disableDeployAutomation();
    disableRotateAutomation();
    m_deployEncoderSet = false;
    m_rotateEncoderSet = false;
    setDeployPower(0.0);
    setRotatePower(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Cl Dep Auto", m_enableDeployAutomation);
    SmartDashboard.putNumber("Cl Dep Target", m_targetDeployDistanceM);
    SmartDashboard.putNumber("Cl Dep Dist", getDeployDistanceM());
    SmartDashboard.putNumber("Cl Dep Power", m_deployMotor.get());
    SmartDashboard.putBoolean("Cl Dep In", isAtDeployInLimit());
    SmartDashboard.putBoolean("Cl Dep Out", isAtDeployOutLimit());

    SmartDashboard.putBoolean("Cl Rot Auto", m_enableRotationAutomation);
    SmartDashboard.putNumber("Cl Rot Target", m_targetRotationAngleDeg);
    SmartDashboard.putNumber("Cl Rot Distance", getRotationAngleDeg());
    SmartDashboard.putNumber("Cl Rot Power", m_rotateMotor.get());
    SmartDashboard.putBoolean("Cl Rot CCW", isAtRotateCCWLimit());
    SmartDashboard.putBoolean("Cl Rot CW", isAtRotateCWLimit());

    // Reset Deploy Encoder If We Have Not Already
    if(!m_deployEncoderSet && isAtDeployInLimit()) {
      resetDeployEncoder();
      m_deployEncoderSet = true;
    }

    // Reset Rotate Encoder If We Have Not Already
    if(!m_rotateEncoderSet && isAtRotateCCWLimit()) {
      resetRotateEncoder();
      m_rotateEncoderSet = true;
    }

  }

  public void setDeployPower(double power) {
    m_deployMotor.set(power);
  }

  /**
   * Manually set the power of the climb rotate motor.
   * 
   * @param power power value -1.0 to 1.0
   */
  public void setRotatePower(double power) {
    m_rotateMotor.set(power);

  }

  public double getDeployDistanceM() {
    return m_deployMotor.get();
  }

  public double getDeployVelocityDegPerSec() {
    return m_deployMotor.getVelocity().getValueAsDouble();
  }

  /**
   * Returns the current angular position of the climb arm
   * 
   * @return position in degrees
   */
  public double getRotationAngleDeg() {
    return m_rotateMotor.getPosition().getValueAsDouble();
  }

  /**
   * Sets the current angle as the zero angle
   */
  public void resetRotateEncoder() {
    m_rotateMotor.setPosition(0.0);
  }

  public void resetDeployEncoder() {
    m_deployMotor.setPosition(Constants.Climb.kDeployInDistanceM);
  }

  public boolean isAtDeployInLimit() {
    return m_inboardLimit.get();
  }

  public boolean isAtDeployOutLimit() {
    return m_outboardLimit.get();
  }

  public boolean isAtRotateCCWLimit() {
    return m_rotateCCWLimit.get();
  }

  public boolean isAtRotateCWLimit() {
    return m_rotateCWLimit.get();
  }

  ////////////////////////////// Deploy automation //////////////////////////////
  public void enableDeployAutomation() {
    // TODO: Convert Distance To Rotation if necessary
    m_deployMotor.setControl(new MotionMagicTorqueCurrentFOC(m_targetDeployDistanceM).withSlot(0));
    m_enableDeployAutomation = true;
  }

  public void disableDeployAutomation() {
    m_enableDeployAutomation = false;
    m_deployMotor.setControl(new DutyCycleOut(0.0));
  }

  public void setTargetDeployDistance(double distanceM) {
    m_targetDeployDistanceM = distanceM;
    enableDeployAutomation();
  }

  public boolean isAtTargetDistance() {
    if((getDeployDistanceM() - m_targetDeployDistanceM) <= Constants.Climb.kDeployToleranceM) {
      return true;
    } else {
      return false;
    }
  }

  ////////////////////////////// Rotation automation //////////////////////////////
  public void enableRotateAutomation() {
    // TODO: Tune PID values on motor (need to configure Slot 0)
    m_enableRotationAutomation = true;
    m_rotateMotor.setControl(new PositionDutyCycle(null).withSlot(0));
  }

  public void disableRotateAutomation() {
    m_enableRotationAutomation = false;
    m_rotateMotor.setControl(new DutyCycleOut(0.0));
  }

  public void setTargetRotateAngle(double angleDeg) {
    m_targetRotationAngleDeg = angleDeg;
    enableRotateAutomation();
  }

  public boolean isAtTargetRotateAngle() {
    if((getRotationAngleDeg() - m_targetRotationAngleDeg) <= Constants.Climb.kRotationToleranceDeg) {
      return true;
    } else {
      return false;
    }
  }

  ////////////////////////////// Deploy SysId //////////////////////////////
  public void runDeploySysIdVolts(double volts) {
    MathUtil.clamp(volts, -(Constants.Climb.kDeployMaxPower * 12.0), (Constants.Climb.kDeployMaxPower * 12.0));
    if(((volts < 0) && isAtDeployInLimit()) || (volts > 0) && isAtDeployOutLimit()) {
      disableDeployAutomation();
    }
    if(isAtRotateCCWLimit() || isAtRotateCWLimit()) {
      disableRotateAutomation();
    }
  }

  public Command deploySysIdQuasistaticCmd(SysIdRoutine.Direction dir) {
    return m_deploySysIdRoutine.quasistatic(dir);
  }

  public Command deploySysIdDynamicCmd(SysIdRoutine.Direction dir) {
    return m_deploySysIdRoutine.dynamic(dir);
  }
}
