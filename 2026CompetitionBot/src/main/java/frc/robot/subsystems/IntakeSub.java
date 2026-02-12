// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import java.util.logging.Logger;
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


public class IntakeSub extends SubsystemBase {
  //private static Logger m_logger = Logger.getLogger(IntakeSub.class.getName());

  private final SparkFlex m_beltMotor = new SparkFlex(Constants.CanIds.kIntakeBeltMotor, MotorType.kBrushless);
  private final SparkMax m_deployMotorL = new SparkMax(Constants.CanIds.kIntakeDeployMotorL, MotorType.kBrushless);
  private final SparkMax m_deployMotorR = new SparkMax(Constants.CanIds.kIntakeDeployMotorR, MotorType.kBrushless); // Run in tandem
  private final DigitalInput m_deployInLimit = new DigitalInput(Constants.DioIds.kIntakeDeployInLimit);
  private final DigitalInput m_deployOutLimit = new DigitalInput(Constants.DioIds.kIntakeDeployOutLimit);

  private final PIDController m_deployPid =
      new PIDController(Constants.Intake.kDeployKP, Constants.Intake.kDeployKI, Constants.Intake.kDeployKD);

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

    motorConfig.encoder.positionConversionFactor(1.0); // Don't care about the belt position
    m_beltMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
    m_deployMotorL.set(power);
  }

  public double getDeployAngleDeg() {
    return m_deployMotorL.getEncoder().getPosition();
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

  public void enableDeployAutomation() {
    m_deployAutomationEnabled = true;
  }

  public void disableDeployAutomation() {
    m_deployAutomationEnabled = false;
  }

  public void setTargetDeployAngle(double angleDeg) {
    m_targetDeployAngleDeg = angleDeg;
    runDeployAngleControl(true);
    enableDeployAutomation();
  }

  public boolean isAtTargetDeployAngle() {
    // TODO:  Need to find the difference between the current and target angles and see if that is
    // smaller than the tolerance
    return getDeployAngleDeg() == m_targetDeployAngleDeg;
  }

  public void runDeployAngleControl(boolean setPower) {
    // m_logger.info("intake");
    double currentAngle = getDeployAngleDeg();
    double pidPower = m_deployPid.calculate(currentAngle, m_targetDeployAngleDeg);

    // Make sure we don't exceed our maxiumum allowed power
    if(Math.abs(pidPower) > Constants.Intake.kDeployMaxPower) {
      double sign = (pidPower >= 0.0) ? 1.0 : -1.0;
      pidPower = Constants.Intake.kDeployMaxPower * sign;
    }

    // If we are at the out limit, set our kP to a very small value so that it will retract if it gets hit
    // TODO: Choose an accurate value for this
    // Note: If we end up having to hold an angle that's not against a hard stop, we'll need to use a weak PID instead of a fixed power
    if(isAtTargetDeployAngle()) {
      pidPower = 0.001;
    }
    if(isAtInLimit() && pidPower < 0.0) {
      pidPower = 0.0;

    } else if(isAtOutLimit() && pidPower > 0.001) { // TODO: Make value a constant
      pidPower = 0.001;
    }

    if(setPower) {
      setDeployPower(pidPower);
    }
  }
}
