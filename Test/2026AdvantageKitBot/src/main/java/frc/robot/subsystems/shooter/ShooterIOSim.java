package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {

  private DCMotorSim m_DcShooterMotoSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getCIM(2), 0.002, 1), DCMotor.getCIM(2));

  private double m_shooterVoltage = 0.0;
  private double m_YawCurrentPosition = 0;
  private double m_PitchCurrentPosition = 0;
  private double m_YawTargetPosition = 0;
  private double m_PitchTargetPosition = 0;
  private boolean m_yawZeroPositionDetected = false;
  private final double maxYawDegPerSec = 30; // 30 degrees per second
  private final double maxPitchDegPerSec = 30; // 30 degrees per second

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // update simulated encoder velocities and positions
    // m_DcYawMotorSim.setInputVoltage(m_yawVoltage);
    // m_DcPitchMotorSim.setInputVoltage(m_pitchVoltage);
    m_DcShooterMotoSim.setInputVoltage(m_shooterVoltage);
    // m_DcYawMotorSim.update(0.02);
    // m_DcPitchMotorSim.update(0.02);
    m_DcShooterMotoSim.update(0.02);

    // yaw and pitch sim
    // Simple simulation: move towards target
    double yawDelta =
        Math.signum(m_YawTargetPosition - m_YawCurrentPosition) * maxYawDegPerSec * 0.02;
    m_YawCurrentPosition += yawDelta;
    // Simple simulation: move towards target
    double pitchDelta =
        Math.signum(m_PitchTargetPosition - m_PitchCurrentPosition) * maxPitchDegPerSec * 0.02;
    m_PitchCurrentPosition += pitchDelta;

    // update inputs
    // inputs.shooterYawAngleDegrees = m_DcYawMotorSim.getAngularPositionRotations() * 360.0;
    // inputs.shooterPitchAngleDegrees = m_DcPitchMotorSim.getAngularPositionRotations() * 360.0;

    inputs.shooterYawAngleDegrees = m_YawCurrentPosition;
    inputs.shooterPitchAngleDegrees = m_PitchCurrentPosition;
    inputs.shooterVelocityRPM = m_DcShooterMotoSim.getAngularVelocityRPM();
    inputs.shooterVoltageVolts = m_shooterVoltage;
    inputs.yawZeroPositionDetected = m_yawZeroPositionDetected;
  }

  @Override
  public void setShooterTargetYawAngleDegrees(double angleDegrees) {
    if (angleDegrees == 0.0) {
      m_yawZeroPositionDetected = true;
    } else {
      m_yawZeroPositionDetected = false;
    }
    m_YawTargetPosition = angleDegrees;
  }

  @Override
  public void setShooterTargetPitchAngleDegrees(double angleDegrees) {
    m_PitchTargetPosition = angleDegrees;
  }

  @Override
  public void setShooterVoltageVolts(double volts) {
    m_shooterVoltage = MathUtil.clamp(volts, -12.0, 12.0);
  }
}
