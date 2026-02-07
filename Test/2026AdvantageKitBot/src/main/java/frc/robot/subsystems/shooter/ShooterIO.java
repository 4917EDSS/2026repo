package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    public double shooterVelocityRPM = 0.0; // encoder velocity in RPM
    public boolean yawZeroPositionDetected = false; // limit switch
    public double shooterVoltageVolts = 0.0; // applied voltage setpoint
    public double shooterYawAngleDegrees = 0.0; // encoder position degrees
    public double shooterPitchAngleDegrees = 0.0; // encoder position degrees
  }

  /** Update the set of loggable inputs */
  public default void updateInputs(ShooterIOInputs inputs) {}

  // run closed loop at this velocity in RPM
  public default void setShooterVelocityRPM(double velocityRPM) {}

  // run open loop at this voltage in volts
  public default void setShooterVoltage(double volts) {}

  // closed loop set target angle for yaw in degrees
  public default void setTargetYawAngleDegrees(double angleDegrees) {}

  // open loop set yaw voltage
  public default void setShooterYawVoltage(double volts) {}

  /**
   * set pitch motor target angle in degrees
   *
   * @param angleDegrees
   */
  public default void setTargetPitchDegrees(double angleDegrees) {}

  /**
   * set pitch motor voltage
   *
   * @param volts
   */
  public default void setShooterPitchVoltage(double volts) {}
}
