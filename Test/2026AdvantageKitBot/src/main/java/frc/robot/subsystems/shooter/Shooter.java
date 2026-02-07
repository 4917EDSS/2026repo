package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO m_io;
  private final ShooterIOInputsAutoLogged m_inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    m_io = io;
  }

  @Override
  public void periodic() {
    // Update inputs from hardware
    m_io.updateInputs(m_inputs);

    // Log inputs
    Logger.processInputs("Shooter", m_inputs);
  }

  public void setTargetYawAngle(double angle) {
    // Not doing anything yet
    m_inputs.shooterYawAngleDegrees = angle;
  }

  public void setTargetPitchAngle(double angle) {
    // Not doing anything yet
    m_inputs.shooterPitchAngleDegrees = angle;
  }

  public void setTargetFlywheelVelocity(double velocity) {
    // Not doing anything yet
    m_inputs.shooterVelocityRPM = velocity;
  }


}
