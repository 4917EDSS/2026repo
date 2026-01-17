// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.logging.Logger;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

public class IntakeSub extends SubsystemBase {
  private static Logger m_logger = Logger.getLogger(IntakeSub.class.getName());
  private boolean m_intakeison = false;
  private final TalonFX m_IntakeMotor = new TalonFX(Constants.CanIds.kIntakeMotor); // To be changed later
    private final TalonFX m_IntakeArmMotor = new TalonFX(Constants.CanIds.kIntakeArmMotor);
  /** Creates a new IntakeSub. */
  public IntakeSub() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("intake status", m_intakeison);
  }
  public void intake () {
    m_logger.info(
      "intake"
    );
    m_intakeison = true;
  }
  public void setPower(double power) {
    m_IntakeMotor.set(power);
    SmartDashboard.putNumber("Intake power", power);
  }

}