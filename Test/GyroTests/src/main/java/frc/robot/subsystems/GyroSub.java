// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GyroSub extends SubsystemBase {
  private final CANBus kCANivore = new CANBus("*"); // The first CANivore bus
  private final AHRS m_navXGyro = new AHRS(NavXComType.kMXP_SPI);
  private final Pigeon2 m_pigeon2Gyro = new Pigeon2(13, kCANivore);


  /** Creates a new GyroSub. */
  public GyroSub() {
    resetNavXYaw();
    resetPigeonYaw();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("NavX Yaw", getNavXYaw());
    SmartDashboard.putNumber("Pigeon Yaw", getPigeonYaw());
  }

  public void resetNavXYaw() {
    m_navXGyro.reset();
  }

  public double getNavXYaw() {
    return m_navXGyro.getYaw();
  }

  public void resetPigeonYaw() {
    m_pigeon2Gyro.reset();
  }

  public double getPigeonYaw() {
    return m_pigeon2Gyro.getYaw().getValueAsDouble();
  }
}
