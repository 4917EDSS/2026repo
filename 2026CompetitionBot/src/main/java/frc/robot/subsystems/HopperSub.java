// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperSub extends SubsystemBase {

  //IMPORTANT: The term singulator refer to the mechanism in the hopper which forces balls into the shooter.
  //kraken x60 motor for singulator rotation (done)
  //2 ir sensors for hopper full and hopper empty (1 each)
  //internal encoder for singulator velocity

  double targetVelocity = 0.0;
  double targetPower = 0.0;

  private final TalonFX m_singulatorMotor = new TalonFX(Constants.CanIds.kSingulatorMotor);
  private Boolean singulatorMoving = false;

  /** Creates a new HopperSub. */
  public HopperSub() {
    TalonFXConfigurator talonFXConfigurator = m_singulatorMotor.getConfigurator();

    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 60; //this might be too high idk
    limitConfigs.StatorCurrentLimitEnable = true;
    talonFXConfigurator.apply(limitConfigs);

    MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    outputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    outputConfigs.NeutralMode = NeutralModeValue.Brake;
    talonFXConfigurator.apply(outputConfigs);
  }

  public void runSingulator(double speed) {
    m_singulatorMotor.set(speed);
    singulatorMoving = true;
  }

  public void stopSingulator() {
    m_singulatorMotor.set(0.0);
    singulatorMoving = false;
  }

  public double getSingulatorVelocity() {
    singulatorMoving = true;
    return m_singulatorMotor.getVelocity().getValueAsDouble();
  }

  public void setSpindulatorTargetVelocity(double velocity) {
    targetVelocity = velocity;
  }

  public boolean isAtTargetVelocity() {
    if(Constants.HopperConstants.singulatorVelocityFlexibility > Math.abs(targetVelocity - getSingulatorVelocity())) {
      return true;
    }
    return false;
  }

  public void runVelocityControl() {
    if(isAtTargetVelocity()) {
      //stop accelerating
      runSingulator(0);
      return;
    }
    //change power
  }


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Singulating", singulatorMoving);
    SmartDashboard.putNumber("Singulator Velocity", m_singulatorMotor.getVelocity().getValueAsDouble());
    // This method will be called once per scheduler run

  }
}
