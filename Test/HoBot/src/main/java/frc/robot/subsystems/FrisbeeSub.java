// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FrisbeeSub extends SubsystemBase {
  private final TalonFX m_launcher = new TalonFX(15);
  private double power = 0.0;
  private double powerIncrement = 0.1;


  /** Creates a new FrisbeeSub. */
  public FrisbeeSub() {
    TalonFXConfigurator talonFXConfigurator = m_launcher.getConfigurator();
    //This is how you set a current limit inside the motor (vs on the input power supply)
    //subject to change
    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 120.0;
    limitConfigs.StatorCurrentLimitEnable = true;
    talonFXConfigurator.apply(limitConfigs);

    // This is how you can set a deadband, invert the motor rotoation and set brake/coast
    MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    outputConfigs.DutyCycleNeutralDeadband = 0.02; // Ignore values below 2%
    outputConfigs.Inverted = InvertedValue.Clockwise_Positive; // Invert = Clockwise
    outputConfigs.NeutralMode = NeutralModeValue.Coast;
    talonFXConfigurator.apply(outputConfigs);
  }

  @Override
  public void periodic() {
    //System.out.println(power);
    // This method will be called once per scheduler run
  }

  public void stop() {
    m_launcher.set(0.0);
  }

  public void powerUp() {
    if(power <= 0.9) {
      power += powerIncrement;
      m_launcher.set(power);
      System.out.println(power);
    }
  }

  public void powerDown() {
    if(power >= 0.1) {
      power -= powerIncrement;
      m_launcher.set(power);
      System.out.println(power);
    } else {
      power = 0.0;
      System.out.println(power);
    }
  }
}
