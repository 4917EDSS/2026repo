// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.time.Instant;
import java.time.temporal.ChronoUnit;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class FeedbackSub extends SubsystemBase {

  private final CommandXboxController m_gameController;
  private Instant m_stopVibratingTime = null;


  /** Creates a new FeedbackSub. */
  public FeedbackSub(CommandXboxController controller) {
    m_gameController = controller;
  }

  @Override
  public void periodic() {
    if(m_stopVibratingTime.isBefore(Instant.now())) {
      disableVibration();
    }

    // This method will be called once per scheduler run
  }

  public void vibrateFeedback(Integer seconds) {
    m_stopVibratingTime = Instant.now().plus(seconds, ChronoUnit.SECONDS);
    enableVibration();
    //Rumble only works on the robot, not in the simulation

  }

  private void enableVibration() {
    m_gameController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
  }

  private void disableVibration() {
    m_gameController.getHID().setRumble(RumbleType.kBothRumble, 0.0);

  }

  public Command vibrate(Integer duration) {
    //return new Command(()-> vibrateFeedback(duration));
    //return new StartEndCommand(null, null, null)
    return run(() -> vibrateFeedback(duration));
  };
}

