// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ShooterSub;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final ShooterSub m_shooterSub = new ShooterSub();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Have four buttons to set the angle we want for quick testing of Yaw
    m_driverController.a().onTrue(new InstantCommand(() -> m_shooterSub.setTargetYawAngle(5.0)));
    m_driverController.b().onTrue(new InstantCommand(() -> m_shooterSub.setTargetYawAngle(45.0)));
    m_driverController.y().onTrue(new InstantCommand(() -> m_shooterSub.setTargetYawAngle(90.0)));
    m_driverController.x().onTrue(new InstantCommand(() -> m_shooterSub.setTargetYawAngle(180.0)));

    // Have four buttons to run the SysId tests so we can determine the feedforward constants using the generated log files and the SysId 2026 app
    m_driverController.leftBumper().whileTrue(m_shooterSub.yawSysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_driverController.rightBumper().whileTrue(m_shooterSub.yawSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_driverController.leftTrigger().whileTrue(m_shooterSub.yawSysIdDynamic(SysIdRoutine.Direction.kForward));
    m_driverController.rightTrigger().whileTrue(m_shooterSub.yawSysIdDynamic(SysIdRoutine.Direction.kReverse));

    m_driverController.povLeft().whileTrue(new InstantCommand(() -> m_shooterSub.disableYawAutomation(), m_shooterSub)
        .andThen(new StartEndCommand(() -> m_shooterSub.setYawPower(0.10), () -> m_shooterSub.setYawPower(0.0),
            m_shooterSub)));
    m_driverController.povRight().whileTrue(new InstantCommand(() -> m_shooterSub.disableYawAutomation(), m_shooterSub)
        .andThen(new StartEndCommand(() -> m_shooterSub.setYawPower(-0.10), () -> m_shooterSub.setYawPower(0.0),
            m_shooterSub)));

    // Zero the encoder
    m_driverController.start().onTrue(new InstantCommand(() -> m_shooterSub.setYawPower(-0.1), m_shooterSub)
        .andThen(new WaitUntilCommand(() -> m_shooterSub.isAtYawAtCWLimit()))
        .andThen(new InstantCommand(() -> m_shooterSub.setYawPower(0.0))));

    // Stop any commands that are running (e.g. SysId tests)
    m_driverController.leftStick().onTrue(new InstantCommand(() -> m_shooterSub.setYawVoltage(0.0), m_shooterSub));
    m_driverController.rightStick().onTrue(new InstantCommand(() -> m_shooterSub.setYawVoltage(0.0), m_shooterSub));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PrintCommand("No auto");
  }
}
