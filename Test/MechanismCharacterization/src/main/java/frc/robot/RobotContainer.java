// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.QuasistaticYawCmd;
import frc.robot.subsystems.ShooterSub;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private String m_currentTest = "None"; // The name of the currently running test (for logging purposes)
  private double m_minPosition = 999; // The encoder value at the lower limit
  private double m_maxPosition = -999; // The encoder value at the upper limit
  private double m_percentMargin = 0.05; // How far from the min/max to run the text
  private double m_minTestPosition = 999; // Min position * percent buffer space
  private double m_maxTestPosition = -999; // Max position * (1 - percent buffer space)
  private double m_minPosFindVolts = -2.4; // Voltage to run when finding the min position
  private double m_maxPosFindVolts = 2.4; // Voltage to run when finding the max position
  private double m_dynamicFwdVolts = 2.5; // Voltage to run for the constant-voltage (aka "dynamic" test), forward direction
  private double m_dynamicRevVolts = -2.5; // Voltage to run for the constant-voltage (aka "dynamic" test), reverse direction
  private double m_quasistaticStepTime = 0.5; // How many seconds between each step up of 1 volt

  // The robot's subsystems and commands are defined here...
  private final ShooterSub m_shooterSub = new ShooterSub();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putString("Test Current", m_currentTest);
    SmartDashboard.putNumber("Test Min Pos", m_minPosition);
    SmartDashboard.putNumber("Test Max Pos", m_maxPosition);
    SmartDashboard.putNumber("Test % Margin", m_percentMargin);
    SmartDashboard.putNumber("Test Min Test Pos", m_minTestPosition);
    SmartDashboard.putNumber("Test Max Test Pos", m_maxTestPosition);
    SmartDashboard.putNumber("Test Min Pos Find Volts", m_minPosFindVolts);
    SmartDashboard.putNumber("Test Max Pos Find Volts", m_maxPosFindVolts);
    SmartDashboard.putNumber("Test Dyn Fwd Volts", m_dynamicFwdVolts);
    SmartDashboard.putNumber("Test Dyn Rev Volts", m_dynamicRevVolts);
    SmartDashboard.putNumber("Test QStatic Step Time", m_quasistaticStepTime);

    SmartDashboard.putNumber("kS", 0.0);
    SmartDashboard.putNumber("kV", 0.0);
    SmartDashboard.putNumber("kP", 0.0);
    SmartDashboard.putNumber("kI", 0.0);
    SmartDashboard.putNumber("kD", 0.0);
    SmartDashboard.putNumber("kG", 0.0);

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
    // Run mechanism backwards until at lower limit switch (or cancelled) to reset the encoder
    m_driverController.back().whileTrue(
        new InstantCommand(() -> {
          m_currentTest = "Min Find";
          m_shooterSub.setYawVoltage(m_minPosFindVolts);
        }, m_shooterSub)
            .andThen(new WaitUntilCommand(() -> m_shooterSub.isAtYawAtCWLimit()))
            .finallyDo(interrupted -> {
              m_shooterSub.setYawVoltage(0.0);
              m_minPosition = m_shooterSub.getYawAngleDeg();
              SmartDashboard.putNumber("Test Min Pos", m_minPosition);
              m_minTestPosition = m_minPosition * m_percentMargin; // Stop tests X% from end
              SmartDashboard.putNumber("Test Min Test Pos", m_minTestPosition);
              m_currentTest = "None";
            }));

    // Run mechanism forwards until at upper limit switch (or cancelled) to get the max encoder value
    m_driverController.start().whileTrue(
        new InstantCommand(() -> {
          m_currentTest = "Max Find";
          m_shooterSub.setYawPower(m_maxPosFindVolts);
        }, m_shooterSub)
            .andThen(new WaitUntilCommand(() -> m_shooterSub.isAtYawAtCCWLimit()))
            .finallyDo(interrupted -> {
              m_shooterSub.setYawVoltage(0.0);
              m_maxPosition = m_shooterSub.getYawAngleDeg();
              SmartDashboard.putNumber("Test Max Pos", m_maxPosition);
              m_maxTestPosition = m_maxPosition * (1 - m_percentMargin); // Stop tests X% from end
              SmartDashboard.putNumber("Test Max Test Pos", m_maxTestPosition);
            }));

    // Sets the voltage and checks how the mechanism responsd (forwards)
    m_driverController.a().whileTrue(
        new InstantCommand(() -> {
          m_currentTest = "dynamic-forward";
          m_shooterSub.setYawPower(m_dynamicFwdVolts);
        }, m_shooterSub)
            .andThen(new WaitUntilCommand(() -> (m_shooterSub.getYawAngleDeg() >= m_maxTestPosition) ? true : false))
            .finallyDo(interrupted -> {
              m_shooterSub.setYawVoltage(0.0);
              m_currentTest = "None";
            }));

    // Sets the voltage and checks how the mechanism responsd (forwards)
    m_driverController.b().whileTrue(
        new InstantCommand(() -> {
          m_currentTest = "dynamic-reverse";
          m_shooterSub.setYawPower(m_dynamicRevVolts);
        }, m_shooterSub)
            .andThen(new WaitUntilCommand(() -> (m_shooterSub.getYawAngleDeg() <= m_minTestPosition) ? true : false))
            .finallyDo(interrupted -> {
              m_shooterSub.setYawVoltage(0.0);
              m_currentTest = "None";
            }));

    // Increase the voltage at a set interval to see how the mechanism responds
    m_driverController.x().whileTrue(
        new InstantCommand(() -> {
          m_currentTest = "quasistatic-foward";
        })
            .andThen(new QuasistaticYawCmd(true, m_quasistaticStepTime, m_maxTestPosition, m_shooterSub))
            .finallyDo(interrupted -> {
              m_currentTest = "None";
            }));

    // Deccrease the voltage at a set interval to see how the mechanism responds
    m_driverController.y().whileTrue(
        new InstantCommand(() -> {
          m_currentTest = "quasistatic-reverse";
        })
            .andThen(new QuasistaticYawCmd(false, m_quasistaticStepTime, m_minTestPosition, m_shooterSub))
            .finallyDo(interrupted -> {
              m_currentTest = "None";
            }));

    // Get any new user values for tests from dashboard
    m_driverController.leftBumper().onTrue(new InstantCommand(() -> {
      m_minPosition = SmartDashboard.getNumber("Test Min Pos", m_minPosition);
      m_maxPosition = SmartDashboard.getNumber("Test Max Pos", m_maxPosition);
      m_percentMargin = SmartDashboard.getNumber("Test % Margin", m_percentMargin);
      m_minTestPosition = SmartDashboard.getNumber("Test Min Test Pos", m_minTestPosition);
      m_maxTestPosition = SmartDashboard.getNumber("Test Max Test Pos", m_maxTestPosition);
      m_minPosFindVolts = SmartDashboard.getNumber("Test Min Pos Find Volts", m_minPosFindVolts);
      m_maxPosFindVolts = SmartDashboard.getNumber("Test Max Pos Find Volts", m_maxPosFindVolts);
      m_dynamicFwdVolts = SmartDashboard.getNumber("Test Dyn Fwd Volts", m_dynamicFwdVolts);
      m_dynamicRevVolts = SmartDashboard.getNumber("Test Dyn Rev Volts", m_dynamicRevVolts);
      m_quasistaticStepTime = SmartDashboard.getNumber("Test QStatic Step Time", m_quasistaticStepTime);
    }));

    // Get any new control values from dashboard
    m_driverController.rightBumper().onTrue(new InstantCommand(() -> m_shooterSub.setYawTuningConstants(
        SmartDashboard.getNumber("kS", 0.0),
        SmartDashboard.getNumber("kV", 0.0),
        SmartDashboard.getNumber("kP", 0.0),
        SmartDashboard.getNumber("kI", 0.0),
        SmartDashboard.getNumber("kD", 0.0)), m_shooterSub));

    // Set some positions for the mechanism to go to
    // Min position
    m_driverController.povDown()
        .onTrue(new InstantCommand(() -> m_shooterSub.setTargetYawAngle(m_minTestPosition), m_shooterSub));
    // Max position
    m_driverController.povUp()
        .onTrue(new InstantCommand(() -> m_shooterSub.setTargetYawAngle(m_maxTestPosition), m_shooterSub));
    // 1/4 position
    m_driverController.povLeft()
        .onTrue(new InstantCommand(
            () -> m_shooterSub.setTargetYawAngle((m_maxTestPosition - m_minTestPosition) * 0.25 + m_minTestPosition),
            m_shooterSub));
    // 1/2 position
    m_driverController.povRight()
        .onTrue(new InstantCommand(
            () -> m_shooterSub.setTargetYawAngle((m_maxTestPosition - m_minTestPosition) * 0.5 + m_minTestPosition),
            m_shooterSub));

    // Stop commands
    m_driverController.leftStick().onTrue(new InstantCommand(() -> {
      m_shooterSub.setTargetYawAngle(m_shooterSub.getYawAngleDeg());
      m_shooterSub.setYawVoltage(0.0);
    }, m_shooterSub));

    m_driverController.rightStick().onTrue(new InstantCommand(() -> {
      m_shooterSub.setTargetYawAngle(m_shooterSub.getYawAngleDeg());
      m_shooterSub.setYawVoltage(0.0);
    }, m_shooterSub));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PrintCommand("No autos");
  }

  public void periodic() {
    //SmartDashboard.putNumber();
    SmartDashboard.putString("Test Current", m_currentTest);
  }
}
