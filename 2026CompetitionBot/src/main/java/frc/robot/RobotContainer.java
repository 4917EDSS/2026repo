// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSub;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  enum defendFirst {
    YES, NO, UNKNOWN
  }

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final IntakeSub m_IntakeSub = new IntakeSub();

  private defendFirst m_amIDefendingFirst = defendFirst.UNKNOWN;

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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));
    m_driverController.povDown()
        .onTrue(new InstantCommand(() -> m_IntakeSub.intake()));
    m_driverController.leftBumper()
        .onTrue(new InstantCommand(() -> m_IntakeSub.pullArmUp()))
        .onFalse(new InstantCommand(() -> m_IntakeSub.pullArmDown()));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

  public String amIDefendingFirst() {
    if(m_amIDefendingFirst == defendFirst.UNKNOWN) {
      processGameData();
      if(m_amIDefendingFirst == defendFirst.UNKNOWN) {
        return "unknown";
      }
    }

    if(m_amIDefendingFirst == defendFirst.YES) {
      return "true";
    } else {
      return "false";
    }
  }

  // public String canScoreNow() {
  //   if(m_amIDefendingFirst == defendFirst.UNKNOWN) {
  //     processGameData();
  //     if(m_amIDefendingFirst == defendFirst.UNKNOWN) {
  //       return "unknown";
  //     }
  //   }

  //   if(){

  //   }


  public void processGameData() {
    String data = DriverStation.getGameSpecificMessage();
    String allianceColour = "";

    Optional<Alliance> ally = DriverStation.getAlliance();
    if(ally.isPresent()) {
      if(ally.get() == Alliance.Red) {
        allianceColour = "R";
      }
      if(ally.get() == Alliance.Blue) {
        allianceColour = "B";
      }
    } else {
      return;
    }

    if(data.length() > 0) {
      if(data.charAt(0) == allianceColour.charAt(0)) {
        m_amIDefendingFirst = defendFirst.YES;
      } else {
        m_amIDefendingFirst = defendFirst.NO;
      }
    } else {
      m_amIDefendingFirst = defendFirst.UNKNOWN;
    }
  }


  //canScoreNow
  //chcek if enum is unknown (if unknown process)
  //check if unknown again
  //if not unknown check match timer
  //find shift
  //if shift matches with game data and if 

  //130, 105, 80, 55

}
