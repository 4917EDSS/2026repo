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
import frc.robot.subsystems.IntakeSub;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.HopperSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.VisionSub;

public class RobotContainer {
  enum defendFirst {
    YES, NO, UNKNOWN
  }

  // The robot's subsystems and commands are defined here...
  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private defendFirst m_amIDefendingFirst = defendFirst.UNKNOWN;

  private final CommandXboxController m_diverContoller =
      new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorContoller =
      new CommandXboxController(Constants.OperatorConstants.kOperatorControllerPort);
  public final ClimbSub m_climbSub = new ClimbSub();
  public final DrivetrainSub m_drivetrainSub = TunerConstants.createDrivetrain();
  public final HopperSub m_hopperSub = new HopperSub();
  public final IntakeSub m_intakeSub = new IntakeSub();
  public final ShooterSub m_shooterSub = new ShooterSub();
  public final VisionSub m_visionSub = new VisionSub(m_drivetrainSub);

  public static boolean disableShuffleboardPrint = false;
  private SendableChooser<Command> m_Chooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
    registerNameCommand();
    autoChooserSetup();
  }


  private void registerNameCommand() {

  }
  /*
   * Use this method to define your trigger->command mappings.
   */

  private void configureBindings() {

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    m_drivetrainSub.setDefaultCommand(
        // Drivetrain will execute this command periodically
        m_drivetrainSub.applyRequest(() -> drive.withVelocityX(-m_diverContoller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-m_diverContoller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-m_diverContoller.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));


    // Reset the field-centric heading on left bumper press.
    m_diverContoller.back().onTrue(m_drivetrainSub.runOnce(m_drivetrainSub::seedFieldCentric));

  }

  public Command getAutonomousCommand() {
    return m_Chooser.getSelected();
  }
  /*
   * Create a list of auto period action choices+
   */

  void autoChooserSetup() {


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

  public String canScoreNow() {
    if(m_amIDefendingFirst == defendFirst.UNKNOWN) {
      processGameData();
      if(m_amIDefendingFirst == defendFirst.UNKNOWN) {
        return "unknown";
      } else {
        Double timer = DriverStation.getMatchTime();
        if ((timer <= 130 && timer > 105) || (timer <= 80 && timer > 55)) {
          return (m_amIDefendingFirst == defendFirst.YES) ? "no" : "yes";
        } else if ((timer <= 105 && timer > 80) || (timer <= 55 && timer > 30)) {
          return (m_amIDefendingFirst == defendFirst.YES) ? "yes" : "no";
        } else {
          return "yes";
        }
      }
    }
    return "unknown";
  }

  //canScoreNow
  //chcek if enum is unknown (if unknown process) 
  //check if unknown again
  //if not unknown check match timer

  //find shift
  //if shift matches with game data and if 

  //130, 105, 80, 55, 30



  public void processGameData() {
    String data = DriverStation.getGameSpecificMessage();
    String allianceColour = "";

    Optional<Alliance> alliance = DriverStation.getAlliance();
    if(alliance.isPresent()) {
      if(alliance.get() == Alliance.Red) { // could be opposite
        allianceColour = "R";
      }
      if(alliance.get() == Alliance.Blue) {
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



}
