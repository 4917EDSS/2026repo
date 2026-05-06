// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import java.time.Instant;
import java.time.temporal.ChronoUnit;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AimCmd;
import frc.robot.commands.HitLimitSwitchesCmd;
import frc.robot.commands.HoldIntakeCmd;
import frc.robot.commands.HoldShooterCmd;
import frc.robot.commands.IntakeBumpLiftCmd;
import frc.robot.commands.IntakeSetPositionCmd;
import frc.robot.commands.KillAllCmd;
import frc.robot.commands.RunIntakeAgitation;
import frc.robot.commands.ShootCmd;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CanSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.HopperSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.VisionSub;
import frc.robot.utils.RobotStatus;
import frc.robot.utils.ShooterAimingCalcs;


public class RobotContainer {
  // Power Distribution access
  PowerDistribution m_pd = new PowerDistribution();

  // Swerve variables
  private double m_maxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double m_maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
      .withDeadband(m_maxSpeed * 0.01).withRotationalDeadband(m_maxAngularRate * 0.05) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors

  // Controllers
  private final CommandXboxController m_driverController =
      new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(Constants.OperatorConstants.kOperatorControllerPort);

  private Instant m_stopVibratingTime = null;

  // The robot's subsystems and commands are defined here
  public final CanSub m_canSub = new CanSub(1); // TODO:  Can we remove this?
  public final DrivetrainSub m_drivetrainSub = TunerConstants.createDrivetrain();
  public final HopperSub m_hopperSub = new HopperSub(m_canSub);
  public final IntakeSub m_intakeSub = new IntakeSub();
  public final VisionSub m_visionSub = new VisionSub(m_drivetrainSub);

  public final ShooterAimingCalcs m_shooterAimingCalcs = new ShooterAimingCalcs();

  public final ShooterSub m_shooterSub = new ShooterSub(m_shooterAimingCalcs, m_drivetrainSub);

  private SendableChooser<Command> m_Chooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
    registerNameCommand();
    autoChooserSetup();
    SmartDashboard.putNumber("kS", 0.0);
    SmartDashboard.putNumber("kV", 0.0);
    SmartDashboard.putNumber("kP", 0.0);
    SmartDashboard.putNumber("kI", 0.0);
    SmartDashboard.putNumber("kD", 0.0);


    // Note that X (coordinate) is defined as forward according to WPILib convention,
    // and Y (coordinate) is defined as to the left according to WPILib convention.
    m_drivetrainSub.setDefaultCommand(
        // Drivetrain will execute this command periodically
        m_drivetrainSub.applyRequest(() -> m_drive
            .withVelocityX(
                -MathUtil.clamp((m_driverController.getLeftY() * Math.abs(m_driverController.getLeftY()) * m_maxSpeed),
                    -RobotStatus.slowDriveClamp(), RobotStatus.slowDriveClamp())) // Drive forward with negative Y (forward)
            .withVelocityY(
                -MathUtil.clamp((m_driverController.getLeftX() * Math.abs(m_driverController.getLeftX()) * m_maxSpeed),
                    -RobotStatus.slowDriveClamp(), RobotStatus.slowDriveClamp())) // Drive left with negative X (left)
            .withRotationalRate(-m_driverController.getRightX() * m_maxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    m_shooterSub
        .setDefaultCommand(new AimCmd(m_shooterSub, m_shooterAimingCalcs, m_drivetrainSub, m_intakeSub, m_visionSub));

    m_intakeSub.setDefaultCommand(new HoldIntakeCmd(m_intakeSub));
  }


  private void registerNameCommand() {
    // TODO: Add commands that PathPlanner needs access to here
    NamedCommands.registerCommand("IntakeDeployCmd",
        new IntakeSetPositionCmd(Constants.Intake.kDeployOutAngleDeg, Constants.Intake.kDeployDeployVoltage,
            m_intakeSub));

    NamedCommands.registerCommand("IntakeRetractCmd",
        new ParallelCommandGroup(
            new IntakeSetPositionCmd(Constants.Intake.kDeployInAngleDeg, Constants.Intake.kDeployRetractVoltage,
                m_intakeSub),
            new HoldShooterCmd(m_shooterSub)));

    NamedCommands.registerCommand("AimCmd",
        (new AimCmd(m_shooterSub, m_shooterAimingCalcs, m_drivetrainSub, m_intakeSub, m_visionSub)));

    NamedCommands.registerCommand("HitLimitSwitchesCmd", new HitLimitSwitchesCmd(m_shooterSub));

    NamedCommands.registerCommand("ShootCmd", new ShootCmd(m_hopperSub, m_intakeSub, m_shooterSub));

    NamedCommands.registerCommand("IntakeBumpLiftCmd", new IntakeBumpLiftCmd(m_intakeSub));

    NamedCommands.registerCommand("RunIntakeAgitation", new RunIntakeAgitation(m_intakeSub, m_driverController));
  }

  /*
   * Use this method to define your trigger->command mappings.
   */
  private void configureBindings() {
    ////////////////////////////// Driver Buttons //////////////////////////////
    // Driver A
    m_driverController.a()
        .toggleOnTrue(new RunIntakeAgitation(m_intakeSub, m_driverController));//.onTrue(new IntakeBumpLiftCmd(m_intakeSub));

    // Driver B
    m_driverController.b()
        .onTrue(
            new IntakeSetPositionCmd(Constants.Intake.kDeployInAngleDeg, Constants.Intake.kDeployRetractVoltage,
                m_intakeSub));

    // Driver X
    m_driverController.x().onTrue(new HitLimitSwitchesCmd(m_shooterSub));

    // Driver Y
    m_driverController.y().whileTrue(new StartEndCommand(() -> {
      m_shooterSub.setTargetYawAngle(180.0);
      m_shooterSub.setTargetPitchAngle(24.0);
      m_shooterSub.setTargetFlywheelVelocity(66.0);
    }, () -> m_shooterSub.getYawAngleDeg(), m_shooterSub));
    // m_driverController.y().onTrue(
    //     new InstantCommand(() -> m_shooterSub.setTargetPitchAngle(m_shooterAimingCalcs
    //         .getInterpolatedPitchAngle(m_shooterAimingCalcs.getDistanceToHub(m_drivetrainSub.getPose()))))
    //             .andThen(new InstantCommand(
    //                 () -> m_shooterSub.setTargetFlywheelVelocity(m_shooterAimingCalcs.getInterpolatedFlywheelVelocity(
    //                     m_shooterAimingCalcs.getDistanceToHub(m_drivetrainSub.getPose()))))));

    // Driver Left Bumper
    m_driverController.leftBumper().onTrue(new InstantCommand(() -> m_intakeSub.setBeltVoltage(0.0), m_intakeSub));

    // Driver Right Bumper
    m_driverController.rightBumper().onTrue(new InstantCommand(() -> {
      m_hopperSub.disableEscalatorAutomation();
      m_hopperSub.setSingulatorVoltage(0.0);
      m_shooterSub.disableFlywheelAutomation();
    }, m_shooterSub, m_hopperSub));

    // Driver Left Trigger
    m_driverController.leftTrigger()
        .onTrue(new IntakeSetPositionCmd(Constants.Intake.kDeployOutAngleDeg, 2.0, m_intakeSub));

    // Driver Right Trigger
    m_driverController.rightTrigger().onTrue(new ParallelCommandGroup(
        new ShootCmd(m_hopperSub, m_intakeSub, m_shooterSub), new RunIntakeAgitation(m_intakeSub, m_driverController)));

    // Driver Back
    m_driverController.back().onTrue(m_drivetrainSub.runOnce(m_drivetrainSub::seedFieldCentric)); // Reset the field-centric heading 

    // Driver Start
    m_driverController.start()
        .onTrue(new InstantCommand(() -> m_drivetrainSub.resetPose((m_visionSub.getEstimatedPose()))));

    // Driver POV Up
    m_driverController.povUp().onTrue(new InstantCommand(
        () -> m_shooterSub.setTargetFlywheelVelocity(SmartDashboard.getNumber("Set Sht Fly Vel Rps", 0.0))));

    // m_driverController.povUp().whileTrue(new StartEndCommand(() -> {
    //   m_shooterSub.setTargetYawAngle(180.0);
    //   m_shooterSub.setTargetPitchAngle(24.0);
    //   m_shooterSub.setTargetFlywheelVelocity(66.0);
    // }, () -> m_shooterSub.getYawAngleDeg(), m_shooterSub)); // The end of the command is dumb so that we still require the shooterSub

    // Driver POV Right
    //m_driverController.povRight().;

    // Driver POV Down
    m_driverController.povDown().onTrue(
        new InstantCommand(() -> m_shooterSub.setTargetPitchAngle(SmartDashboard.getNumber("Set Ptc Pos", 0.0))));

    // Driver POV Left


    // Driver Left Stick
    m_driverController.leftStick()
        .onTrue(new KillAllCmd(m_canSub, m_drivetrainSub, m_hopperSub, m_intakeSub, m_shooterSub));

    // Driver Right Stick
    m_driverController.rightStick()
        .onTrue(new KillAllCmd(m_canSub, m_drivetrainSub, m_hopperSub, m_intakeSub, m_shooterSub));


    ////////////////////////////// Operator Buttons //////////////////////////////
    // Operator A
    m_operatorController.a()
        .whileTrue(new InstantCommand(() -> m_intakeSub.disableDeployAutomation())
            .andThen(new StartEndCommand(() -> m_intakeSub.setDeployVoltage(2.0),
                () -> m_intakeSub.setDeployVoltage(0.0), m_intakeSub)));

    // Operator B
    m_operatorController.b()
        .whileTrue(new InstantCommand(() -> m_intakeSub.disableDeployAutomation())
            .andThen(new StartEndCommand(() -> m_intakeSub.setDeployVoltage(-2.0),
                () -> m_intakeSub.setDeployVoltage(0.0), m_intakeSub)));

    // Operator X
    m_operatorController.x().whileTrue(
        new StartEndCommand(() -> m_intakeSub.setBeltVoltage(Constants.Intake.kBeltIntakeVoltage),
            () -> m_intakeSub.setBeltVoltage(0.0), m_intakeSub));

    // Operator Y
    m_operatorController.y().whileTrue(
        new StartEndCommand(() -> m_intakeSub.setBeltVoltage(-4.0), () -> m_intakeSub.setBeltVoltage(0.0),
            m_intakeSub));

    // Operator Left Bumper
    m_operatorController.leftBumper().whileTrue(new StartEndCommand(() -> m_hopperSub.setSingulatorVoltage(-10.0),
        () -> m_hopperSub.setSingulatorVoltage(0.0), m_hopperSub));

    // Operator Right Bumper
    m_operatorController.rightBumper().whileTrue(new StartEndCommand(() -> m_hopperSub.setSingulatorVoltage(10.0),
        () -> m_hopperSub.setSingulatorVoltage(0.0), m_hopperSub));

    // Operator Left Trigger
    m_operatorController.leftTrigger()
        .whileTrue(new InstantCommand(() -> m_hopperSub.disableEscalatorAutomation())
            .andThen(new StartEndCommand(() -> m_hopperSub.setEscalatorVoltage(4.0),
                () -> m_hopperSub.setEscalatorVoltage(0.0), m_hopperSub)));

    // Operator Right Trigger
    m_operatorController.rightTrigger()
        .whileTrue(new InstantCommand(() -> m_shooterSub.disableFlywheelAutomation())
            .andThen(new StartEndCommand(() -> m_shooterSub.setFlywheelVoltage(6.0),
                () -> m_shooterSub.setFlywheelVoltage(0.0), m_shooterSub)));

    // Operator Back
    m_operatorController.back().whileTrue(new InstantCommand(() -> m_shooterSub.disableYawAutomation()).andThen(
        new StartEndCommand(() -> m_shooterSub.setYawVoltage(2.0), () -> m_shooterSub.setYawVoltage(0.0),
            m_shooterSub)));

    // Operator Start
    m_operatorController.start().whileTrue(new InstantCommand(() -> m_shooterSub.disableYawAutomation()).andThen(
        new StartEndCommand(() -> m_shooterSub.setYawVoltage(-2.0), () -> m_shooterSub.setYawVoltage(0.0),
            m_shooterSub)));

    // Operator POV Up
    m_operatorController.povUp().whileTrue(new InstantCommand(() -> m_shooterSub.disablePitchAutomation()).andThen(
        new StartEndCommand(() -> m_shooterSub.setPitchVoltage(2.0), () -> m_shooterSub.setPitchVoltage(0.0),
            m_shooterSub)));

    // Operator POV Right
    m_operatorController.povRight().whileTrue(
        new StartEndCommand(() -> RobotStatus.dontCompensateForMotion(), () -> RobotStatus.doCompensateForMotion()));

    // Operator POV Down
    m_operatorController.povDown().whileTrue(new InstantCommand(() -> m_shooterSub.disablePitchAutomation()).andThen(
        new StartEndCommand(() -> m_shooterSub.setPitchVoltage(-2.0), () -> m_shooterSub.setPitchVoltage(0.0),
            m_shooterSub)));

    // Operator POV Left
    m_operatorController.povLeft().whileTrue(new StartEndCommand(() -> {
      m_shooterSub.setTargetYawAngle(180.0);
      m_shooterSub.setTargetPitchAngle(24.0);
      m_shooterSub.setTargetFlywheelVelocity(66.0);
    }, () -> m_shooterSub.getYawAngleDeg(), m_shooterSub));

    // Operator Left Stick
    m_operatorController.leftStick()
        .onTrue(new KillAllCmd(m_canSub, m_drivetrainSub, m_hopperSub, m_intakeSub, m_shooterSub));

    // Operator Right Stick
    m_operatorController.rightStick()
        .onTrue(new InstantCommand(() -> {
          m_shooterSub.disableYawAutomation();
          m_shooterSub.unsetYawEncoder();
        }));


  }

  public Command getAutonomousCommand() {
    return m_Chooser.getSelected();
  }

  /*
   * Create a list of auto period action choices+
   */
  void autoChooserSetup() {
    m_Chooser.addOption("Right, Middle, Depot", new PathPlannerAuto("Right, Middle, Depot"));
    m_Chooser.addOption("Centre and Depot Scoring Auto", new PathPlannerAuto("Centre and Depot Scoring Auto"));
    m_Chooser.addOption("UNTESTED Smooth Centre and Depot Scoring Auto",
        new PathPlannerAuto("Smooth Centre and Depot Scoring Auto"));
    m_Chooser.addOption("Just Shoot", new PathPlannerAuto("Just Shoot"));
    m_Chooser.addOption("new rs", new PathPlannerAuto("new rs"));
    m_Chooser.addOption("new ls", new PathPlannerAuto("new ls"));


    SmartDashboard.putData("Auto Choices", m_Chooser);
  }

  public void initSubsystems() {
    m_canSub.init();
    m_drivetrainSub.init();
    m_hopperSub.init();
    m_intakeSub.init();
    m_shooterSub.init();
    m_visionSub.init();
  }

  public void vibrateFeedback(Integer seconds) {
    m_stopVibratingTime = Instant.now().plus(seconds, ChronoUnit.SECONDS);
    enableVibration();
  }

  public void enableVibration() {
    m_driverController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
  }

  public void disableVibration() {
    m_driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
  }

  public Command vibrate(Integer duration) {
    //return new Command(()-> vibrateFeedback(duration));
    //return new StartEndCommand(null, null, null)
    return new InstantCommand(() -> vibrateFeedback(duration)); //Maybe wrong subsystem for drivetrain
  }

  //Vibrations for Joysticks
  public void vibrateIfNeeded() {
    if(m_stopVibratingTime != null)
      if(m_stopVibratingTime.isBefore(Instant.now())) {
        disableVibration();
      }
  }

  public void periodic() {}
}
