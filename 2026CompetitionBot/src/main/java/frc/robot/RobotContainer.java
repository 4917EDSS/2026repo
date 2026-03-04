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
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveToPoseCmd;
import frc.robot.commands.IntakeDeployCmd;
import frc.robot.commands.KillAllCmd;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CanSub;
import frc.robot.subsystems.ClimbSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.HopperSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.VisionSub;
import frc.robot.utils.ShooterAimingCalcs;


public class RobotContainer {
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
  public final CanSub m_canSub = new CanSub(1);
  public final ClimbSub m_climbSub = new ClimbSub();
  public final DrivetrainSub m_drivetrainSub = TunerConstants.createDrivetrain();
  public final HopperSub m_hopperSub = new HopperSub(m_canSub);
  public final IntakeSub m_intakeSub = new IntakeSub();
  public final ShooterSub m_shooterSub = new ShooterSub();
  public final VisionSub m_visionSub = new VisionSub(m_drivetrainSub);

  public final ShooterAimingCalcs m_shooterAimingCalcs = new ShooterAimingCalcs();

  private SendableChooser<Command> m_Chooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
    registerNameCommand();
    autoChooserSetup();

    // Note that X (coordinate) is defined as forward according to WPILib convention,
    // and Y (coordinate) is defined as to the left according to WPILib convention.
    m_drivetrainSub.setDefaultCommand(
        // Drivetrain will execute this command periodically
        m_drivetrainSub.applyRequest(() -> m_drive.withVelocityX(-m_driverController.getLeftY() * m_maxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-m_driverController.getLeftX() * m_maxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-m_driverController.getRightX() * m_maxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    // m_shooterSub.setDefaultCommand(new RunCommand(
    //     () -> m_shooterSub.setPitchAndYawPower(-m_operatorController.getRightY() * 0.15,
    //         m_operatorController.getLeftX() * 0.15),
    //     m_shooterSub));

    m_shooterSub.setDefaultCommand(new RunCommand(
        () -> m_shooterSub.setPitchYawFlywheelPower(m_shooterAimingCalcs.calculationsInMotion(m_drivetrainSub.getPose(),
            m_drivetrainSub.getRobotRelativeSpeeds())),
        m_shooterSub));
  }


  private void registerNameCommand() {
    // TODO: Add commands that PathPlanner needs access to here
  }

  /*
   * Use this method to define your trigger->command mappings.
   */
  private void configureBindings() {
    // Driver A
    // m_driverController.a()
    //     .onTrue(new ConditionalCommand(
    // new InstantCommand(() -> m_calculateShooterAiming.setLobbingMode(false)),
    // new InstantCommand(() -> m_calculateShooterAiming.setLobbingMode(true)),
    // m_calculateShooterAiming.getLobbingMode()));

    // Driver B

    // // Driver X
    // m_driverController.x()
    //     .whileTrue(
    //         new DriveToPoseCmd(new Pose2d(new Translation2d(15.23, 5.26), new Rotation2d(Math.toRadians(-90.0))),
    //             m_drivetrainSub));

    // // Driver Y
    // m_driverController.y()
    //     .whileTrue(
    //         m_drivetrainSub.applyRequest(() -> m_drive.withVelocityX(9999.9).withVelocityY(0).withRotationalRate(0)));

    // Driver Left Bumper

    // Driver Right Bumper
    m_driverController.rightBumper()
        .onTrue(new InstantCommand(() -> m_climbSub.setTargetDeployDistance(Constants.Climb.kDeployOutDistanceM),
            m_climbSub)
                .andThen(new WaitUntilCommand(() -> m_climbSub.isAtDeployOutLimit()))
                .andThen(new InstantCommand(
                    () -> m_climbSub.setTargetRotateAngle(Constants.Climb.kRotationFinalAngleDeg), m_climbSub)));

    // Driver Left Trigger
    m_driverController.leftTrigger().whileTrue(new IntakeDeployCmd(m_hopperSub, m_intakeSub));

    // Driver Right Trigger
    m_driverController.rightTrigger().whileTrue(new StartEndCommand(() -> m_hopperSub.setSingulatorTargetVelocity(3),
        () -> m_hopperSub.setSingulatorTargetVelocity(0))); // Should be 9 balls per second

    // Driver Back
    m_driverController.back().onTrue(m_drivetrainSub.runOnce(m_drivetrainSub::seedFieldCentric)); // Reset the field-centric heading
    //m_driverController.back().onTrue(m_drivetrainSub.runOnce(() -> m_drivetrainSub.seedFieldCentric())); 

    // Driver Start
    m_driverController.start()
        .onTrue(new InstantCommand(() -> m_drivetrainSub.resetPose((m_visionSub.getEstimatedPose()))));

    // Driver POV Up

    // Driver POV Right
    m_driverController.povRight()
        .onTrue(new InstantCommand(() -> m_shooterSub.setTargetYawAngle(Constants.Shooter.kYawMaxAngleDeg)));
    // Driver POV Down
    m_driverController.povDown()
        .onTrue(new InstantCommand(() -> m_climbSub.setTargetDeployDistance(Constants.Climb.kDeployInDistanceM))
            .andThen(new WaitUntilCommand(() -> m_climbSub.isAtDeployInLimit()))
            .andThen(
                new InstantCommand(() -> m_climbSub.setTargetRotateAngle(Constants.Climb.kRotationInitialAngleDeg))));

    // Driver POV Left
    m_driverController.povLeft()
        .onTrue(new InstantCommand(() -> m_shooterSub.setTargetYawAngle(Constants.Shooter.kYawMaxAngleDeg)));
    // Driver Left Stick
    m_driverController.leftStick()
        .onTrue(new KillAllCmd(m_canSub, m_climbSub, m_drivetrainSub, m_hopperSub, m_intakeSub, m_shooterSub));

    // Driver Right Stick
    m_driverController.rightStick()
        .onTrue(new KillAllCmd(m_canSub, m_climbSub, m_drivetrainSub, m_hopperSub, m_intakeSub, m_shooterSub));


    // Operator A
    m_operatorController.a().whileTrue(
        new StartEndCommand(() -> m_intakeSub.setDeployPower(0.1), () -> {
          m_intakeSub.setDeployPower(0.0);
          m_intakeSub.disableDeployAutomation();
        }, m_intakeSub));

    // Operator B
    m_operatorController.b().whileTrue(new StartEndCommand(() -> m_intakeSub.setDeployPower(-0.1),
        () -> {
          m_intakeSub.setDeployPower(0.0);
          m_intakeSub.disableDeployAutomation();
        }, m_intakeSub));

    // Operator X
    m_operatorController.x().whileTrue(
        new StartEndCommand(() -> m_intakeSub.setBeltPower(0.10), () -> m_intakeSub.setBeltPower(0.0), m_intakeSub));

    // Operator Y
    m_operatorController.y()
        .whileTrue(new StartEndCommand(() -> m_intakeSub.setBeltPower(-0.1), () -> m_intakeSub.setBeltPower(0.0),
            m_intakeSub));


    // Operator Left Bumper
    m_operatorController.leftBumper().whileTrue(new StartEndCommand(() -> m_hopperSub.setSingulatorPower(-0.1),
        () -> m_hopperSub.setSingulatorPower(0.0), m_hopperSub));

    // Operator Right Bumper
    m_operatorController.rightBumper()
        .whileTrue(new StartEndCommand(() -> m_hopperSub.setSingulatorPower(0.1),
            () -> m_hopperSub.setSingulatorPower(0.0), m_hopperSub));

    // Operator Left Trigger
    m_operatorController.leftTrigger().whileTrue(new StartEndCommand(() -> m_hopperSub.setEscalatorPower(0.10),
        () -> m_hopperSub.setEscalatorPower(0.0), m_hopperSub));

    // Operator Right Trigger
    m_operatorController.rightTrigger()
        .whileTrue(new StartEndCommand(() -> m_shooterSub.setFlywheelPower(0.1),
            () -> m_shooterSub.setFlywheelPower(0.0), m_shooterSub));

    // Operator Back
    m_operatorController.back().whileTrue(
        new StartEndCommand(() -> m_climbSub.setRotatePower(0.1), () -> m_climbSub.setRotatePower(0.0), m_climbSub));

    // Operator Start
    m_operatorController.start().whileTrue(
        new StartEndCommand(() -> m_climbSub.setRotatePower(-0.1), () -> m_climbSub.setRotatePower(0.0), m_climbSub));

    // Operator POV Up
    m_operatorController.povUp().whileTrue(
        new StartEndCommand(() -> m_climbSub.setTargetDeployDistance(0.1),
            () -> m_climbSub.setTargetDeployDistance(0.0), m_climbSub));

    // Operator POV Right

    // Operator POV Down
    m_operatorController.povDown().whileTrue(
        new StartEndCommand(() -> m_climbSub.setTargetDeployDistance(0.1),
            () -> m_climbSub.setTargetDeployDistance(0.0), m_climbSub));

    // Operator POV Left

    // Operator Left Stick
    m_operatorController.leftStick()
        .onTrue(new KillAllCmd(m_canSub, m_climbSub, m_drivetrainSub, m_hopperSub, m_intakeSub, m_shooterSub));

    // Operator Right Stick
    m_operatorController.rightStick()
        .onTrue(new KillAllCmd(m_canSub, m_climbSub, m_drivetrainSub, m_hopperSub, m_intakeSub, m_shooterSub));

  }

  public Command getAutonomousCommand() {
    return m_Chooser.getSelected();
  }

  /*
   * Create a list of auto period action choices+
   */
  void autoChooserSetup() {
    m_Chooser.addOption("Straight 3m", new PathPlannerAuto("Go Straight"));
    m_Chooser.addOption("Test Auto", new PathPlannerAuto("Test Auto"));
    SmartDashboard.putData("Auto Choices", m_Chooser);
  }

  public void initSubsystems() {
    m_canSub.init();
    m_climbSub.init();
    m_drivetrainSub.init();
    m_hopperSub.init();
    m_intakeSub.init();
    m_shooterSub.init();
    m_visionSub.init();
  }

  public void vibrateFeedback(Integer seconds) {
    m_stopVibratingTime = Instant.now().plus(seconds, ChronoUnit.SECONDS);
    enableVibration();
    //Rumble requires the Driver Station, it does not work in the simulator
  }

  private void enableVibration() {
    m_driverController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
  }

  private void disableVibration() {
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
}
