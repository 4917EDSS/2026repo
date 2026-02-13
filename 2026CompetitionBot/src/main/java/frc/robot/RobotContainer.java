// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ClimbCmd;
import frc.robot.commands.DriveToPoseCmd;
import frc.robot.commands.IntakeDeployCmd;
import frc.robot.commands.KillAllCmd;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CanSub;
import frc.robot.subsystems.ClimbSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.FeedbackSub;
import frc.robot.subsystems.HopperSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.VisionSub;
import frc.robot.utils.CalculateShooterAiming;

public class RobotContainer {
  private double maxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  // Controllers
  private final CommandXboxController m_driverController =
      new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorContoller =
      new CommandXboxController(Constants.OperatorConstants.kOperatorControllerPort);

  // The robot's subsystems and commands are defined here
  public final CanSub m_canSub = new CanSub(1);
  public final ClimbSub m_climbSub = new ClimbSub();
  public final DrivetrainSub m_drivetrainSub = TunerConstants.createDrivetrain();
  public final HopperSub m_hopperSub = new HopperSub(m_canSub);
  public final IntakeSub m_intakeSub = new IntakeSub();
  public final ShooterSub m_shooterSub = new ShooterSub();
  public final VisionSub m_visionSub = new VisionSub(m_drivetrainSub);
  public final FeedbackSub m_FeedbackSub = new FeedbackSub(m_driverController);

  public final CalculateShooterAiming m_calculateShooterAiming = new CalculateShooterAiming();

  private SendableChooser<Command> m_Chooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
    registerNameCommand();
    autoChooserSetup();

    // Note that X (coordinate) is defined as forward according to WPILib convention,
    // and Y (coordinate) is defined as to the left according to WPILib convention.
    m_drivetrainSub.setDefaultCommand(
        // Drivetrain will execute this command periodically
        m_drivetrainSub.applyRequest(() -> drive.withVelocityX(-m_driverController.getLeftY() * maxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-m_driverController.getLeftX() * maxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-m_driverController.getRightX() * maxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    m_shooterSub.setDefaultCommand(new RunCommand(
        () -> m_shooterSub.setYawPower(m_operatorContoller.getLeftX() * 0.15), m_shooterSub));

    m_shooterSub.setDefaultCommand(new RunCommand(
        () -> m_shooterSub.setPitchPower(-m_operatorContoller.getRightY() * 0.15), m_shooterSub));
  }


  private void registerNameCommand() {

  }

  /*
   * Use this method to define your trigger->command mappings.
   */
  private void configureBindings() {
    // Driver A
    // m_driverController.a()
    //  .whileTrue(new StartEndCommand(() -> m_intakeSub.setIntakePower(1.0), () -> m_intakeSub.setIntakePower(0.0)));

    //A SYNTAX IS INCORRECT AND CAUSIN ERRORS IN THE CODE PLEASE FIX
    // m_driverController.a()
    //     .onTrue(new ConditionalCommand(
    //         new InstantCommand(() -> m_calculateShooterAiming.setLobbingMode(false)), 
    //         new InstantCommand(() -> m_calculateShooterAiming.setLobbingMode(true)), null));

    // Driver B

    // Driver X
    m_driverController.x()
        .whileTrue(
            new DriveToPoseCmd(new Pose2d(new Translation2d(15.23, 5.26), new Rotation2d(Math.toRadians(-90.0))),
                m_drivetrainSub));

    // Driver Y

    // Driver Left Bumper

    // Driver Right Bumper
    m_driverController.rightBumper().onTrue(new ClimbCmd(m_climbSub));

    // Driver Left Trigger
    m_driverController.leftTrigger().whileTrue(new IntakeDeployCmd(m_hopperSub, m_intakeSub));

    // Driver Right Trigger
    // TODO: To spin the Singulator, call m_hopperSub.setSingulatorTargetVelocity()    
    //m_driverController.rightTrigger().whileTrue(new SpinSingulatorCmd(m_hopperSub));

    // Driver Back
    m_driverController.back().onTrue(m_drivetrainSub.runOnce(m_drivetrainSub::seedFieldCentric)); // Reset the field-centric heading
    //m_driverController.back().onTrue(m_drivetrainSub.runOnce(() -> m_drivetrainSub.seedFieldCentric())); 

    // Driver Start
    m_driverController.start()
        .onTrue(new InstantCommand(() -> m_drivetrainSub.resetPose((m_visionSub.getEstimatedPose()))));

    // Driver POV Up

    // Driver POV Right

    // Driver POV Down

    // Driver POV Left

    // Driver Left Stick
    m_driverController.leftStick()
        .onTrue(new KillAllCmd(m_canSub, m_climbSub, m_drivetrainSub, m_hopperSub, m_intakeSub, m_shooterSub));

    // Driver Right Stick
    m_driverController.rightStick()
        .onTrue(new KillAllCmd(m_canSub, m_climbSub, m_drivetrainSub, m_hopperSub, m_intakeSub, m_shooterSub));


    // Operator A
    // TODO: Run deploy motor with low positive power while held

    // Operator B
    // TODO: Run deploy motor with low negative power while held
    m_operatorContoller.b().whileTrue(new StartEndCommand(() -> m_intakeSub.setDeployPower(-0.1),
        () -> m_intakeSub.setDeployPower(0.0), m_intakeSub));

    // Operator X
    // TODO: Run belt motor with low positive power while held

    // Operator Y
    // TODO: Run belt motor with low negative power while held

    // Operator Left Bumper
    // TODO: Run singulator motor with low negative power while held

    // Operator Right Bumper
    // TODO: Run singulator motor with low positive power while held

    // Operator Left Trigger
    // TODO: Run elevator motor with low positive power while held

    // Operator Right Trigger
    // TODO: Convert this to a StartEndCommand instead of two InstantCommands
    m_operatorContoller.rightTrigger()
        .onTrue(new InstantCommand(() -> m_shooterSub.setFlywheelPower(0.1)))
        .onFalse(new InstantCommand(() -> m_shooterSub.setFlywheelPower(0.0)));

    // Operator Back
    // TODO: Modify this to simply set a low negative power while held (i.e. don't use the alogorithm)
    //m_operatorContoller.back().whileTrue(new InstantCommand(() -> m_climbSub.setTargetRotateAngle(1, -0.1)));

    // Operator Start
    // TODO: Modify this to simply set a low positive power while held (i.e. don't use the alogorithm)
    //m_operatorContoller.start().whileTrue(new InstantCommand(() -> m_climbSub.setTargetRotateAngle(1, 0.1)));

    // Operator POV Up
    // TODO: Modify this to simply set a low positive power while held (i.e. don't use the alogorithm)
    //m_operatorContoller.povUp().whileTrue(new InstantCommand(() -> m_climbSub.setTargetDeployDistance(1, 0.1)));

    // Operator POV Right

    // Operator POV Down
    // TODO: Modify this to simply set a low negative power while held (i.e. don't use the alogorithm)
    //m_operatorContoller.povDown().whileTrue(new InstantCommand(() -> m_climbSub.setTargetDeployDistance(1, -0.1)));

    // Operator POV Left

    // Operator Left Stick
    m_operatorContoller.leftStick()
        .onTrue(new KillAllCmd(m_canSub, m_climbSub, m_drivetrainSub, m_hopperSub, m_intakeSub, m_shooterSub));

    // Operator Right Stick
    m_operatorContoller.rightStick()
        .onTrue(new KillAllCmd(m_canSub, m_climbSub, m_drivetrainSub, m_hopperSub, m_intakeSub, m_shooterSub));

  }

  public Command getAutonomousCommand() {
    return m_Chooser.getSelected();
  }

  /*
   * Create a list of auto period action choices+
   */
  void autoChooserSetup() {

  }

}
