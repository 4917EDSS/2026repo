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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ClimbCmd;
import frc.robot.commands.DriveToPoseCmd;
import frc.robot.commands.IntakeDeployCmd;
import frc.robot.commands.KillAllCmd;
import frc.robot.commands.SpinSingulatorCmd;
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


  // The robot's subsystems and commands are defined here...
  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors


  private final CommandXboxController m_driverController =
      new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorContoller =
      new CommandXboxController(Constants.OperatorConstants.kOperatorControllerPort);
  public final ClimbSub m_climbSub = new ClimbSub();
  public final CanSub m_CanSub = new CanSub();
  public final DrivetrainSub m_drivetrainSub = TunerConstants.createDrivetrain();
  public final HopperSub m_hopperSub = new HopperSub(m_CanSub);
  public final IntakeSub m_intakeSub = new IntakeSub();
  public final ShooterSub m_shooterSub = new ShooterSub();
  public final VisionSub m_visionSub = new VisionSub(m_drivetrainSub);
  public final FeedbackSub m_FeedbackSub = new FeedbackSub(m_driverController);
  public final CalculateShooterAiming m_calculateShooterAiming = new CalculateShooterAiming();

  public static boolean disableShuffleboardPrint = false;
  private SendableChooser<Command> m_Chooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
    registerNameCommand();
    autoChooserSetup();

   m_shooterSub.setDefaultCommand(new RunCommand(
    () -> m_shooterSub.setYawPower(-m_operatorContoller.getLeftX() *0.15), m_shooterSub));

    m_shooterSub.setDefaultCommand(new RunCommand(
    () -> m_shooterSub.setPitchPower(-m_operatorContoller.getRightX() *0.15), m_shooterSub));
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
        m_drivetrainSub.applyRequest(() -> drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));


    // Reset the field-centric heading on left bumper press.
    m_driverController.back().onTrue(m_drivetrainSub.runOnce(m_drivetrainSub::seedFieldCentric));

    // m_driverController.a()
    //  .whileTrue(new StartEndCommand(() -> m_intakeSub.setIntakePower(1.0), () -> m_intakeSub.setIntakePower(0.0)));

    m_driverController.x()
        .whileTrue(
            new DriveToPoseCmd(new Pose2d(new Translation2d(15.23, 5.26), new Rotation2d(Math.toRadians(90.0))),
                m_drivetrainSub));

    //A
    m_driverController.a().onTrue(new ConditionalCommand(new InstantCommand(() -> m_calculateShooterAiming.setLobbingMode(false)), new InstantCommand(() -> m_calculateShooterAiming.setLobbingMode(true)), null));

    m_driverController.start()
        .onTrue(new InstantCommand(() -> m_drivetrainSub.resetPose((m_visionSub.getEstimatedPose()))));

    //Left Trigger
    m_driverController.leftTrigger().whileTrue(new IntakeDeployCmd(m_hopperSub, m_intakeSub));
    
    //Right Trigger
    m_driverController.rightTrigger()
        .whileTrue(new SpinSingulatorCmd(m_hopperSub));
    //Right Bumper  
    m_driverController.rightBumper().onTrue(new ClimbCmd(m_climbSub));
   
    //Back
    m_driverController.back().onTrue(m_drivetrainSub.runOnce(() -> m_drivetrainSub.seedFieldCentric()));
    //L4
    m_driverController.leftStick().onTrue(new KillAllCmd(m_CanSub, m_climbSub, m_drivetrainSub, m_hopperSub, m_intakeSub, m_shooterSub));
    //R4
    m_driverController.rightStick().onTrue(new KillAllCmd(m_CanSub, m_climbSub, m_drivetrainSub, m_hopperSub, m_intakeSub, m_shooterSub));
    //R2
    m_operatorContoller.rightTrigger()
        .onTrue(new InstantCommand(()->m_shooterSub.setFlywheelPower(0.1))).onFalse(new InstantCommand(()->m_shooterSub.setFlywheelPower(0.0)));

    //Left Stick In
    m_operatorContoller.leftStick().onTrue(new KillAllCmd(m_CanSub, m_climbSub, m_drivetrainSub, m_hopperSub, m_intakeSub, m_shooterSub));
    //Right Stick In
    m_operatorContoller.rightStick().onTrue(new KillAllCmd(m_CanSub, m_climbSub, m_drivetrainSub, m_hopperSub, m_intakeSub, m_shooterSub));
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
