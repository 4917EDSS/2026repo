// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSub;
import frc.robot.subsystems.DrivetrainSub;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private double m_maxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double m_maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
      .withDeadband(m_maxSpeed * 0.01).withRotationalDeadband(m_maxAngularRate * 0.05) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors

  // The robot's subsystems and commands are defined here...
  public final ClimbSub m_climbSub = new ClimbSub();
  public final DrivetrainSub m_drivetrainSub = TunerConstants.createDrivetrain();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    SmartDashboard.putNumber("kS", 0.0);
    SmartDashboard.putNumber("kV", 0.0);
    SmartDashboard.putNumber("kP", 0.0);
    SmartDashboard.putNumber("kI", 0.0);
    SmartDashboard.putNumber("kD", 0.0);
    SmartDashboard.putNumber("kG", 0.0);

    m_drivetrainSub.setDefaultCommand(
        // Drivetrain will execute this command periodically
        m_drivetrainSub.applyRequest(() -> m_drive.withVelocityX(-m_driverController.getLeftY() * m_maxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-m_driverController.getLeftX() * m_maxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-m_driverController.getRightX() * m_maxAngularRate) // Drive counterclockwise with negative X (left)
        ));
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
    m_driverController.a().whileTrue(
        new StartEndCommand(() -> m_climbSub.setRotatePower(0.4), () -> m_climbSub.setRotatePower(0.0), m_climbSub));
    m_driverController.x().whileTrue(
        new StartEndCommand(() -> m_climbSub.setRotatePower(-0.4), () -> m_climbSub.setRotatePower(0.0), m_climbSub));

    // Driver Back
    m_driverController.back().onTrue(m_drivetrainSub.runOnce(m_drivetrainSub::seedFieldCentric)); // Reset the field-centric heading
    //m_driverController.back().onTrue(m_drivetrainSub.runOnce(() -> m_drivetrainSub.seedFieldCentric())); 

  }
}
