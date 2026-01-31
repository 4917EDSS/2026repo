// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.security.PublicKey;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class CanIds {
    // These are the roboRIO CAN bus IDs
    // CTRE Swerve drivetrain uses CAN IDs 1-13 on CANivore bus
    // This does not conflict with the roboRIO bus which can also use these IDs
    public static final int kYawMotor = 1;
    public static final int kShooterMotor = 2;
    public static final int kPitchMotor = 3;
    public static final int kClimbMotor = 4;
    public static final int kSingulatorMotor = 5;
    public static final int kIntakeMotor = 6;
    public static final int kDeployMotor1 = 7;
    public static final int kDeployMotor2 = 8;
    public static final int kEscalatorMotor = 9;

  }

  public static class DioIds {
    public static final int kClimbInLimitSwitch = 1;
    public static final int kClimbOutLimitSwitch = 2;
    public static final int kIntakeInLimitSwitch = 3;
    public static final int kIntakeOutLimitSwitch = 4;
    public static final int kIntakeAbsoluteEncoder1 = 5;
    public static final int kIntakeAbsoluteEncoder2 = 6;
    public static final int kShooterYawAbsoluteEncoder1 = 7;
    public static final int kShooterYawAbsoluteEncoder2 = 8;
    public static final int kShooterPitchAbsoluteEncoder1 = 9;
    public static final int kShooterPitchAbsoluteEncoder2 = 10;
  }

  public static class FieldElements {
    public static final double kRedHubX = 4.675;
    public static final double kRedHubY = 4.035;
    public static final double kBlueHubX = 11.856;
    public static final double kBlueHubY = 4.035;
  }

  public static class HopperConstants {
    public static final double kSingulatorVelocityTolerance = 4.0;
    public static final double kMaxSingulatorVelocity = 111.1;
    public static final double kSingulatorVelocity = 11.1; //singulator velocity needs to be lower than Escalator velocity so we don't get fuel building up in certain areas
    public static final double kSingulatorTicksInMeter = 6767.1;

    public static final double kEscalatorVelocityTolerance = 4.0;
    public static final double kMaxEscalatorVelocity = 111.1;
    public static final double kEscalatorVelocity = 420.1; //Escalator velocity needs to be higher than singulator velocity so we don't get fuel building up in certain areas
    public static final double kEscalatorTicksInMeter = 6967.1;
  }
  public static class Vision {
    public static final double kDistanceToCloseToDrive = 0.5;
  }
  public static class IntakeConstants {
    public static final double kRotationToDegrees = 270.0 / 3;
  }
}

