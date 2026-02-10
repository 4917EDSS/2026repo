// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  //////////////////// These are the hardware mapping constants ////////////////////
  public static class CanIds {
    // These are the roboRIO CAN bus IDs
    // CTRE Swerve drivetrain uses CAN IDs 1-13 on CANivore bus
    // This does not conflict with the roboRIO bus which can also use these IDs
    public static final int kClimbDeployMotor = 1;
    public static final int kClimbRotateMotor = 2;
    public static final int kHopperEscalatorMotor = 3;
    public static final int kHopperSingulatorMotor = 4;
    public static final int kIntakeBeltMotor = 5;
    public static final int kIntakeDeployMotorL = 6;
    public static final int kIntakeDeployMotorR = 7;
    public static final int kShooterFlywheelMotor1 = 8;
    public static final int kShooterFlywheelMotor2 = 9;
    public static final int kShooterPitchMotor = 10;
    public static final int kShooterYawMotor = 11;
  }

  public static class CustomBoardCanIds {
    public static final int kHopperEmptyDetection = 1;
    public static final int kHopperFullDetection = 2;
    public static final int kHopperEscalatorFuelPresent = 3;
    public static final int kShooterFuelPresent = 4;
  }

  public static class DioIds {
    public static final int kClimbInLimitSwitch = 1;
    public static final int kClimbOutLimitSwitch = 2;
    public static final int kClimbCCWLimitSwitch = 3;
    public static final int kClimbCWLimitSwitch = 4;
    public static final int kIntakeDeployInLimit = 5;
    public static final int kIntakeDeployOutLimit = 6;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }


  //////////////////// These are the subsystem constants ////////////////////
  public static class Climb {
    // TODO:  Set all the values correctly
    public static final double kDeployEncoderToMmConversionFactor = 1.0;
    public static final double kDeployInDistanceMm = 0.0;
    public static final double kDeployOutDistanceMm = 10.0;
    public static final double kDeployMaxPower = 1.0;
    public static final double kDeployTolerance = 2.0;

    public static final double kRotationEncoderToDegConversionFactor = 1.0;
    public static final double kRotationInitialAngleDeg = 0.0;
    public static final double kRotationFinalAngleDeg = 180.0;
    public static final double kRotationMaxPower = 1.0;
    public static final double kRotationTolerance = 2.0;
  }

  public static class Hopper {
    public static final double kSingulatorVelocityTolerance = 4.0;
    public static final double kMaxSingulatorVelocity = 111.1;
    public static final double kSingulatorVelocity = 11.1; //singulator velocity needs to be lower than Escalator velocity so we don't get fuel building up in certain areas
    public static final double kSingulatorTicksToDegrees = 1.0;

    public static final double kEscalatorVelocityTolerance = 4.0;
    public static final double kMaxEscalatorVelocity = 111.1;
    public static final double kEscalatorVelocity = 420.1; //Escalator velocity needs to be higher than singulator velocity so we don't get fuel building up in certain areas
    public static final double kEscalatorTicksInMeter = 6967.1;
  }

  public static class Intake {
    public static final double kRotationToDegrees = 1.0;
    public static final double kBeltPower = 0.25;
    public static final double kDeployMaxPower = 0.25;
    public static final double kDeployedAngle = 10.0;
    public static final double kInAngle = 0.0;
  }

  public static class Shooter {
    public static final double kShooterYawKS = 0.25;
    public static final double kShooterYawKV = 0.12;
    public static final double kShooterYawKP = 0.1;
    public static final double kShooterYawKI = 0.0;
    public static final double kShooterYawKD = 0.05;
    public static final double kPitchMaxPower = 10;
    public static final double kYawMaxPower = 10;
    public static final double kGravity = 9.80665;
  }

  public static class Vision {
    public static final double kDistanceTooCloseToDrive = 0.5;
  }

  //////////////////// These are the other constants ////////////////////
  public static class FieldElements {
    public static final double kRedHubX = 4.675; // coordinates of the hub on the field
    public static final double kRedHubY = 4.035;
    public static final double kBlueHubX = 11.856;
    public static final double kBlueHubY = 4.035;
  }
}
