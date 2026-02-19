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
    public static final int kShooterFlywheelMotorL = 8;
    public static final int kShooterFlywheelMotorR = 9;
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
    public static final double kDeployToleranceMm = 2.0;
    public static final double kDeployMaxCurrent = 120.0;
    public static final double kDeployMaxPower = 1.0;

    public static final double kRotationEncoderToDegConversionFactor = 1.0;
    public static final double kRotationInitialAngleDeg = 0.0;
    public static final double kRotationFinalAngleDeg = 180.0;
    public static final double kRotationToleranceDeg = 2.0;
    public static final double kRotationMaxCurrent = 120.0;
    public static final double kRotationMaxPower = 1.0;
  }

  public static class Hopper {
    // TODO:  Set all the values correctly
    public static final double kSingulatorEncoderToRpsConversionFactor = 1.0; // Gearing TBD
    public static final double kSingulatorMaxVelocityRps = 1.0; // Throughput must be slower than Escalator to avoid jams
    public static final double kSingulatorVelocityToleranceRps = 0.1;
    public static final double kSingulatorMaxCurrent = 120.0;
    public static final double kSingulatorMaxPower = 1.0;
    public static final double kSingulatorKS = 0.25;
    public static final double kSingulatorKV = 0.12;
    public static final double kSingulatorKP = 0.1;
    public static final double kSingulatorKI = 0.0;
    public static final double kSingulatorKD = 0.0;

    public static final double kEscalatorEncoderToRpsConversionFactor = 1.0; // Gearing TBD
    public static final double kEscalatorMaxVelocityRps = 4.0; // Throughput must be faster than Singulator but slower than Shooter to avoid jams
    public static final double kEscalatorVelocityToleranceRps = 0.1;
    public static final double kEscalatorMaxCurrent = 120.0;
    public static final double kEscalatorMaxPower = 1.0;
    public static final double kEscalatorKS = 0.25;
    public static final double kEscalatorKV = 0.12;
    public static final double kEscalatorKP = 0.1;
    public static final double kEscalatorKI = 0.0;
    public static final double kEscalatorKD = 0.0;
  }

  public static class Intake {
    // TODO:  Set all the values correctly
    public static final double kDeployEncoderToDegConversionFactor = 1.0; // Gearing is 0.014368 (approx 1:69.5)
    public static final double kDeployInAngleDeg = 0.0;
    public static final double kDeployOutAngleDeg = 10.0;
    public static final double kDeployToleranceDeg = 1.0;
    public static final double kDeployMaxCurrent = 120.0;
    public static final double kDeployMaxPower = 0.25;
    public static final double kDeployKP = 0.1;
    public static final double kDeployKI = 0.0;
    public static final double kDeployKD = 0.0;

    public static final double kBeltMaxCurrent = 120.0;
    public static final double kBeltPower = 0.25;

    public static final double kHoldPositionPidPower = 0.001;
  }

  public static class Shooter {
    // TODO:  Set all the values correctly
    public static final double kYawEncoderToDegConversionFactor = 0.02 * 360; // Gearing is 0.01851851852
    public static final double kYawMinAngleDeg = 0.0;
    public static final double kYawMaxAngleDeg = 180.0;
    public static final double kYawMaxCurrent = 120.0;
    public static final double kYawMaxPower = 0.4;
    public static final double kYawTolerance = 2.0;
    public static final double kYawMaxVelocityDegPerSec = 180.0;
    public static final double kYawMaxAccelerationDegPerSec = 360.0;
    public static final double kYawKS = 0.25;
    public static final double kYawKV = 0.12;
    public static final double kYawKP = 0.011111111111;
    public static final double kYawKI = 0.0;
    public static final double kYawKD = 0.0;

    public static final double kPitchEncoderToDegConversionFactor = 1.0; // Gearing is 0.0616 (approx 1:16.2)
    public static final double kPitchMinAngleDeg = 0.0;
    public static final double kPitchMaxAngleDeg = 45.0;
    public static final double kPitchMaxCurrent = 120.0;
    public static final double kPitchMaxPower = 0.10;
    public static final double kPitchTolerance = 2.0;
    public static final double kPitchKS = 0.25;
    public static final double kPitchKV = 0.12;
    public static final double kPitchKP = 0.1;
    public static final double kPitchKI = 0.0;
    public static final double kPitchKD = 0.0;

    public static final double kFlywheelEncoderToRpsConversionFactor = 1.0; // Gearing is 3.41 (overdriven 3.41:1)
    public static final double kFlywheelMaxVelocityRps = 25.0;
    public static final double kFlywheelVelocityToleranceRps = 1.0;
    public static final double kFlywheelMaxCurrent = 120.0;
    public static final double kFlywheelKS = 0.25;
    public static final double kFlywheelKV = 0.12;
    public static final double kFlywheelKP = 0.1;
    public static final double kFlywheelKI = 0.0;
    public static final double kFlywheelKD = 0.0;

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
