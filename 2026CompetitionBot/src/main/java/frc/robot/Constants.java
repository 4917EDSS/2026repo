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
    public static final int kIntakeDeployMotor = 6;
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
    public static final int kIntakeEncoder = 1;
    public static final int kClimbInLimitSwitch = 6;
    public static final int kClimbOutLimitSwitch = 7;
    public static final int kClimbCCWLimitSwitch = 8;
    public static final int kClimbCWLimitSwitch = 9;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }


  //////////////////// These are the subsystem constants ////////////////////
  public static class Hopper {
    // TODO:  Set all the values correctly
    public static final double kSingulatorEncoderToRpsConversionFactor = 3.0; // Gearing TBD
    public static final double kSingulatorMaxVelocityRps = 20.0; // Throughput must be slower than Escalator to avoid jams
    public static final double kSinglatorRpsToMpsConversionFactor = 0.63403070014 * 0.6; //  // 4 inches diamater to metters / 2
    public static final double kSingulatorVelocityToleranceRotPerSec = 0.1;
    public static final double kSingulatorMaxCurrent = 60.0;
    public static final double kSingulatorMaxVoltage = 10.0;
    public static final double kSingulatorFeedVoltage = 7.2;
    public static final double kSingulatorKS = 0.25;
    public static final double kSingulatorKV = 0.04;
    public static final double kSingulatorKP = 0.5;
    public static final double kSingulatorKI = 0.0;
    public static final double kSingulatorKD = 0.0;

    public static final double kEscalatorEncoderToRpsConversionFactor = 3.0;
    public static final double kEscalatorFeedSpeedRps = 30;
    public static final double kEscalatorMaxVelocityRotPerSec = 31.0; // Throughput must be faster than Singulator but slower than Shooter to avoid jams
    public static final double kInputEscalatorRpsToMpsConversionFactor = 0.1795 / 2; //  inches diamater to meters / 2
    public static final double kOutputEscalatorRpsToMpsConversionFactor = 0.1795 * 5 / 3; //  inches diamater to meters / 2
    public static final double kEscalatorVelocityToleranceRotPerSec = 2;
    public static final double kEscalatorMaxCurrent = 60.0;
    public static final double kEscalatorMaxVoltage = 12.0;
    public static final double kEscalatorKS = 0.37;
    public static final double kEscalatorKV = 0.36; // TODO: Test this value
    public static final double kEscalatorKP = 0.4;
    public static final double kEscalatorKI = 0.0; // TODO: Add KI
    public static final double kEscalatorKD = 0.0;
  }

  public static class Intake {
    // TODO:  Set all the values correctly
    public static final double kDeployEncoderToDegConversionFactor = 2.72244; // Gearing is 0.014368 (approx 1:69.5)
    public static final double kDeployInAngleDeg = -110.0;
    public static final double kDeployBumpAngleDeg = -45.0; //it will never get to this angle (this is on purpose)
    public static final double kDeployInToleranceDeg = 40.0;
    public static final double kDeployShakeMin = -20.0;
    public static final double kDeployOutAngleDeg = 1.0;
    public static final double kDeployNearlyMax = 7.0;
    public static final double kDeployMaxCurrent = 60.0;
    public static final double kDeployMaxVoltage = 6.0;
    public static final double kDeployRetractVoltage = 2.0; //voltage for retracting intake
    public static final double kDeployDeployVoltage = 2.0; //voltage for deploying intake
    public static final double kDeployKS = 0.8;
    public static final double kDeployKG = 0.0;
    public static final double kDeployKP = 0.028;
    public static final double kDeployKI = 0.0;
    public static final double kDeployKD = 0.0;

    public static final double kBeltMaxCurrent = 60.0;
    public static final double kBeltIntakeVoltage = 12.0;
    public static final double kBeltAgitationVoltage = 4.0;
  }

  public static class Shooter {
    // TODO:  Set all the values correctly
    public static final double kYawEncoderToDegConversionFactor = 180.0 / 2.2429;
    public static final double kYawRelativeEncoderConversion = 360.0 / 25.0 / 4.5;
    //(360.0 / kYawEncoderToDegConversionFactor) / (25.0);
    public static final double kYawEncoderOffset = 0.4496492;
    public static final double kYawMaxCurrent = 40.0;
    public static final double kYawMaxVoltage = 9.0;
    public static final double kYawTolerance = 1.0;
    public static final double kYawMaxVelocityDegPerSec = 30.0;
    public static final double kYawMaxAccelerationDegPerSec = 36.0;
    public static final double kYawDeadzoneMin = 0.0;
    public static final double kYawDeadzoneMinWrapparound = 355.0;
    public static final double kYawDeadzoneMax = 50.0;
    public static final double kYawKS = 0.20;//0.3;
    public static final double kYawKP = 0.7;//0.6;
    public static final double kYawKI = 0.0;
    public static final double kYawKD = 0.01;

    public static final double kPitchEncoderToDegConversionFactor = 29.0 / 39.29; // Difference between min and max / motor rotations
    public static final double kPitchMinAngleDeg = 18.0;
    public static final double kPitchMaxAngleDeg = 48.0;
    public static final int kPitchMaxCurrent = 12;
    public static final double kPitchMaxVoltage = 3.6;
    public static final double kPitchTolerance = 1.0;
    public static final double kPitchMaxVelocityDegPerSec = 5.0;
    public static final double kPitchMaxAccelerationDegPerSec = 120.0;
    public static final double kPitchKS = 0.03;
    public static final double kPitchKG = 0.37;
    public static final double kPitchKV = 0.0;
    public static final double kPitchKP = 0.6;
    public static final double kPitchKI = 0.0;
    public static final double kPitchKD = 0.0;

    public static final double kFlywheelEncoderToRotsPerSecConversionFactor = 1.0 / (36.0 / 28.0 * 24.0 / 18.0); // 0.467;
    public static final double kFlywheelRotsPerSecToMpsConversionFactor = 0.3164 / 2.0; // 4 inches diamater to metters / 2
    public static final double kFlywheelMinVelocityRotsPerSec = Hopper.kEscalatorMaxVelocityRotPerSec
        * (Hopper.kInputEscalatorRpsToMpsConversionFactor / kFlywheelRotsPerSecToMpsConversionFactor);
    public static final double kFlywheelMaxVelocityRotsPerSec = 140.0;
    public static final double kFlywheelVelocityToleranceRotsPerSec = 1.0;
    public static final double kFlywheelMaxCurrent = 60.0;
    public static final double kFlywheelMaxVoltage = 12.0;
    public static final double kFlywheelTargetVelocityRotsPerSec = 70.0;
    public static final double kFlywheelKS = 0.055;
    public static final double kFlywheelKV = 0.0789;
    public static final double kFlywheelKP = 0.45;//ADD FLYWHELL PID
    public static final double kFlywheelKI = 0.1;
    public static final double kFlywheelKD = 0.0;

    public static final double kGravity = 9.80665;

    public static final double kTurretOffsetX = -0.141;
    public static final double kTurretOffsetY = 0.127;
  }

  public static class Vision {
    public static final double kDistanceTooCloseToDrive = 0.5;
    public static final double kStandardDeviation = 0.1;
    public static final double kDistanceTrustThreshold = 3.0;
    public static final double kAreaTrustThreshold = 0.02;
  }

  //////////////////// These are the other constants ////////////////////
  public static class FieldElements {
    public static final double kRedHubX = 11.856; // coordinates of the hub on the field
    public static final double kRedHubY = 4.035;
    public static final double kBlueHubX = 4.675;
    public static final double kBlueHubY = 4.035;

    public static final double kBlueNeutralZoneX = 5.208524;
    public static final double kBlueAllianceZoneX = 4.675;
    public static final double kRedNeutralZoneX = 11.304524;
    public static final double kRedAllianceZoneX = 11.856;
    public static final double kLeftSideY = 3.411728;
    public static final double kRightSideY = 4.630928;
  }

  public static class TrajectoryCalculations {
    public static final double interpolationShoot = 0.3;
    public static final double interpolationLobber = 0.5;
    public static final double piecewiseSwapCalculationDistance = 3.5;
    public static final double shooterToHubHeight = 1.0668; // 42 in, not exactly measured
    public static final double kShooterToFloorHeight = 0.762;
    public static final double kLeftLobY = 2.0;
    public static final double kRightLobY = 6.0;
    public static final double kBlueLobX = 1.5;
    public static final double kRedLobX = 13.5;
  }
}
