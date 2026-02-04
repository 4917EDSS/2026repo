// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

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
    public static final int kIntakeArmMotor = 7;
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
    public static final double singulatorSpeed = 0.5;
  }

  public static class Vision {
    public static final double kDistanceToCloseToDrive = 0.5;
  }
}
