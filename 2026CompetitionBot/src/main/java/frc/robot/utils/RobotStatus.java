// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class RobotStatus extends Command {
  public enum FieldPosition {
    LeftLob(0), RightLob(1), Shoot(2), BumpTransition(3), CenterlineTransition(4);

    private int value;

    FieldPosition(int value) {
      this.value = value;
    }

    public int getValue() {
      return value;
    }
  }

  private static FieldPosition currentFieldPosition = FieldPosition.Shoot;

  public static void LeftLob() {
    currentFieldPosition = FieldPosition.LeftLob;
    SmartDashboard.putNumber("Shooting Mode", currentFieldPosition.getValue());
  }

  public static void RightLob() {
    currentFieldPosition = FieldPosition.RightLob;
    SmartDashboard.putNumber("Shooting Mode", currentFieldPosition.getValue());
  }

  public static void Shoot() {
    currentFieldPosition = FieldPosition.Shoot;
    SmartDashboard.putNumber("Shooting Mode", currentFieldPosition.getValue());
  }

  public static void BumpTransition() {
    currentFieldPosition = FieldPosition.BumpTransition;
    SmartDashboard.putNumber("Shooting Mode", currentFieldPosition.getValue());
  }

  public static void CenterlineTransition() {
    currentFieldPosition = FieldPosition.CenterlineTransition;
    SmartDashboard.putNumber("Shooting Mode", currentFieldPosition.getValue());
  }

  public static String getCurrentFieldPosition() {
    if(currentFieldPosition == FieldPosition.LeftLob) {
      return "LeftLob";
    } else if(currentFieldPosition == FieldPosition.RightLob) {
      return "RightLob";
    } else if(currentFieldPosition == FieldPosition.Shoot) {
      return "Shoot";
    } else if(currentFieldPosition == FieldPosition.BumpTransition) {
      return "BumpTransition";
    } else if(currentFieldPosition == FieldPosition.CenterlineTransition) {
      return "CenterlineTransition";
    }
    return "";
  }

  public static boolean isLobbing() {
    return(currentFieldPosition == FieldPosition.LeftLob || currentFieldPosition == FieldPosition.RightLob);
 }

  public static boolean isShooting() {
    return(currentFieldPosition == FieldPosition.Shoot);
  }


}