package frc.robot.utils;

import java.util.Optional;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GameData {
  enum DefendFirst {
    YES, NO, UNKNOWN
  }

  private static DefendFirst s_amIDefendingFirst = DefendFirst.UNKNOWN;

  public static Alliance getAlliance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if(alliance.isPresent()) {
      return alliance.get();
    } else {
      return Alliance.Red;
    }
  }

  public static boolean scoringSoon() {
    if(s_amIDefendingFirst == DefendFirst.UNKNOWN) {
      processGameData();
      if(s_amIDefendingFirst == DefendFirst.UNKNOWN) {
        return false;
      }
    }
    Double timer = DriverStation.getMatchTime();
    if((timer <= 135 && timer > 130) || (timer <= 85 && timer > 80)) {
      return (s_amIDefendingFirst == DefendFirst.YES) ? false : true;
    } else if((timer <= 110 && timer > 105) || (timer <= 60 && timer > 55)) {
      return (s_amIDefendingFirst == DefendFirst.YES) ? true : false;
    } else if(timer <= 35 && timer > 30) {
      return true;
    }
    return false;
  }

  public static boolean canScoreNow() {
    if(s_amIDefendingFirst == DefendFirst.UNKNOWN) {
      processGameData();
      if(s_amIDefendingFirst == DefendFirst.UNKNOWN) {
        return true;
      }
    }
    Double timer = DriverStation.getMatchTime();
    if((timer <= 130 && timer > 105) || (timer <= 80 && timer > 55)) {
      return (s_amIDefendingFirst == DefendFirst.YES) ? false : true;
    } else if((timer <= 105 && timer > 80) || (timer <= 55 && timer > 30)) {
      return (s_amIDefendingFirst == DefendFirst.YES) ? true : false;
    } else {
      return true;
    }
  }


  public static void processGameData() {
    Alliance alliance = getAlliance();

    String allianceColour = "";
    if(alliance == Alliance.Red) { // could be opposite
      allianceColour = "R";
    }
    if(alliance == Alliance.Blue) {
      allianceColour = "B";
    }

    String data = DriverStation.getGameSpecificMessage();
    SmartDashboard.putString("Game Data", data);
    if(data.length() > 0) {
      if(data.charAt(0) == allianceColour.charAt(0)) {
        s_amIDefendingFirst = DefendFirst.YES;
      } else {
        s_amIDefendingFirst = DefendFirst.NO;
      }
    } else {
      s_amIDefendingFirst = DefendFirst.UNKNOWN;
    }
  }


}
