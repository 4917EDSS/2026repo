package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

public class GameData {
  enum DefendFirst {
    YES,
    NO,
    UNKNOWN
  }

  private static DefendFirst s_amIDefendingFirst = DefendFirst.UNKNOWN;

  public static Alliance getAlliance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get();
    } else {
      return Alliance.Red;
    }
  }

  public static Boolean canScoreNow() {
    if (s_amIDefendingFirst == DefendFirst.UNKNOWN) {
      processGameData();
      if (s_amIDefendingFirst == DefendFirst.UNKNOWN) {
        return true;
      } else {
        Double timer = DriverStation.getMatchTime();
        if ((timer <= 130 && timer > 105) || (timer <= 80 && timer > 55)) {
          return (s_amIDefendingFirst == DefendFirst.YES) ? false : true;
        } else if ((timer <= 105 && timer > 80) || (timer <= 55 && timer > 30)) {
          return (s_amIDefendingFirst == DefendFirst.YES) ? true : false;
        } else {
          return true;
        }
      }
    }
    return true;
  }

  public static void processGameData() {
    Alliance alliance = getAlliance();

    String allianceColour = "";
    if (alliance == Alliance.Red) { // could be opposite
      allianceColour = "R";
    }
    if (alliance == Alliance.Blue) {
      allianceColour = "B";
    }

    String data = DriverStation.getGameSpecificMessage();
    if (data.length() > 0) {
      if (data.charAt(0) == allianceColour.charAt(0)) {
        s_amIDefendingFirst = DefendFirst.YES;
      } else {
        s_amIDefendingFirst = DefendFirst.NO;
      }
    } else {
      s_amIDefendingFirst = DefendFirst.UNKNOWN;
    }
  }
}
