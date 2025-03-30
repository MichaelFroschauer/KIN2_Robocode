package kin;

import robocode.AdvancedRobot;
import robocode.ScannedRobotEvent;

public class SniperBot extends AdvancedRobot {
  public void run() {

    while (true) {
      turnRadarRight(360); // spin radar
    }
  }

  public void onScannedRobot(ScannedRobotEvent e) {
    fire(2);
  }
}
