package kin;

import robocode.AdvancedRobot;
import robocode.util.Utils;

import java.awt.*;

public class RectangleRunner extends AdvancedRobot {

  private static final double MARGIN = 50; // distance from walls

  @Override
  public void run() {
    // Set colors
    setBodyColor(java.awt.Color.BLUE);
    setGunColor(java.awt.Color.BLACK);
    setRadarColor(java.awt.Color.YELLOW);

    // Move to bottom-left corner (with margin)
    goTo(MARGIN, MARGIN);

    // Keep running around the edges
    while (true) {
      moveAlongRectangle();
    }
  }

  private void moveAlongRectangle() {
    double width = getBattleFieldWidth();
    double height = getBattleFieldHeight();

    goTo(width - MARGIN, MARGIN);       // Bottom-right
    goTo(width - MARGIN, height - MARGIN); // Top-right
    goTo(MARGIN, height - MARGIN);      // Top-left
    goTo(MARGIN, MARGIN);               // Bottom-left
  }

  private void goTo(double x, double y) {
    double dx = x - getX();
    double dy = y - getY();
    double angleToTarget = Math.toDegrees(Math.atan2(dx, dy));
    double targetAngle = Utils.normalRelativeAngleDegrees(angleToTarget - getHeading());

    turnRight(targetAngle);
    ahead(Math.hypot(dx, dy));
  }
}
