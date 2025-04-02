package kin;

import robocode.*;
import robocode.util.Utils;

import java.awt.geom.Point2D;

public class GunV4_GFSurfer_Merge extends AdvancedRobot {

  public void run() {

    setAdjustGunForRobotTurn(true);
    setAdjustRadarForGunTurn(true);

    while (true) {
      turnRadarRight(360);
    }
  }

  public void onScannedRobot(ScannedRobotEvent e) {
    // Lock Radar on target
    setTurnRadarRight(2.0 * Utils.normalRelativeAngleDegrees(getHeading() + e.getBearing() - getRadarHeading()));

    double bulletPower = getBulletPower(e.getDistance());
    System.out.println("bulletPower: " + bulletPower);
    var bulletSpeed = Rules.getBulletSpeed(bulletPower);

    // Target position in relative coordinates
    double absoluteBearing = getHeadingRadians() + e.getBearingRadians();
    double targetX = e.getDistance() * Math.cos(absoluteBearing);
    double targetY = e.getDistance() * Math.sin(absoluteBearing);

    // Target velocity components
    double vtX = e.getVelocity() * Math.cos(e.getHeadingRadians());
    double vtY = e.getVelocity() * Math.sin(e.getHeadingRadians());

    // Quadratic coefficients for time
    double a = vtX * vtX + vtY * vtY - bulletSpeed * bulletSpeed;
    double b = 2 * (vtX * targetX + vtY * targetY);
    double c = targetX * targetX + targetY * targetY;

    double discriminant = b * b - 4 * a * c;

    if (discriminant < 0) {
      throw new IllegalArgumentException("No solution: Target is too fast or bullet is too slow.");
    }

    // Smallest positive time
    double t1 = (-b + Math.sqrt(discriminant)) / (2 * a);
    double t2 = (-b - Math.sqrt(discriminant)) / (2 * a);
    double timeToImpact = Math.min(t1, t2);
    if (timeToImpact < 0) timeToImpact = Math.max(t1, t2);
    if (timeToImpact < 0) {
      throw new IllegalArgumentException("No valid intercept time.");
    }

    // Future target position
    double futureX = targetX + vtX * timeToImpact;
    double futureY = targetY + vtY * timeToImpact;

    // Angle to aim at
    var targetHeading = Math.toDegrees(Math.atan2(futureY, futureX));
    if(targetHeading < 0)
      targetHeading = targetHeading + 360;

    var delta = targetHeading - getGunHeading();

    //System.out.println("gunHeading: " + getGunHeading() + ", targetHeading: " + targetHeading + ", delta: " + delta);

//    double gunTurn = Utils.normalRelativeAngleDegrees(delta);
//    setTurnGunRight(gunTurn);


    double enemyAbsoluteBearing = getHeadingRadians() + e.getBearingRadians();
    double enemyDistance = e.getDistance();
    double enemyVelocity = e.getVelocity();
    if (enemyVelocity != 0) {
      lateralDirection = kin.GFTUtils.sign(enemyVelocity * Math.sin(e.getHeadingRadians() - enemyAbsoluteBearing));
    }
    kin.GFTWave wave = new kin.GFTWave(this);
    wave.gunLocation = new Point2D.Double(getX(), getY());
    kin.GFTWave.targetLocation = kin.GFTUtils.project(wave.gunLocation, enemyAbsoluteBearing, enemyDistance);
    wave.lateralDirection = lateralDirection;
    wave.bulletPower = 1.9;
    wave.setSegmentations(enemyDistance, enemyVelocity, lastEnemyVelocity);
    lastEnemyVelocity = enemyVelocity;
    wave.bearing = enemyAbsoluteBearing;

    setTurnGunRightRadians(Utils.normalRelativeAngle(enemyAbsoluteBearing - getGunHeadingRadians() + wave.mostVisitedBearingOffset()));
    setFire(1.9);
  }

  private static double lateralDirection;
  private static double lastEnemyVelocity;

  private double getBulletPower(double distance) {
    double myEnergy = getEnergy();
    double maxBulletPower = 3.0;  // Maximaler Wert für Schaden
    double minBulletPower = 0.1;  // Verhindert Energieverschwendung

    System.out.println("distance: " + distance + ", myEnergy: " + myEnergy + ", Optimal Firepower: " + lastEnemyEnergy / 4);

    // Falls vorher getroffen, Firepower anpassen
    if (lastEnemyEnergy != 0.0 && lastEnemyEnergy < 15) {
      // https://robowiki.net/wiki/Selecting_Fire_Power
      double firePower = Math.min(lastEnemyEnergy / 4, maxBulletPower);
      return Math.max(firePower, minBulletPower);
    }

    if (myEnergy < 10) {
      return 0.5;
    }

    // Dynamische Anpassung an Distanz
    if (distance > 400) {
      return 1.0;
    } else if (distance > 200) {
      return 2.0;
    }

    return maxBulletPower;
  }


  private String lastEnemyName = "";
  private double lastEnemyEnergy = 0.0;

  public void onBulletHit(BulletHitEvent event) {
    lastEnemyName = event.getName();
    lastEnemyEnergy = event.getEnergy();
  }

  public void onRobotDeath(RobotDeathEvent event) {
    if (event.getName().equals(lastEnemyName)) {
      lastEnemyName = "";
      lastEnemyEnergy = 0.0;
    }
  }

}


class GFTWave extends Condition {
  static Point2D targetLocation;

  double bulletPower;
  Point2D gunLocation;
  double bearing;
  double lateralDirection;

  private static final double MAX_DISTANCE = 900;
  private static final int DISTANCE_INDEXES = 5;
  private static final int VELOCITY_INDEXES = 5;
  private static final int BINS = 25;
  private static final int MIDDLE_BIN = (BINS - 1) / 2;
  private static final double MAX_ESCAPE_ANGLE = 0.7;
  private static final double BIN_WIDTH = MAX_ESCAPE_ANGLE / (double)MIDDLE_BIN;

  private static int[][][][] statBuffers = new int[DISTANCE_INDEXES][VELOCITY_INDEXES][VELOCITY_INDEXES][BINS];

  private int[] buffer;
  private AdvancedRobot robot;
  private double distanceTraveled;

  GFTWave(AdvancedRobot _robot) {
    this.robot = _robot;
  }

  public boolean test() {
    advance();
    if (hasArrived()) {
      buffer[currentBin()]++;
      robot.removeCustomEvent(this);
    }
    return false;
  }

  double mostVisitedBearingOffset() {
    return (lateralDirection * BIN_WIDTH) * (mostVisitedBin() - MIDDLE_BIN);
  }

  void setSegmentations(double distance, double velocity, double lastVelocity) {
    int distanceIndex = Math.min(DISTANCE_INDEXES-1, (int)(distance / (MAX_DISTANCE / DISTANCE_INDEXES)));
    int velocityIndex = (int)Math.abs(velocity / 2);
    int lastVelocityIndex = (int)Math.abs(lastVelocity / 2);
    buffer = statBuffers[distanceIndex][velocityIndex][lastVelocityIndex];
  }

  private void advance() {
    distanceTraveled += kin.GFTUtils.bulletVelocity(bulletPower);
  }

  private boolean hasArrived() {
    return distanceTraveled > gunLocation.distance(targetLocation) - 18;
  }

  private int currentBin() {
    int bin = (int)Math.round(((Utils.normalRelativeAngle(kin.GFTUtils.absoluteBearing(gunLocation, targetLocation) - bearing)) /
            (lateralDirection * BIN_WIDTH)) + MIDDLE_BIN);
    return kin.GFTUtils.minMax(bin, 0, BINS - 1);
  }

  private int mostVisitedBin() {
    int mostVisited = MIDDLE_BIN;
    for (int i = 0; i < BINS; i++) {
      if (buffer[i] > buffer[mostVisited]) {
        mostVisited = i;
      }
    }
    return mostVisited;
  }
}

class GFTUtils {
  static double bulletVelocity(double power) {
    return 20 - 3 * power;
  }

  static Point2D project(Point2D sourceLocation, double angle, double length) {
    return new Point2D.Double(sourceLocation.getX() + Math.sin(angle) * length,
            sourceLocation.getY() + Math.cos(angle) * length);
  }

  static double absoluteBearing(Point2D source, Point2D target) {
    return Math.atan2(target.getX() - source.getX(), target.getY() - source.getY());
  }

  static int sign(double v) {
    return v < 0 ? -1 : 1;
  }

  static int minMax(int v, int min, int max) {
    return Math.max(min, Math.min(max, v));
  }
}
