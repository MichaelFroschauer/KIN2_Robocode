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
  // Static property holding the target location for the wave (where the bullet is aimed at)
  static Point2D targetLocation;

  // The power of the bullet (affects its speed)
  double bulletPower;

  // The position of the robot's gun when the bullet is fired
  Point2D gunLocation;

  // The bearing (angle) between the robot and the target when the bullet is fired
  double bearing;

  // The direction in which the target is moving (left or right)
  double lateralDirection;

  // Constants for wave simulation
  private static final double MAX_DISTANCE = 900;  // Maximum distance for the bullet
  private static final int DISTANCE_INDEXES = 5;   // Number of bins for distance categorization
  private static final int VELOCITY_INDEXES = 5;   // Number of bins for velocity categorization
  private static final int BINS = 25;              // Number of bins to track the opponent's escape angle
  private static final int MIDDLE_BIN = (BINS - 1) / 2; // Middle bin index used for normalization
  private static final double MAX_ESCAPE_ANGLE = 0.7; // Maximum angle for predicting escape
  private static final double BIN_WIDTH = MAX_ESCAPE_ANGLE / (double)MIDDLE_BIN; // Width of each bin

  // A 4-dimensional array to store statistical data based on distance, velocity, and escape angle
  private static int[][][][] statBuffers = new int[DISTANCE_INDEXES][VELOCITY_INDEXES][VELOCITY_INDEXES][BINS];

  // A reference to the statistics buffer for the current wave based on distance, velocity, and last velocity
  private int[] buffer;

  // The robot instance for interaction with the game environment
  private AdvancedRobot robot;

  // The total distance traveled by the bullet
  private double distanceTraveled;

  // Constructor for initializing the GFTWave with the robot instance
  GFTWave(AdvancedRobot _robot) {
    this.robot = _robot;
  }

  // Method to test if the wave has arrived at its target and update the statistics
  public boolean test() {
    advance();  // Move the wave forward by a small increment
    if (hasArrived()) {  // Check if the wave has arrived at the target
      buffer[currentBin()]++;  // Update the statistics for the current "bin" (escape angle)
      robot.removeCustomEvent(this);  // Remove the custom event once the wave has arrived
    }
    return false;
  }

  // Method to calculate the bearing offset where the target was most frequently encountered
  double mostVisitedBearingOffset() {
    return (lateralDirection * BIN_WIDTH) * (mostVisitedBin() - MIDDLE_BIN);
  }

  // Method to set the correct segmentation based on the opponent's distance, velocity, and last velocity
  void setSegmentations(double distance, double velocity, double lastVelocity) {
    // Compute the indexes based on distance, velocity, and last velocity
    int distanceIndex = Math.min(DISTANCE_INDEXES - 1, (int)(distance / (MAX_DISTANCE / DISTANCE_INDEXES)));
    int velocityIndex = (int)Math.abs(velocity / 2);
    int lastVelocityIndex = (int)Math.abs(lastVelocity / 2);
    // Set the buffer to the appropriate statistics segment
    buffer = statBuffers[distanceIndex][velocityIndex][lastVelocityIndex];
  }

  // Method to advance the wave's travel based on its bullet velocity
  private void advance() {
    distanceTraveled += kin.GFTUtils.bulletVelocity(bulletPower);  // Increase the distance based on bullet speed
  }

  // Method to check if the wave has arrived at the target location
  private boolean hasArrived() {
    // If the bullet has traveled enough to reach the target
    return distanceTraveled > gunLocation.distance(targetLocation) - 18;
  }

  // Method to calculate the "bin" (escape angle range) in which the wave's current bearing falls
  private int currentBin() {
    int bin = (int)Math.round(((Utils.normalRelativeAngle(kin.GFTUtils.absoluteBearing(gunLocation, targetLocation) - bearing)) /
            (lateralDirection * BIN_WIDTH)) + MIDDLE_BIN);
    // Return the bin index within valid range
    return kin.GFTUtils.minMax(bin, 0, BINS - 1);
  }

  // Method to find the most visited bin (the most likely escape angle based on historical data)
  private int mostVisitedBin() {
    int mostVisited = MIDDLE_BIN;  // Start with the middle bin
    for (int i = 0; i < BINS; i++) {
      // Compare the number of visits for each bin and select the most visited one
      if (buffer[i] > buffer[mostVisited]) {
        mostVisited = i;
      }
    }
    return mostVisited;
  }
}


class GFTUtils {

  // Method to calculate the bullet velocity based on the power
  static double bulletVelocity(double power) {
    // Bullet speed decreases with increasing power
    return 20 - 3 * power;
  }

  // Method to project a point a certain distance in a specified direction (angle)
  static Point2D project(Point2D sourceLocation, double angle, double length) {
    // Use trigonometry to calculate the new position based on angle and distance
    return new Point2D.Double(sourceLocation.getX() + Math.sin(angle) * length,
            sourceLocation.getY() + Math.cos(angle) * length);
  }

  // Method to calculate the absolute bearing (angle) between two points
  static double absoluteBearing(Point2D source, Point2D target) {
    // Calculate the angle from the source point to the target point using atan2
    return Math.atan2(target.getX() - source.getX(), target.getY() - source.getY());
  }

  // Method to return the sign of a value: -1 for negative, 1 for positive
  static int sign(double v) {
    return v < 0 ? -1 : 1;
  }

  // Method to clamp a value within a specified minimum and maximum range
  static int minMax(int v, int min, int max) {
    return Math.max(min, Math.min(max, v));
  }
}

