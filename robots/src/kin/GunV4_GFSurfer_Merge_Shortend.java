package kin;

import robocode.*;
import robocode.util.Utils;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.*;

public class GunV4_GFSurfer_Merge_Shortend extends AdvancedRobot {

  private static final double BULLET_POWER = 1.9;
  private static double lateralDirection;
  private static double lastEnemyVelocity;

  private static GFTMovement movement;

  public GunV4_GFSurfer_Merge_Shortend() {
    movement = new GFTMovement(this);
  }


  public void run() {

    setAdjustGunForRobotTurn(true);
    setAdjustRadarForGunTurn(true);

    while (true) {
      turnRadarRight(360);
    }
  }

  private double getInitialGuess(ScannedRobotEvent e) {

    var bulletSpeed = Rules.getBulletSpeed(BULLET_POWER);

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

    return Utils.normalRelativeAngleDegrees(delta);
  }

  static int hitCalcTotal = 0;
  static int hitGuessTotal = 0;
  static int bulletsFiredTotal = 0;
  Map<Integer, Boolean> bullets = new HashMap<Integer, Boolean>();
  Map<String, EnemyStats> enemys = new HashMap<String, EnemyStats>();

  public void onScannedRobot(ScannedRobotEvent e) {

    // Lock Radar on target
    //setTurnRadarRight(2.0 * Utils.normalRelativeAngleDegrees(getHeading() + e.getBearing() - getRadarHeading()));


    double enemyAbsoluteBearing = getHeadingRadians() + e.getBearingRadians();
    double enemyDistance = e.getDistance();
    double enemyVelocity = e.getVelocity();
    if (enemyVelocity != 0) {
      lateralDirection = GFTUtils2.sign(enemyVelocity * Math.sin(e.getHeadingRadians() - enemyAbsoluteBearing));
    }
    GFTWave2 wave = new GFTWave2(this);
    wave.gunLocation = new Point2D.Double(getX(), getY());
    GFTWave2.targetLocation = GFTUtils2.project(wave.gunLocation, enemyAbsoluteBearing, enemyDistance);
    wave.lateralDirection = lateralDirection;
    wave.bulletPower = BULLET_POWER;
    wave.setSegmentations(enemyDistance, enemyVelocity, lastEnemyVelocity);
    lastEnemyVelocity = enemyVelocity;
    wave.bearing = enemyAbsoluteBearing;

    double gunAdjust;

    var enemy = enemys.get(e.getName());
    if (enemy == null) {
      enemy = new EnemyStats(e.getName());
      enemys.put(e.getName(), enemy);
    }

    boolean aimWithGuess = enemy.shouldFireWithGuess();
    if (aimWithGuess) {
      gunAdjust = Utils.normalRelativeAngle(enemyAbsoluteBearing - getGunHeadingRadians() + wave.mostVisitedBearingOffset());
      setTurnGunRightRadians(gunAdjust);
      //System.out.println("Bin enemyDistance: " + enemyDistance + " enemyVelocity: " + enemyVelocity + " lastEnemyVelocity: " + lastEnemyVelocity);
    } else {
      gunAdjust = getInitialGuess(e);
      setTurnGunRight(gunAdjust);
      //System.out.println("Init enemyDistance: " + enemyDistance + " enemyVelocity: " + enemyVelocity + " lastEnemyVelocity: " + lastEnemyVelocity);
    }

    Bullet bullet;
    if (getGunHeat() == 0 && gunAdjust < Math.atan2(9, e.getDistance()) && (bullet = setFireBullet(BULLET_POWER)) != null) {
      bullets.put(bullet.hashCode(), aimWithGuess);
      bulletsFiredTotal++;
      enemy.bulletsFired++;
      if (getEnergy() >= BULLET_POWER) {
        addCustomEvent(wave);
      }
    }

    //System.out.println("Bullets fired: " + bulletsFiredTotal + " hit Calc: " + hitCalcTotal + " hit Guess: " + hitGuessTotal);

    movement.onScannedRobot(e);
    setTurnRadarRightRadians(Utils.normalRelativeAngle(enemyAbsoluteBearing - getRadarHeadingRadians()) * 2);
  }

  public void onBulletHit(BulletHitEvent event) {
    var bulletHitWithGuess = bullets.get(event.getBullet().hashCode());
    var enemy = enemys.get(event.getName());
    if (enemy != null) {
      if (bulletHitWithGuess) {
        System.out.println(enemy.toString() + " - hit with guess");
        enemy.hitGuess++;
        hitGuessTotal++;
      } else {
        System.out.println(enemy.toString() + " - hit with calc");
        enemy.hitCalc++;
        hitCalcTotal++;
      }
    }
  }

}


class EnemyStats {
  public int hitCalc = 0;
  public int hitGuess = 0;
  public int bulletsFired = 0;
  public String name = "";

  EnemyStats(String name) {
    this.name = name;
  }

  public boolean shouldFireWithGuess() {
    Random r = new Random();
    var hitsTotal = hitCalc + hitGuess;

    if (hitsTotal <= 5) {
      return r.nextBoolean();
    }

    double ratio = getWeightedRatio();
    boolean fireWithGuess = r.nextDouble() <= ratio;
//    if (fireWithGuess) {
//      System.out.println("Ratio: " + String.format( "%.2f", ratio ) + " Fire with guess at " + name);
//    } else {
//      System.out.println("Ratio: " + String.format( "%.2f", ratio ) + " Fire with calc at " + name);
//    }
    return fireWithGuess;
  }

  private double getWeightedRatio() {
    var hitsTotal = hitCalc + hitGuess;
    return (double) (hitGuess == 0 ? 1 : hitGuess) / hitsTotal;
  }

  public String toString() {
    return "Name: " + name + " Bullets fired: " + bulletsFired
            + " hit Calc: " + hitCalc + " hit Guess: " + hitGuess
            + " ratio: " + String.format( "%.2f", getWeightedRatio() );
  }
}


class GFTWave2 extends Condition {
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
  private static final double MAX_ESCAPE_ANGLE = 0.2; // Maximum angle for predicting escape
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
  GFTWave2(AdvancedRobot _robot) {
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

  public boolean hasBufferData() {
    for (int i = 0; i < BINS; i++) {
      if(buffer[i] > 0) return true;
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


class GFTUtils2 {

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


class GFTMovement {
  private static final double BATTLE_FIELD_WIDTH = 800;
  private static final double BATTLE_FIELD_HEIGHT = 600;
  private static final double WALL_MARGIN = 18;
  private static final double MAX_TRIES = 125;
  private static final double REVERSE_TUNER = 0.421075;
  private static final double DEFAULT_EVASION = 1.2;
  private static final double WALL_BOUNCE_TUNER = 0.699484;

  private AdvancedRobot robot;
  private Rectangle2D fieldRectangle = new Rectangle2D.Double(WALL_MARGIN, WALL_MARGIN,
          BATTLE_FIELD_WIDTH - WALL_MARGIN * 2, BATTLE_FIELD_HEIGHT - WALL_MARGIN * 2);
  private double enemyFirePower = 3;
  private double direction = 0.4;

  GFTMovement(AdvancedRobot _robot) {
    this.robot = _robot;
  }

  public void onScannedRobot(ScannedRobotEvent e) {
    double enemyAbsoluteBearing = robot.getHeadingRadians() + e.getBearingRadians();
    double enemyDistance = e.getDistance();
    Point2D robotLocation = new Point2D.Double(robot.getX(), robot.getY());
    Point2D enemyLocation = GFTUtils2.project(robotLocation, enemyAbsoluteBearing, enemyDistance);
    Point2D robotDestination;
    double tries = 0;
    while (!fieldRectangle.contains(robotDestination = GFTUtils2.project(enemyLocation, enemyAbsoluteBearing + Math.PI + direction,
            enemyDistance * (DEFAULT_EVASION - tries / 100.0))) && tries < MAX_TRIES) {
      tries++;
    }
    if ((Math.random() < (GFTUtils2.bulletVelocity(enemyFirePower) / REVERSE_TUNER) / enemyDistance ||
            tries > (enemyDistance / GFTUtils2.bulletVelocity(enemyFirePower) / WALL_BOUNCE_TUNER))) {
      direction = -direction;
    }
    // Jamougha's cool way
    double angle = GFTUtils2.absoluteBearing(robotLocation, robotDestination) - robot.getHeadingRadians();
    robot.setAhead(Math.cos(angle) * 100);
    robot.setTurnRightRadians(Math.tan(angle));
  }
}
