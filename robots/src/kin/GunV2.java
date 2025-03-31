package kin;

import robocode.*;
import robocode.util.Utils;

public class GunV2 extends AdvancedRobot {

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

    System.out.println("gunHeading: " + getGunHeading() + ", targetHeading: " + targetHeading + ", delta: " + delta);

    double gunTurn = Utils.normalRelativeAngleDegrees(targetHeading - getGunHeading());
    setTurnGunRight(gunTurn);
    setFire(bulletPower);
  }

  private double getBulletPower(double distance) {

    var myEnergy = getEnergy();

    if (lastEnemyEnergy != 0.0) {
      // https://robowiki.net/wiki/Selecting_Fire_Power
      return lastEnemyEnergy / 4;
    }

    if (distance > 200 || myEnergy < 15) {
      return 1;
    } else if (distance > 50) {
      return 2;
    }
    return 3;
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


