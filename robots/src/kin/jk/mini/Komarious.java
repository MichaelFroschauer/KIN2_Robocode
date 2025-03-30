package voidious.mini;

import robocode.*;
import robocode.util.Utils;
import java.awt.geom.*;
import java.util.LinkedList;
import java.awt.Color;
import robocode.Rules;

/**
 * Komarious - a MiniBot by Voidious
 *
 * She's a duelist, and assumes it's a 1 on 1. This is a first attempt by
 * Voidious at making a mini-WaveSurfer.
 *
 * CREDITS:
 *   Jamougha - gun based on RaikoMicro 1.44
 *   rozu - mini-sized PrecisePrediction from Apollon
 *   Iiley - small codesize BackAsFront method
 *   PEZ - iterative WallSmoothing algorithm
 *
 * Code is open source, released under the RoboWiki Public Code License:
 * http://robowiki.net/cgi-bin/robowiki?RWPCL
 */

public class Komarious extends AdvancedRobot {
    ////////////////////////////////////////////////////////
    // Shared between movement and gun
    static final int GF_ZERO = 23;
    static final int GF_ONE = 46;
    public static Point2D.Double _myLocation;
    public static Point2D.Double _enemyLocation;
    private static double _lastDistance;
    private static double _enemyAbsoluteBearing;
    ////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////
    // Movement
    static final double WALL_STICK = 140;
    static final double A_LITTLE_LESS_THAN_HALF_PI = 1.25; // Math.PI/2 would be
    // perpendicular movement, less will keep us moving away slightly
    private static double _surfStats[][][] = new double[4][5][GF_ONE+1];
    public static LinkedList _enemyWaves;
    private static double _oppEnergy;
    private static Wave _surfWave;
    private static Wave _nextSurfWave;
    private static double _lastLatVel;
    private static double _lastAbsBearingRadians;
    private static double _goAngle;
    private static java.awt.geom.Rectangle2D.Double _fieldRect
            = new java.awt.geom.Rectangle2D.Double(18, 18, 764, 564);
    ////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////
    // Gun
    static final double LOG_BASE_E_TO_2_CONVERSION_CONSTANT = 1.4427;
    static double[][][][][][] _gunStats = new double[6][4][4][2][3][GF_ONE+1];
    static double lastVChangeTime;
    static int enemyVelocity;
    private static int _ramCounter;
    private static int _lastGunOrientation;
    ////////////////////////////////////////////////////////

    public void run() {
//        setColors(Color.black, Color.black, Color.white);
        setBodyColor(Color.black);

        _enemyWaves = new LinkedList();
        setAdjustGunForRobotTurn(true);
        setAdjustRadarForGunTurn(true);

        do {
            turnRadarRightRadians(1);
        } while (true);

    }

    public void onScannedRobot(ScannedRobotEvent e) {
        Wave w;
        int direction;

        double bulletPower;
        if ((bulletPower = _oppEnergy - e.getEnergy()) <= 3
                && bulletPower > 0) {
            (w = _nextSurfWave).bulletSpeed = Rules.getBulletSpeed(bulletPower);
            addCustomEvent(w);
            _enemyWaves.addLast(w);
        }
        (_nextSurfWave = w = new Wave()).directAngle = _lastAbsBearingRadians;
        w.waveGuessFactors = _surfStats[(int)(Math.min((_lastDistance+50)/200, 3))][(int)((Math.abs(_lastLatVel)+1)/2)];
        w.orientation = direction = sign(_lastLatVel);
        double enemyAbsoluteBearing;
        w.sourceLocation = _enemyLocation =
                project((_myLocation = new Point2D.Double(getX(), getY())),
                        _enemyAbsoluteBearing = enemyAbsoluteBearing = getHeadingRadians() + e.getBearingRadians(), _lastDistance = e.getDistance());

//        _oppEnergy = e.getEnergy(); // MC (done in gun normally)

        (w = new Wave()).sourceLocation = _myLocation;
        addCustomEvent(w);
        setTurnRadarRightRadians(Utils.normalRelativeAngle((w.directAngle = enemyAbsoluteBearing) - getRadarHeadingRadians()) * 2);


        // WaveSurfing /////////////////////////////////////////////////////////
        try {
            _goAngle = (_surfWave = (Wave)(_enemyWaves.getFirst()))
                    .absoluteBearing(_myLocation) +
                    A_LITTLE_LESS_THAN_HALF_PI *
                            (direction = (sign(checkDanger(-1) - checkDanger(1))));
        } catch (Exception ex) { }

        // CREDIT: code by Iiley, optimized with idea from ChaseSan
        // http://robowiki.net?BackAsFront
        double angle;
        setTurnRightRadians(Math.tan(angle =
                wallSmoothing(_myLocation, _goAngle, direction) - getHeadingRadians()));
        setAhead(Math.cos(angle) * Double.POSITIVE_INFINITY);

        ////////////////////////////////////////////////////////
        // CREDIT: Originally based on RaikoMicro's gun, by Jamougha
        // http://robowiki.net?RaikoMicro

        /////
        // TC
/*
        Wave w;
        addCustomEvent(w = new Wave());
        _enemyLocation =
            project((w.sourceLocation = _myLocation = new Point2D.Double(getX(), getY())),
            _enemyAbsoluteBearing, _lastDistance = e.getDistance());
*/
        /////

        // ------------- Fire control -------
        double enemyLatVel;
        _lastGunOrientation = w.orientation = sign(enemyLatVel = (e.getVelocity())*Math.sin(e.getHeadingRadians() - enemyAbsoluteBearing));

        int bestGF = Math.min(3, (int)(Math.pow(280*lastVChangeTime++/_lastDistance, .7)));
//        int bestGF = Math.min(3, (int)(Math.sqrt(220D*lastVChangeTime++/_lastDistance))); // TC
        int newVelocity;
        if (enemyVelocity != (newVelocity = (int)(enemyLatVel = Math.abs(enemyLatVel)))) {
            lastVChangeTime = 0;
            bestGF = 4;
            if (enemyVelocity > newVelocity) {
                bestGF = 5;
            }
        }
        enemyVelocity = newVelocity;


//        w.sourceLocation = _myLocation;
//        w.directAngle = _enemyAbsoluteBearing;
        w.waveGuessFactors = _gunStats[bestGF][(int)(LOG_BASE_E_TO_2_CONVERSION_CONSTANT * Math.log(enemyLatVel + 1.5))][gunWallDistance(0.18247367367) ? (gunWallDistance(0.36494734735) ? (gunWallDistance(0.63865785787) ? 3 : 2) : 1) : 0][gunWallDistance(-0.36494734735) ? 0 : 1][(int)limit(0, (_lastDistance-75)/200, 2)];
//        w.waveGuessFactors = _gunStats[bestGF][(int)(LOG_BASE_E_TO_2_CONVERSION_CONSTANT * Math.log(enemyLatVel + 1.5))][gunWallDistance(0.24430198263) ? (gunWallDistance(0.48860396528) ? (gunWallDistance(0.85505693924) ? 3 : 2) : 1) : 0][gunWallDistance(-0.48860396528) ? 0 : 1][(int)limit(0, (_lastDistance-75)/200, 2)]; // TC

        bestGF = GF_ZERO;

        for (int gf = GF_ONE; gf >= 0 && (_oppEnergy = e.getEnergy()) > 0; gf--) // Jamougha: Saves one byte compared to going up, weird
            if (w.waveGuessFactors[gf] > w.waveGuessFactors[bestGF])
                bestGF = gf;

        double power = 2 - Math.max(0, (30 - getEnergy()) / 16);
//      double power = 3; // TC
        w.distance = -1.5 * (w.bulletSpeed = Rules.getBulletSpeed(power));

        setTurnGunRightRadians(Utils.normalRelativeAngle(enemyAbsoluteBearing - getGunHeadingRadians() + ((_lastGunOrientation*(Math.asin(8/w.bulletSpeed)/GF_ZERO))*(bestGF-GF_ZERO)) ));
//        setTurnGunRightRadians(Utils.normalRelativeAngle(_enemyAbsoluteBearing - getGunHeadingRadians() + ((_lastGunOrientation*.035406)*(bestGF-GF_ZERO)) )); // TC

        if (Math.abs(getGunTurnRemaining()) < 3 && setFireBullet(power + (_ramCounter / (3*getRoundNum()+1))) != null) {
//        if (Math.abs(getGunTurnRemaining()) < 3 && setFireBullet(3) != null) { // TC
            w.weight = 4;
        }

        ////////////////////////////////////////////////////////

        _lastLatVel = getVelocity()*Math.sin(e.getBearingRadians());
        _goAngle = _lastAbsBearingRadians = enemyAbsoluteBearing + Math.PI;
    }

    public void onHitByBullet(HitByBulletEvent e) {
        _oppEnergy += e.getBullet().getPower() * 3;
        logAndRemoveWave(_myLocation);
    }

    public void onBulletHitBullet(BulletHitBulletEvent e) {
        logAndRemoveWave(new Point2D.Double(e.getBullet().getX(),
                e.getBullet().getY()));
    }

    public void logAndRemoveWave(Point2D.Double hitLocation) {
        Wave w = _surfWave;
        int x = 0;
        do {
            try {
                if (Math.abs(w.distanceToPoint(hitLocation) - w.distance) < 100) {
                    logHit(w, hitLocation, 0.85);
                    _enemyWaves.remove(w);
                    removeCustomEvent(w);
                    return;
                }
                w = (Wave)_enemyWaves.get(x++);
            } catch (Exception ex) { }
        } while (x <= _enemyWaves.size());
    }

    public void onBulletHit(BulletHitEvent e) {
        _oppEnergy -= Rules.getBulletDamage(e.getBullet().getPower());
    }

    public void onHitRobot(HitRobotEvent event) {
        _ramCounter++;
    }

    public void onCustomEvent(CustomEvent e) {
        removeCustomEvent(e.getCondition());
    }

    public static void logHit(Wave w, Point2D.Double targetLocation, double rollingDepth) {
        for (int x = GF_ONE; x >= 0; x--) {
            w.waveGuessFactors[x] = ((w.waveGuessFactors[x] * rollingDepth)
                    + ((1 + w.weight) / (Math.pow(x - getFactorIndex(w, targetLocation), 2) + 1)))
                    / (rollingDepth + 1 + w.weight);
        }
    }

    private static int getFactorIndex(Wave w, Point2D.Double botLocation) {
        return (int)limit(0,
                ((((Utils.normalRelativeAngle(
                        w.absoluteBearing(botLocation)
                                - w.directAngle) * w.orientation)
                        / Math.asin(8.0/w.bulletSpeed))
                        * (GF_ZERO)) + (GF_ZERO)), GF_ONE);
    }

    private double checkDanger(int direction) {
        Wave surfWave = Komarious._surfWave;
        Point2D.Double predictedPosition = _myLocation;
        double predictedHeading = getHeadingRadians();
        double predictedVelocity = getVelocity();
        double maxTurning, moveAngle, moveDir, lastPredictedDistance;

        // - actual distance traveled so far needs +bullet_velocity,
        //    because it's detected one after being fired
        // - another +bullet_velocity b/c bullet advances before collisions
        // ...So start the counter at 2.
        // - another +bullet_velocity to approximate a bot's half width
        // ...So start the counter at 3.
        int counter = 3;

        do {
            moveDir = 1;

            if (Math.cos(moveAngle =
                    wallSmoothing(predictedPosition, surfWave.absoluteBearing(
                            predictedPosition) + (direction * A_LITTLE_LESS_THAN_HALF_PI), direction)
                            - predictedHeading) < 0) {
                moveAngle += Math.PI;
                moveDir = -1;
            }

            // rozu comment:
            // this one is nice ;). if predictedVelocity and moveDir have different signs you want to breack down
            // otherwise you want to accelerate (look at the factor "2")
//            predictedVelocity += (predictedVelocity * moveDir < 0 ? 2*moveDir : moveDir);
//            predictedVelocity = limit(-8,
//                predictedVelocity + (predictedVelocity * moveDir < 0 ? 2*moveDir : moveDir),
//                8);

            // rozu comment:
            // calculate the new predicted position
            predictedPosition = project(predictedPosition,
                    predictedHeading = Utils.normalRelativeAngle(predictedHeading +
                            limit(-(maxTurning = Rules.getTurnRateRadians(Math.abs(predictedVelocity))),
                                    Utils.normalRelativeAngle(moveAngle), maxTurning)),
                    (predictedVelocity = limit(-8,
                            predictedVelocity + (predictedVelocity * moveDir < 0 ? 2*moveDir : moveDir),
                            8)));


        } while (
                (lastPredictedDistance = surfWave.distanceToPoint(predictedPosition)) >=
                        surfWave.distance + ((++counter) * surfWave.bulletSpeed));

        int index;
        return (surfWave.waveGuessFactors[index = getFactorIndex(surfWave, predictedPosition)]
                + .01 / (Math.abs(index - GF_ZERO) + 1))
                / Math.pow(Math.min(_enemyLocation.distance(predictedPosition), lastPredictedDistance), 4);
    }

    private static double limit(double min, double value, double max) {
        return Math.max(min, Math.min(value, max));
    }

    // CREDIT: ganked from CassiusClay, by PEZ
    // http://robowiki.net?CassiusClay
    private static Point2D.Double project(Point2D.Double sourceLocation, double angle, double length) {
        return new Point2D.Double(sourceLocation.x + Math.sin(angle) * length,
                sourceLocation.y + Math.cos(angle) * length);
    }

    private static int sign(double d) {
        if (d < 0) { return -1; }
        return 1;
    }

    // CREDIT: Iterative WallSmoothing by PEZ
    // http://robowiki.net?WallSmoothing
    private static double wallSmoothing(Point2D.Double botLocation, double angle, int orientation) {
        while (!_fieldRect.contains(project(botLocation, angle, WALL_STICK))) {
            angle += orientation*0.05;
        }
        return angle;
    }

    private static boolean gunWallDistance(double wallDistance) {
        return _fieldRect.contains(project(_myLocation, _enemyAbsoluteBearing + (_lastGunOrientation*wallDistance), _lastDistance));
    }

    static class Wave extends Condition {
        Point2D.Double sourceLocation;
        double[] waveGuessFactors;
        double bulletSpeed, directAngle, distance;
        int orientation, weight;

        public double distanceToPoint(Point2D.Double p) {
            return sourceLocation.distance(p);
        }

        public double absoluteBearing(Point2D.Double target) {
            return Math.atan2(target.x - sourceLocation.x, target.y - sourceLocation.y);
        }

        public boolean test() {
            if (distanceToPoint(Komarious._enemyWaves.contains(this)?
                    Komarious._myLocation:Komarious._enemyLocation)
                    <= (distance+=bulletSpeed) + (2 * bulletSpeed)) {
//            logHit((Wave)_enemyWaves.removeFirst(),
//                Komarious._myLocation, 500);
                if (!Komarious._enemyWaves.remove(this)) {
                    Komarious.logHit(this, Komarious._enemyLocation, 600);
                }

                return true;
            }
            return false;
        }
    }
}
