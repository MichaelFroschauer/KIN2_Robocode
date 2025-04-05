import robocode.*;
import robocode.util.Utils;

import java.awt.*;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.List;
import java.util.*;

/**
 * "True Surfing" based on tutorial on Wiki
 * with Segmentation, Bin Smoothing, etc
 * <p>
 * General Concept:
 * 1) Detect shots by energy drops in enemies
 * 2) Detected Shot creates Wave with speed determined by amount of energy drop
 * 3) Every single tick
 * - a) Determine the most dangerous wave
 * - b) Circle around that wave (move facing 90 degrees away from its center)
 * - c) Predict the exact position of where we would intercept that wave if we were to circle left or right
 * - d) Determine the riskiness of those two positions for the current wave
 * - e) Choose the less risky direction and move there!
 * <p>
 * The riskiness of a position is determined as follows:
 * 1) When we are hit by a bullet,
 * - a) calculate the offset from
 * - - the angle towards the position the enemy saw us at (so looking directly at us),
 * - - and where he must have shot instead to land a hit on us.
 * - b) Then, mark those angle offset as more dangerous (using a number of risk scores, called buckets)
 * 2) When we need to calculate the riskiness of a position and wave,
 * we simply check which offset angle the enemy would have had those choose to hit that position when shooting,
 * and check the risk value stored in the corresponding bucket.
 */

/*
 * after 5 hits, switch to stop-and-go mode:
 *
 * "StopAndGo" Movement Strategy with Adaptive Fire Detection
 * <p>
 * This strategy is designed to confuse enemy targeting systems by causing the robot to intermittently
 * stop (ideally when the enemy fires) and then resume lateral movement in an unpredictable manner
 * <p>
 * General Concept:
 * 1) for each detected enemy, compute the absolute bearing from your robot’s current position
 * 2) calculate a lateral movement angle by adding a 90° offset (multiplied by a directional factor)
 *    to the enemy's bearing
 * 3) adjust the calculated movement angle if the projected move (100 units ahead) would place the robot
 *    too close to the battlefield boundaries; if so -> steer toward the center of the battlefield
 * 4) Monitor the enemy’s energy by comparing the current energy level with the previous tick
 *    - when a shot is detected, toggle a flag so that the robot alternates between stopping and moving
 * 5) Execute stop-and-go behavior based on the flag:
 *    - if the flag (stopNextTick) is true, the robot remains stopped
 *    - otherwise, the robot moves forward
 * 6) alternate the lateral movement direction after each scan to add unpredictability
 * 7) upon hitting a wall, immediately reverse direction and adjust movement to avoid getting stuck
 */

public class Evadomatic3000v4 extends AdvancedRobot {

    private static final double NINETY_DEG_RAD = Math.PI / 2;
    private static final double SLIGHTLY_BELOW_NINETY_DEG_RAD = Math.toRadians(72); // to gain distance on enemies

    private static final int NUM_BINS_ANGLE = 47; // 23 on each side + center
    private static final int CENTER_BIN_ANGLE = (NUM_BINS_ANGLE - 1) / 2;

    private static final int NUM_BINS_DISTANCE = 5;
    private static final int DISTANCE_STEPS = 800 / NUM_BINS_DISTANCE;

    private static final int NUM_BINS_FLIGHTTIME = 7;
    private static final int FLIGHTTIME_STEPS = 100 / NUM_BINS_FLIGHTTIME;

    private static final Rectangle2D.Double TARGET_AREA
            = new Rectangle2D.Double(20, 20, 760, 560);
    private static final int TARGET_AREA_PADDING = 150; // for wall smoothing

    private final Map<String, RobotData> enemies = new HashMap<>();
    private Point2D.Double myPos, myPreviousPos;

    // stop and go

    private double movementDirection = 1;
    private final double SAFE_MARGIN = 50.0;

    private double lastEnemyEnergy = 100.0; // typical start value
    private boolean stopNextTick = false;

    // for switching between stop and go and surfing
    private static final int HIT_THRESHOLD = 5; // number of hits to switch between modes
    private int timesHitRecently = 0;

    // for debugging
    private Wave evilWaveJustHitUsReeee;

    // ------------ functions for true surfing ------------
    private void onScannedRobotSurfing(ScannedRobotEvent event) {
        if (myPos != null) {
            myPreviousPos = (Point2D.Double) myPos.clone();
        }
        myPos = new Point2D.Double(getX(), getY());
        Evadomatic3000v4.RobotData enemy = enemies.computeIfAbsent(event.getName(), ignored -> new RobotData());
        enemy.distance = event.getDistance();
        enemy.lastSeen = getTime();
        enemy.heading = event.getHeadingRadians();

        // add heading, as getBearing() returns bearing relative to own heading
        double absoluteBearing = Math.toRadians((getHeading() + event.getBearing()) % 360);

        // TODO: no-no - look at single target
        setTurnRadarRightRadians(Utils.normalRelativeAngle(absoluteBearing
                - getRadarHeadingRadians()) * 2);

        // track surfing data
        double rotationalVelocity = getVelocity() * Math.sin(event.getBearingRadians());
        enemy.surfDirections.addFirst(new Evadomatic3000v4.RobotData.TimestampedEntry<>(enemy.lastSeen,
                rotationalVelocity >= 0 ? 1 : -1  // direction we are currently moving in, relative to enemy
        ));
        enemy.surfAngles.addFirst(new Evadomatic3000v4.RobotData.TimestampedEntry<>(enemy.lastSeen,
                absoluteBearing + Math.PI  // angle enemy shot us with
        ));

        // detect shot by energy drop
        if (enemy.energyLevels.size() >= 2) { // we need at least two ticks of data
            double lastEnergy = enemy.energyLevels.getFirst().value;
            double energyDelta = lastEnergy - event.getEnergy();
            out.println(energyDelta);
            // TODO: account for gaps between scans and other energy fluctuation (idle?)
            if (0.0999 <= energyDelta && // valid energy cost for bullet is in [0.1, 3]
                    energyDelta <= 3.001) {
                // opponent (probably) shot bullet - create new wave
                double bulletVelocity = calcBulletVelocity(energyDelta);
                Evadomatic3000v4.Wave wave = new Evadomatic3000v4.Wave(
                        enemy,
                        ((Point2D.Double) enemy.pos.clone()), // still previous position :)
                        ((Point2D.Double) myPreviousPos.clone()),
                        getTime() - 1, // if we detect it now, the enemy shot last tick
                        bulletVelocity,
                        enemy.surfAngles.get(2).value, // if enemy shot last tick, it shot based on our position...
                        enemy.surfDirections.get(2).value, // ...and our movement two ticks ago
                        bulletVelocity, // and the bullet already moved for one tick
                        energyDelta
                );
                enemy.waves.add(wave);
            }
        }

        // track enemy energy
        enemy.energyLevels.addFirst(new Evadomatic3000v4.RobotData.TimestampedEntry<>(enemy.lastSeen,
                event.getEnergy()
        ));

        // track enemy position
        enemy.pos = offsetPointByAngle(myPos, absoluteBearing, enemy.distance);
    }

    // ------------ functions for stop and go ------------
    /**
     * checks if the future point (100 units ahead) is too close to the battlefield boundaries.
     * if so, adjusts the movement angle so that the robot steers towards the battlefield center.
     */
    private double adjustAngleForWalls(double desiredAngle) {
        double moveDistance = 100; // assumed movement distance
        double futureX = getX() + Math.sin(desiredAngle) * moveDistance;
        double futureY = getY() + Math.cos(desiredAngle) * moveDistance;
        double battlefieldWidth = getBattleFieldWidth();
        double battlefieldHeight = getBattleFieldHeight();

        // if the future point is too close to the boundaries, steer towards the battlefield center.
        if (futureX < SAFE_MARGIN || futureX > battlefieldWidth - SAFE_MARGIN ||
                futureY < SAFE_MARGIN || futureY > battlefieldHeight - SAFE_MARGIN) {
            double centerX = battlefieldWidth / 2.0;
            double centerY = battlefieldHeight / 2.0;
            return Math.atan2(centerX - getX(), centerY - getY());
        }
        return desiredAngle;
    }

    private void onScannedRobotStopAndGo(ScannedRobotEvent event) {
        double absoluteBearing = Math.toRadians((getHeading() + event.getBearing()) % 360);

        // TODO: no-no - look at single target
        setTurnRadarRightRadians(Utils.normalRelativeAngle(absoluteBearing
                - getRadarHeadingRadians()) * 2);

        // compute the desired angle for lateral movement relative to the enemy.
        double desiredAngle = absoluteBearing + (Math.PI / 2 * movementDirection);
        // adjust the angle if the future point (100 units ahead) is too close to the battlefield boundaries.
        desiredAngle = adjustAngleForWalls(desiredAngle);
        setTurnRightRadians(Utils.normalRelativeAngle(desiredAngle - getHeadingRadians()));

        // detect enemy fire by checking for an energy drop.
        double currentEnemyEnergy = event.getEnergy();
        double energyDrop = lastEnemyEnergy - currentEnemyEnergy;
        // if the energy drop is between 0.1 and 3.0, assume that the enemy fired.
        if (energyDrop > 0.1 && energyDrop <= 3.0) {
            // toggle: when the enemy fires, alternate between stopping and moving.
            stopNextTick = !stopNextTick;
        }
        lastEnemyEnergy = currentEnemyEnergy;

        // stop-and-go behavior: if stopnexttick is true, the bot stays still; otherwise, it moves.
        if (stopNextTick) {
            out.println("Stop");
            setAhead(0);
        } else {
            out.println("Go");
            setAhead(100);
        }

        // alternate the lateral movement direction for unpredictability.
        movementDirection = -movementDirection;
    }

    // ------------ main functions ------------
    public void run() {
        // TODO: reset between rounds?
        // enemies = new HashMap<>();

        setAdjustRadarForGunTurn(true);
        setAdjustRadarForRobotTurn(true);

        setTurnRadarRight(Double.POSITIVE_INFINITY);
        while (true) {
            update();
            execute();
        }
    }

    @Override
    public void onScannedRobot(ScannedRobotEvent event) {
        if (timesHitRecently / HIT_THRESHOLD % 2 != 0) {
            out.println("stop and go mode");
            onScannedRobotStopAndGo(event);
        } else {
            out.println("surf mode");
            onScannedRobotSurfing(event);
        }
    }

    private void update() {
        // update waves
        for (RobotData enemy : enemies.values()) {
            Iterator<Wave> wavesIterator = enemy.waves.iterator();
            while (wavesIterator.hasNext()) {
                Wave wave = wavesIterator.next();
                wave.progress += wave.velocity;
                if (wave.progress > myPos.distance(wave.center) + 35) { // assume this is enough margin
                    // wave has passed us by margin above.. :relief: 😮‍💨
                    wavesIterator.remove();
                }
            }
        }

        // surf! yippie
        surf();
    }

    private Wave getMostDangerousWave() {
        double closestDistance = Double.MAX_VALUE;
        Wave closestWave = null;

        for (RobotData enemy : enemies.values()) {
            for (Wave wave : enemy.waves) {
                double distance = myPos.distance(wave.center) - wave.progress;
                double ticksUntilHit = distance / wave.velocity;
                if (ticksUntilHit < closestDistance && distance > wave.velocity) { // TODO: adjust?
                    closestDistance = ticksUntilHit;
                    closestWave = wave;
                }
            }
        }
        return closestWave;
    }

    private static int getAngleBinForWaveHit(Wave wave, Point2D.Double hitAt) {
        double angleOffsetEnemyShotAt = Utils.normalRelativeAngle(calculateBearing(wave.center, hitAt) - wave.angle);
        double maxEscapeAngle = maxEscapeAngle(wave.velocity); // TODO: why does distance not matter?
        double enemyShotAngleFactor = angleOffsetEnemyShotAt / maxEscapeAngle;

        // return bin id for angle percentage of maxEscapeAngle chosen by enemy for shot
        return clamp(
                0,
                (int) (enemyShotAngleFactor * wave.direction * CENTER_BIN_ANGLE) + CENTER_BIN_ANGLE,
                NUM_BINS_ANGLE - 1
        );
    }

    private static int getDistanceBinForWaveHit(Wave wave, Point2D.Double hitAt) {
        double distance = wave.center.distance(wave.myPosOnShoot);
        int distanceBin = (int) Math.round(distance / DISTANCE_STEPS);
        return clamp(0, distanceBin, NUM_BINS_DISTANCE - 1);
    }

    private static int getFlighttimeBinForWave(Wave wave) {
        double distance = wave.center.distance(wave.myPosOnShoot) / (20 - 3 * wave.shotPower);
        int distanceBin = (int) Math.round(distance / FLIGHTTIME_STEPS);

        return clamp(0, distanceBin, NUM_BINS_FLIGHTTIME - 1);
    }

    private Wave getAndRemoveWaveThatHitMe(Bullet bullet) {
        for (RobotData enemy : enemies.values()) {
            Iterator<Wave> wavesIterator = enemy.waves.iterator();
            while (wavesIterator.hasNext()) {
                Wave wave = wavesIterator.next();

                double distanceToMe = Math.abs(myPos.distance(wave.center) - wave.progress); // abs() for same "hitbox" after our center
                boolean bulletVelocityMatches = Math.abs(bullet.getVelocity() - wave.velocity) < 0.1; // epsilon
                if (distanceToMe < 50 && bulletVelocityMatches) {
                    wavesIterator.remove();
                    return wave;
                }
            }
        }
        return null;
    }

    @Override
    public void onHitByBullet(HitByBulletEvent event) {
        timesHitRecently++;
        out.println("Hit: " + timesHitRecently);

        Bullet bullet = event.getBullet();
        Point2D.Double hitPos = new Point2D.Double(bullet.getX(), bullet.getY());
        Wave evilWave = getAndRemoveWaveThatHitMe(bullet);
        evilWaveJustHitUsReeee = evilWave; // ...

        if (evilWave != null) {
            RobotData enemy = evilWave.robotData;
            int angleBucket = getAngleBinForWaveHit(evilWave, hitPos);
            int distanceBucket = getDistanceBinForWaveHit(evilWave, hitPos);
            int flighttimeBucket = getFlighttimeBinForWave(evilWave);

            for (int flighttimeI = 0; flighttimeI < NUM_BINS_FLIGHTTIME; flighttimeI++) {
                for (int distanceI = 0; distanceI < NUM_BINS_DISTANCE; distanceI++) {
                    for (int angleI = 0; angleI < NUM_BINS_ANGLE; angleI++) {
                        // increment buckets around hit with even distribution (1/2, 1/5, ...)
                        double oldValue = enemy.stats[flighttimeI][distanceI][angleI];
                        double newValue =
                                0.333 / (Math.pow(angleBucket - angleI, 2) + 1)
                                        + 0.333 / (Math.pow(distanceBucket - distanceI, 2) + 1)
                                        + 0.333 / (Math.pow(flighttimeBucket - distanceI, 2) + 1);

                        oldValue *= 0.95; // decay existing value -> prefer fresher data
                        if (oldValue < 0.00001) { // (0.95 ^ 200) * 0.33
                            oldValue = 0;
                        }
                        enemy.stats[flighttimeI][distanceI][angleI]
                                = oldValue + newValue;
                    }
                }
            }
        }

        // adjust enemy energy
        RobotData enemy = enemies.get(event.getName());
        if (enemy == null) return;

        double bulletBonus = Rules.getBulletHitBonus(bullet.getPower());
        double currentEnergy = enemy.energyLevels.getFirst().value;
        enemy.energyLevels.add(new RobotData.TimestampedEntry<>(getTime(), currentEnergy + bulletBonus));
    }

    // for wave and direction (left/right),
    // simulate movement 90 degrees towards the center of the wave,
    // account for game physics and wall smoothing,
    // calculate precise position for interception with that wave
    private Point2D.Double predictInterception(Wave wave, int direction) {
        Point2D.Double result = new Point2D.Double(getX(), getY());
        double resultVelocity = getVelocity();
        double resultHeading = getHeadingRadians();

        int simulatedTicks = 0;
        boolean intercepted = false;
        do {
            // calculate desired angle (to check), 90 degrees to center of wave
            double ninetyDegAngle = calculateBearing(wave.center, result) + (direction * SLIGHTLY_BELOW_NINETY_DEG_RAD);

            // adjust angle to not run into walls
            double moveAngle = wallSmoothing(result, ninetyDegAngle, direction) - resultHeading;

            // adjust direction (clockwise/counterclockwise)
            double moveDir = 1;
            if (Math.cos(moveAngle) < 0) {
                moveDir = -1;
                moveAngle += Math.PI;
            }
            moveAngle = Utils.normalRelativeAngle(moveAngle);

            // calculate maximum turning rate dependent velocity, game physics
            double maxTurning = Math.PI / 720.0 * (40.0 - 3.0 * Math.abs(resultVelocity));
            // adjust heading to what is actually allowed by maximum turning rate
            double actualMoveAngle = clamp(-maxTurning, moveAngle, maxTurning);

            // calculate resulting heading
            resultHeading = Utils.normalRelativeAngle(resultHeading + actualMoveAngle);

            // consider maximum acceleration (1) and maximum braking (2)
            if (resultVelocity * moveDir < 0) {
                resultVelocity += 2 * moveDir;
            } else {
                resultVelocity += moveDir;
            }
            resultVelocity = clamp(-8, resultVelocity, 8); // game allows maximum velocity of 8

            // calculate next position
            result = offsetPointByAngle(result, resultHeading, resultVelocity);

            simulatedTicks++;

            // if wave reached resulting position: resulting position is interception point!
            if (result.distance(wave.center) < wave.progress + ((simulatedTicks + 1) * wave.velocity)) {
                intercepted = true;
            }
        } while (!intercepted && simulatedTicks < 500); // hard limit ticks

        return result;
    }

    private double getRiskForDirection(Wave wave, int direction) {
        // predict position for direction
        Point2D.Double predictedPosition = predictInterception(wave, direction);

        // determine bucket for this position,
        // and with that how risky such an angle is
        int angleBucket = getAngleBinForWaveHit(wave, predictedPosition);
        int distanceBucket = getDistanceBinForWaveHit(wave, predictedPosition);
        int flighttimeBucket = getFlighttimeBinForWave(wave);
        return wave.robotData.stats[flighttimeBucket][distanceBucket][angleBucket];
    }

    // surf by moving either left or right in relation to the center of the wave
    //   - direction depends on which is less risky according to previous hits on the angle
    //     the predicted position in the chosen direction would imply
    private void surf() {
        Wave waveToSurf = getMostDangerousWave();
        if (waveToSurf == null) return;

        // calculate risk for going left and right
        double risk1 = getRiskForDirection(waveToSurf, -1);
        double risk2 = getRiskForDirection(waveToSurf, 1);

        // calculate left or right move angle, depending on which is lower risk
        double waveAngle = calculateBearing(waveToSurf.center, myPos);
        int betterDirection = risk1 < risk2 ? -1 : 1;
        double moveAngle = wallSmoothing(
                myPos,
                waveAngle + SLIGHTLY_BELOW_NINETY_DEG_RAD * betterDirection,
                betterDirection
        );
        moveAtAbsoluteAngle(moveAngle);
    }

    // adjust enemy energy level when our bullet hits
    @Override
    public void onBulletHit(BulletHitEvent event) {
        RobotData enemy = enemies.get(event.getName());
        if (enemy == null) return;

        double bulletDamage = Rules.getBulletDamage(event.getBullet().getPower());
        double currentEnergy = enemy.energyLevels.getFirst().value;
        enemy.energyLevels.add(new RobotData.TimestampedEntry<>(getTime(), currentEnergy - bulletDamage));
    }

    @Override
    public void onHitWall(HitWallEvent event) {
        // upon hitting a wall, reverse the movement direction to avoid sticking.
        movementDirection = -movementDirection;
        setTurnRight(90);
        setAhead(100);
    }

    @Override
    public void onPaint(Graphics2D g) {
        // draw own headings
        g.setColor(Color.green);
        g.drawLine(
                (int) getX(),
                (int) getY(),
                (int) (getX() + Math.sin(getHeading()) * 100),
                (int) (getX() + Math.cos(getHeading()) * 100)
        );
        g.setColor(Color.red);
        g.drawLine(
                (int) getX(),
                (int) getY(),
                (int) (getX() + Math.sin(getGunHeading()) * 100),
                (int) (getX() + Math.cos(getGunHeading()) * 100)
        );

        g.setColor(Color.WHITE);
        enemies.values().forEach(robot -> {
            // enemy box
            g.drawRect(
                    (int) robot.pos.x - 25,
                    (int) robot.pos.y - 25,
                    50, 50
            );
            // enemy angle
            g.drawLine(
                    (int) robot.pos.x,
                    (int) robot.pos.y,
                    (int) (robot.pos.x + (Math.sin(robot.heading) * 50)),
                    (int) (robot.pos.y + (Math.cos(robot.heading) * 50))
            );

            // waves
            for (Wave wave : robot.waves) {
                // draw wave
                g.drawOval(
                        (int) (wave.center.x - wave.progress),
                        (int) (wave.center.y - wave.progress),
                        (int) (wave.progress * 2),
                        (int) (wave.progress * 2)
                );
            }
        });

        // most dangerous wave
        Wave mostDangerousWave = getMostDangerousWave();
        if (mostDangerousWave != null) {
            g.setColor(Color.pink);
            drawWave(mostDangerousWave, g);
        }

        // evil wave (which just hit us :rage:)
        if (evilWaveJustHitUsReeee != null) {
            g.setColor(Color.GREEN);
            drawWave(evilWaveJustHitUsReeee, g);
        }
    }

    private static void drawWave(Wave wave, Graphics2D g) {
        g.drawOval(
                (int) (wave.center.x - wave.progress),
                (int) (wave.center.y - wave.progress),
                (int) (wave.progress * 2),
                (int) (wave.progress * 2)
        );
    }

    static class Wave {
        RobotData robotData;
        Point2D.Double center = new Point2D.Double();
        Point2D.Double myPosOnShoot = new Point2D.Double();
        long startTicks;
        double velocity, angle /* from center to where we were when enemy fired */, progress;
        int direction /* we were moving when enemy fired */;
        double shotPower;

        public Wave(RobotData robotData, Point2D.Double center, Point2D.Double myPosOnShoot, long startTicks, double velocity, double angle, int direction, double progress, double shotPower) {
            this.robotData = robotData;
            this.center = center;
            this.myPosOnShoot = myPosOnShoot;
            this.startTicks = startTicks;
            this.velocity = velocity;
            this.angle = angle;
            this.progress = progress;
            this.direction = direction;
            this.shotPower = shotPower;
        }
    }

    static class RobotData {
        double[][][] stats = new double[NUM_BINS_FLIGHTTIME][NUM_BINS_DISTANCE][NUM_BINS_ANGLE];
        Point2D.Double pos = new Point2D.Double();
        List<Evadomatic3000v4.Wave> waves = new ArrayList<>();
        List<Evadomatic3000v4.RobotData.TimestampedEntry<Integer>> surfDirections = new LinkedList<>();
        List<Evadomatic3000v4.RobotData.TimestampedEntry<Double>> surfAngles = new LinkedList<>();
        List<Evadomatic3000v4.RobotData.TimestampedEntry<Double>> energyLevels = new LinkedList<>();
        double distance;
        long lastSeen;
        double heading;

        static class TimestampedEntry<T> {
            T value;
            long timestamp;

            public TimestampedEntry(long timestamp, T value) {
                this.timestamp = timestamp;
                this.value = value;
            }
        }
    }

    // from wiki
    // move towards an angle efficiently
    // for example, instead of turning left by 120 degrees and moving forwards,
    // turn right 60 degrees and move backwards
    private void moveAtAbsoluteAngle(double goalAngle) {
        double angleDelta = Utils.normalRelativeAngle(goalAngle - getHeadingRadians());
        double absAngleDelta = Math.abs(angleDelta);
        if (absAngleDelta > NINETY_DEG_RAD) {
            if (angleDelta < 0) { // bottom left quadrant
                setTurnRightRadians(Math.PI + angleDelta);
            } else { // bottom right quadrant
                setTurnLeftRadians(Math.PI - angleDelta);
            }
            setBack(100);
        } else {
            if (angleDelta < 0) { // top left quadrant
                setTurnLeftRadians(absAngleDelta);
            } else { // top right quadrant
                setTurnRightRadians(angleDelta);
            }
            setAhead(100);
        }
    }

    // calculation from wiki
    private static double calcBulletVelocity(double power) {
        return (20.0 - (3.0 * power));
    }

    // calculation from wiki
    // returns maximum angle (delta to direct lock) that could hit enemy, depending on bullet velocity
    private static double maxEscapeAngle(double velocity) {
        return Math.asin(8.0 / velocity);
    }

    // calculation from wiki adjusts angle to prevent bot from running into wall by iteratively checking and reducing said angle
    public double wallSmoothing(Point2D.Double botLocation, double angle, int orientation) {
        while (!TARGET_AREA.contains(offsetPointByAngle(botLocation, angle, TARGET_AREA_PADDING))) {
            angle += orientation * 0.05; // slightly adjust angle
        }
        return angle;
    }

    // math maths
    private static double calculateBearing(Point2D.Double from, Point2D.Double to) {
        double g = to.x - from.x;
        double a = to.y - from.y;
        return Math.atan2(g, a); // tan-1(G, A)
    }

    private static Point2D.Double offsetPointByAngle(Point2D.Double from, double angle, double length) {
        return new Point2D.Double(
                from.x + Math.sin(angle) * length, // sin(a) = G/H; sin(a) * H = G (= xDelta)
                from.y + Math.cos(angle) * length // cos(a) = A/H; cos(a) * H = A (= yDelta)
        );
    }

    private static int clamp(int min, int value, int max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    private static double clamp(double min, double value, double max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }
}

