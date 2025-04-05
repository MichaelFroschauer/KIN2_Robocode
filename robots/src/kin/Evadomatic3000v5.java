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

/**
 * after N hits, switch to random movement (semi-'stop and go') for M ticks
 */
public class Evadomatic3000v5 extends AdvancedRobot {

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

    private static final Random RANDOM = new Random();

    private final Map<String, RobotData> enemies = new HashMap<>();
    private Point2D.Double myPos, myPreviousPos;

    // stop and go
    private int stopAndGoDirection = 1;
    private int ticksToJustGo = 0;
    // for switching between stop and go and surfing
    private static final int HIT_THRESHOLD = 4; // number of hits to switch between modes
    private static final int HIT_TIMESPAN = 90; // ticks to consider previous hits in
    private static final int STOP_AND_GO_DURATION = 40; // ticks to do stop and go movement in
    private final List<Boolean> previousHitsWhileSurfing = new LinkedList<>();
    private MovementType movementType;
    private long stopAndGoEndTime;

    // for debugging
    private Wave evilWaveJustHitUsReeee;

    // ------------ main functions ------------
    public void run() {
        // TODO: reset between rounds?
        // enemies = new HashMap<>();

        movementType = MovementType.TRUE_SURFING;
        stopAndGoEndTime = Long.MAX_VALUE;

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
        if (myPos != null) {
            myPreviousPos = (Point2D.Double) myPos.clone();
        }
        myPos = new Point2D.Double(getX(), getY());
        RobotData enemy = enemies.computeIfAbsent(event.getName(), ignored -> new RobotData());
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
        enemy.surfDirections.addFirst(new RobotData.TimestampedEntry<>(enemy.lastSeen,
                rotationalVelocity >= 0 ? 1 : -1  // direction we are currently moving in, relative to enemy
        ));
        enemy.surfAngles.addFirst(new RobotData.TimestampedEntry<>(enemy.lastSeen,
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
                Wave wave = new Wave(
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
        enemy.energyLevels.addFirst(new RobotData.TimestampedEntry<>(enemy.lastSeen,
                event.getEnergy()
        ));

        // track enemy position
        enemy.pos = offsetPointByAngle(myPos, absoluteBearing, enemy.distance);
    }

    private void update() {
        // update previously hit list
        previousHitsWhileSurfing.addFirst(false);
        if (previousHitsWhileSurfing.size() > HIT_TIMESPAN) {
            previousHitsWhileSurfing.removeLast();
        }

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

        // update movement mode
        int timesPreviouslyHit = calcTimesPreviouslyHitWhileSurfing();
        switch (movementType) {
            case TRUE_SURFING -> {
                // surf! yippie
                surf();

                if (timesPreviouslyHit >= HIT_THRESHOLD) {
                    movementType = MovementType.STOP_AND_GO;
                    stopAndGoEndTime = getTime() + STOP_AND_GO_DURATION;
                }
            }
            case STOP_AND_GO -> {
                // hehehehe
                stopAndGo();

                if (getTime() >= stopAndGoEndTime) {
                    movementType = MovementType.TRUE_SURFING;
                }
            }
        }
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
        if (movementType == MovementType.TRUE_SURFING) {
            previousHitsWhileSurfing.set(0, true);
        }

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
        moveAtAbsoluteAngle(moveAngle, 100); // TODO: tweak?
    }

    private void stopAndGo() {
        Wave waveToFool = getMostDangerousWave();
        if (waveToFool == null) return;

        ticksToJustGo--;
        if (ticksToJustGo > 0) return; // wait until updating movement again

        double distance = RANDOM.nextDouble(70, 120);
        double waveAngle = calculateBearing(waveToFool.center, myPos);
        double moveAngle = wallSmoothing(
                myPos,
                waveAngle + SLIGHTLY_BELOW_NINETY_DEG_RAD * stopAndGoDirection,
                stopAndGoDirection
        );
        moveAtAbsoluteAngle(moveAngle, distance);

        ticksToJustGo += 15;
        if (RANDOM.nextDouble() < 0.7) { // reverse direction most of the time
            stopAndGoDirection = -stopAndGoDirection;
        }
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
        if (movementType == MovementType.STOP_AND_GO) {
            // upon hitting a wall, reverse the movement direction to avoid sticking.
            stopAndGoDirection = -stopAndGoDirection;
            setTurnRight(90);
            setAhead(100);
        }
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

        // Movement Mode
        g.setColor(Color.WHITE);
        String text = "Using " + movementType;
        text += switch (movementType) {
            case STOP_AND_GO -> " for %d more ticks".formatted(stopAndGoEndTime - getTime());
            default -> " with only %d hits".formatted(calcTimesPreviouslyHitWhileSurfing());
        };
        g.drawString(text, 10, 10);
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
        List<Evadomatic3000v5.Wave> waves = new ArrayList<>();
        List<Evadomatic3000v5.RobotData.TimestampedEntry<Integer>> surfDirections = new LinkedList<>();
        List<Evadomatic3000v5.RobotData.TimestampedEntry<Double>> surfAngles = new LinkedList<>();
        List<Evadomatic3000v5.RobotData.TimestampedEntry<Double>> energyLevels = new LinkedList<>();
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

    private enum MovementType {
        TRUE_SURFING,
        STOP_AND_GO
    }

    // TODO: consider number of enemies
    private int calcTimesPreviouslyHitWhileSurfing() {
        return previousHitsWhileSurfing.stream().mapToInt(b -> b ? 1 : 0).sum();
    }

    // from wiki
    // move towards an angle efficiently
    // for example, instead of turning left by 120 degrees and moving forwards,
    // turn right 60 degrees and move backwards
    private void moveAtAbsoluteAngle(double goalAngle, double distance) {
        double angleDelta = Utils.normalRelativeAngle(goalAngle - getHeadingRadians());
        double absAngleDelta = Math.abs(angleDelta);
        if (absAngleDelta > NINETY_DEG_RAD) {
            if (angleDelta < 0) { // bottom left quadrant
                setTurnRightRadians(Math.PI + angleDelta);
            } else { // bottom right quadrant
                setTurnLeftRadians(Math.PI - angleDelta);
            }
            setBack(distance);
        } else {
            if (angleDelta < 0) { // top left quadrant
                setTurnLeftRadians(absAngleDelta);
            } else { // top right quadrant
                setTurnRightRadians(angleDelta);
            }
            setAhead(distance);
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

