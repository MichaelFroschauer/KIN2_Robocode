import robocode.*;
import robocode.util.Utils;

import java.awt.*;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.List;
import java.util.*;
import java.util.function.Function;

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
 * - - - a) Bin Smoothing: Exactly matching bin is incremented, but those surrounding also by a lower amount
 * - c) Also segment by other factors like distance to enemy, bullet speed, ...
 * 2) When we need to calculate the riskiness of a position and wave,
 * we simply check which offset angle the enemy would have had those choose to hit that position when shooting,
 * and check the risk value stored in the corresponding bucket.
 */

/**
 * after N hits, switch to random movement (semi-'stop and go') for M ticks
 */
public class NoobSlayer2500 extends AdvancedRobot {

    private static final double NINETY_DEG_RAD = Math.PI / 2;
    private static final double SLIGHTLY_BELOW_NINETY_DEG_RAD = Math.toRadians(72); // to gain distance on enemies

    private static final int MOV_NUM_BINS_ANGLE = 47; // 23 on each side + center
    private static final int MOV_CENTER_BIN_ANGLE = (MOV_NUM_BINS_ANGLE - 1) / 2;

    private static final int MOV_NUM_BINS_DISTANCE = 5;
    private static final int MOV_DISTANCE_STEPS = 800 / MOV_NUM_BINS_DISTANCE;

    private static final int MOV_NUM_BINS_FLIGHTTIME = 7;
    private static final int MOV_FLIGHTTIME_STEPS = 100 / MOV_NUM_BINS_FLIGHTTIME;

    private static final int GUN_MAX_BULLET_DISTANCE = 900; // TODO: dynamic(tm)
    private static final int GUN_NUM_BINS_DISTANCE = 5;
    private static final int GUN_NUM_BINS_VELOCITY = 5;
    private static final int GUN_NUM_BINS_ANGLE = 45;
    private static final int GUN_BINS_ANGLE_MIDDLE = (GUN_NUM_BINS_ANGLE - 1) / 2;
    private static final double GUN_MAX_ESCAPE_ANGLE_RAD = 0.6; // TODO: optimize
    private static final double GUN_BIN_WIDTH_ANGLE = GUN_MAX_ESCAPE_ANGLE_RAD / (double) GUN_BINS_ANGLE_MIDDLE;
    private static final double GUN_MIN_SHOOTING_ENERGY = 5;
    private static final double GUN_TURN_TOLERANCE_STEP = 0.1;
    private static final double GUN_TURN_TOLERANCE_MAX = 3;

    private static final Rectangle2D.Double TARGET_AREA
            = new Rectangle2D.Double(20, 20, 760, 560);
    private static final int TARGET_AREA_PADDING = 150; // for wall smoothing

    private static final Random RANDOM = new Random();

    private static final Map<String, RobotData> enemies = new HashMap<>();
    private Point2D.Double myPos, myPreviousPos;
    private Map<Integer, TargetingMode> bullets = new HashMap<>();
    private Mode mode;
    private double gunTurnTolerance = 0.1;
    private TargetingMode lastTargetingMode = TargetingMode.values()[0];
    private int waveTickCounter = 0;

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

    public static double battlefieldWidth = 0;
    public static double battlefieldHeight = 0;
    private static int round = 0;

    // for debugging
    private MovementWave evilWaveJustHitUsReeee;

    // ------------ main functions ------------
    public void run() {
        battlefieldWidth = getBattleFieldWidth();
        battlefieldHeight = getBattleFieldHeight();
        round = getRoundNum();
        for (RobotData enemy : enemies.values()) {
            enemy.reset();
        }

        mode = getOthers() == 1 ? Mode.ONE_VS_ONE : Mode.MELEE;

        movementType = MovementType.TRUE_SURFING;
        stopAndGoEndTime = Long.MAX_VALUE;

        setAdjustRadarForGunTurn(true);
        setAdjustRadarForRobotTurn(true);

        while (true) {
            update();
            execute();
        }
    }

    @Override
    public void onRobotDeath(RobotDeathEvent event) {
        mode = getOthers() == 1 ? Mode.ONE_VS_ONE : Mode.MELEE;
        var enemy = enemies.get(event.getName());
        enemy.isDead = true;
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
        enemy.lastVelocity = enemy.velocity;
        enemy.velocity = event.getVelocity();
        enemy.scannedThisRound = true;

        // add heading, as getBearing() returns bearing relative to own heading
        double absoluteBearing = Math.toRadians((getHeading() + event.getBearing()) % 360);

        // track surfing data
        double rotationalVelocity = getVelocity() * Math.sin(event.getBearingRadians());
        enemy.surfDirections.addFirst(new RobotData.TimestampedEntry<>(enemy.lastSeen,
                rotationalVelocity >= 0 ? 1 : -1  // direction we are currently moving in, relative to enemy
        ));
        enemy.surfAngles.addFirst(new RobotData.TimestampedEntry<>(enemy.lastSeen,
                absoluteBearing + Math.PI
        ));

        // detect shot by energy drop
        if (enemy.energyLevels.size() > 2) { // we need at least two ticks of data
            double lastEnergy = enemy.energyLevels.getFirst().value;
            double energyDelta = lastEnergy - event.getEnergy();
            // TODO: account for gaps between scans and other energy fluctuation (idle?)
            if (0.0999 <= energyDelta && // valid energy cost for bullet is in [0.1, 3]
                    energyDelta <= 3.001) {
                // opponent (probably) shot bullet - create new wave
                enemy.gunHeat = calcGunHeat(energyDelta);

                double bulletVelocity = calcBulletVelocity(energyDelta);
                MovementWave wave = new MovementWave(
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
                enemy.movementWaves.add(wave);
            }
        }

        // track enemy energy
        enemy.energyLevels.addFirst(new RobotData.TimestampedEntry<>(enemy.lastSeen,
                event.getEnergy()
        ));

        // track enemy position
        enemy.pos = offsetPointByAngle(myPos, absoluteBearing, enemy.distance);

        onScannedRobotGun(event, enemy, absoluteBearing);
        onScannedRobotDoRadar(event, enemy, absoluteBearing);
    }

    enum TargetingMode {
                CALC,
        MOST_VISITED,
        SECOND_MOST_VISITED,
        LEAST_VISITED_GAP,
        GUESS_LEAST_VISITED_NEAREST_MOST_VISITED
    }

    private void onScannedRobotGun(ScannedRobotEvent event, RobotData enemy, double enemyAbsoluteBearing) {
        int lateralDirection = (int) Math.signum(event.getVelocity() * Math.sin(event.getHeadingRadians() - enemyAbsoluteBearing));
        double bulletPower = getBulletPower(event, enemy);
        double gunAdjust = 0.0;

        var targetingMode = enemy.getNextTargetingMode();

        Function<Integer, Double> getGunAdjustFromBinIndex = (Integer binIndex) -> {
            return Utils.normalRelativeAngle(enemyAbsoluteBearing - getGunHeadingRadians() + enemy.getBearingOffset(lateralDirection, binIndex));
        };

        switch (targetingMode) {
            case TargetingMode.CALC -> {
                gunAdjust = getCalculatedGunBearingAdjustment(event);
            }
            case TargetingMode.MOST_VISITED -> {
                var index = enemy.mostVisitedBinSmoothed();
                gunAdjust = getGunAdjustFromBinIndex.apply(index);
            }
            case TargetingMode.SECOND_MOST_VISITED -> {
                gunAdjust = getGunAdjustFromBinIndex.apply(enemy.nMostVisitedBin(2));;
            }
            case TargetingMode.LEAST_VISITED_GAP -> {
                gunAdjust = getGunAdjustFromBinIndex.apply(enemy.leastVisitedGapCenter());
            }
            case TargetingMode.GUESS_LEAST_VISITED_NEAREST_MOST_VISITED -> {
                gunAdjust = getGunAdjustFromBinIndex.apply(enemy.leastVisitedClosestToMostVisitedBin());
            }
        }

        setTurnGunRightRadians(gunAdjust);
        
        if (getEnergy() > GUN_MIN_SHOOTING_ENERGY && getGunHeat() == 0) {
            if (Math.abs(getGunTurnRemaining()) < gunTurnTolerance) {
                Bullet bullet = setFireBullet(bulletPower);
                lastTargetingMode = targetingMode;
                if (bullet != null) {
                    bullets.put(bullet.hashCode(), targetingMode);
                    enemy.bulletsFired++;
                    enemy.gunWaves.add(new GunWave(
                            enemy,
                            (Point2D.Double) myPos.clone(),
                            calcBulletVelocity(bulletPower),
                            enemyAbsoluteBearing,
                            lateralDirection,
                            true
                    ));

                    // remove old gun waves
                    List<GunWave> gunWavesToRemove = enemy.gunWaves.stream().filter(gw -> gw.canBeRemoved).toList();
                    enemy.gunWaves.removeAll(gunWavesToRemove);
                    var hitStat = enemy.hitStats.getOrDefault(targetingMode, new RobotData.HitStatValue());
                    hitStat.shot++;
                    enemy.hitStats.put(targetingMode, hitStat);

                    gunTurnTolerance = Math.max(GUN_TURN_TOLERANCE_STEP, gunTurnTolerance - GUN_TURN_TOLERANCE_STEP);
                }
            } else {
                gunTurnTolerance = Math.min(GUN_TURN_TOLERANCE_MAX, gunTurnTolerance + GUN_TURN_TOLERANCE_STEP);
            }
        } else {
            if (waveTickCounter == 10) {
                enemy.gunWaves.add(new GunWave(
                        enemy,
                        (Point2D.Double) myPos.clone(),
                        calcBulletVelocity(bulletPower),
                        enemyAbsoluteBearing,
                        lateralDirection,
                        false
                ));
                waveTickCounter = 0;
            }
            waveTickCounter++;
        }
    }

    private void onScannedRobotDoRadar(ScannedRobotEvent event, RobotData enemy, double absoluteBearing) {
        switch (mode) {
            case ONE_VS_ONE -> setTurnRadarRightRadians(Utils.normalRelativeAngle(absoluteBearing
                    - getRadarHeadingRadians()) * 2);
            case MELEE -> {
                // if not every enemy is known (this round): rotate fully
                if (enemies.values().stream().filter(r -> r.scannedThisRound && !r.isDead).count() < getOthers()) {
                    setTurnRadarRightRadians(Double.POSITIVE_INFINITY);
                    break;
                }

                // find most outdated enemy
                Optional<RobotData> oldest = enemies.values().stream()
                        .filter(r -> !r.isDead)
                        .min(Comparator.comparingLong(r -> r.lastSeen));
                if (oldest.isEmpty()) break;

                double targetAngle = calculateBearing(myPos, oldest.get().pos); // TODO: pray
                double targetAngleDelta = Utils.normalRelativeAngle(targetAngle - getRadarHeadingRadians());

                if (getOthers() > 1) {
                    // move in direction as fast as possible
                    setTurnRadarRightRadians(Math.signum(targetAngleDelta) * Double.POSITIVE_INFINITY);
                    break;
                }

                // only one opponent remaining: move towards them
                double anglePadding = Math.toRadians(12.5); // overshoot by 12.5 degrees to make sure we catch 'em
                setTurnRadarRightRadians(clamp(
                        -Math.PI / 4,
                        targetAngleDelta + anglePadding * Math.signum(targetAngleDelta),
                        Math.PI / 4)
                );
                mode = Mode.ONE_VS_ONE;
            }
        }
    }

    private void update() {
        // update previously hit list
        previousHitsWhileSurfing.addFirst(false);
        if (previousHitsWhileSurfing.size() > HIT_TIMESPAN) {
            previousHitsWhileSurfing.removeLast();
        }

        // update waves
        for (RobotData enemy : enemies.values()) {
            Iterator<MovementWave> wavesIterator = enemy.movementWaves.iterator();
            while (wavesIterator.hasNext()) {
                MovementWave wave = wavesIterator.next();
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

                if (timesPreviouslyHit >= HIT_THRESHOLD
                        && mode == Mode.ONE_VS_ONE) {
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

        // update gun stats
        for (RobotData enemy : enemies.values()) {
            enemy.update();
        }

        // radar stuff
        for (RobotData enemy : enemies.values()) {
            enemy.gunHeat -= getGunCoolingRate();
        }
        // help, we are not turning! :rage:
        if (getRadarTurnRemaining() == 0) {
            setTurnRadarRightRadians(Double.POSITIVE_INFINITY);
        }
    }

    private MovementWave getMostDangerousWave() {
        double closestDistance = Double.MAX_VALUE;
        MovementWave closestWave = null;

        for (RobotData enemy : enemies.values()) {
            for (MovementWave wave : enemy.movementWaves) {
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

    private static int getAngleBinForWaveHit(MovementWave wave, Point2D.Double hitAt) {
        double angleOffsetEnemyShotAt = Utils.normalRelativeAngle(calculateBearing(wave.center, hitAt) - wave.angle);
        double maxEscapeAngle = maxEscapeAngle(wave.velocity); // TODO: why does distance not matter?
        double enemyShotAngleFactor = angleOffsetEnemyShotAt / maxEscapeAngle;

        // return bin id for angle percentage of maxEscapeAngle chosen by enemy for shot
        return clamp(
                0,
                (int) (enemyShotAngleFactor * wave.direction * MOV_CENTER_BIN_ANGLE) + MOV_CENTER_BIN_ANGLE,
                MOV_NUM_BINS_ANGLE - 1
        );
    }

    private static int getDistanceBinForWaveHit(MovementWave wave, Point2D.Double hitAt) {
        double distance = wave.center.distance(wave.myPosOnShoot);
        int distanceBin = (int) Math.round(distance / MOV_DISTANCE_STEPS);
        return clamp(0, distanceBin, MOV_NUM_BINS_DISTANCE - 1);
    }

    private static int getFlighttimeBinForWave(MovementWave wave) {
        double distance = wave.center.distance(wave.myPosOnShoot) / (20 - 3 * wave.shotPower);
        int distanceBin = (int) Math.round(distance / MOV_FLIGHTTIME_STEPS);

        return clamp(0, distanceBin, MOV_NUM_BINS_FLIGHTTIME - 1);
    }

    private MovementWave getAndRemoveWaveThatHitMe(Bullet bullet) {
        for (RobotData enemy : enemies.values()) {
            Iterator<MovementWave> wavesIterator = enemy.movementWaves.iterator();
            while (wavesIterator.hasNext()) {
                MovementWave wave = wavesIterator.next();

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
        MovementWave evilWave = getAndRemoveWaveThatHitMe(bullet);
        evilWaveJustHitUsReeee = evilWave; // ...

        if (evilWave != null) {
            RobotData enemy = evilWave.robotData;
            int angleBucket = getAngleBinForWaveHit(evilWave, hitPos);
            int distanceBucket = getDistanceBinForWaveHit(evilWave, hitPos);
            int flighttimeBucket = getFlighttimeBinForWave(evilWave);

            for (int flighttimeI = 0; flighttimeI < MOV_NUM_BINS_FLIGHTTIME; flighttimeI++) {
                for (int distanceI = 0; distanceI < MOV_NUM_BINS_DISTANCE; distanceI++) {
                    for (int angleI = 0; angleI < MOV_NUM_BINS_ANGLE; angleI++) {
                        // increment buckets around hit with even distribution (1/2, 1/5, ...)
                        double oldValue = enemy.movementStats[flighttimeI][distanceI][angleI];
                        double newValue =
                                0.333 / (Math.pow(angleBucket - angleI, 2) + 1)
                                        + 0.333 / (Math.pow(distanceBucket - distanceI, 2) + 1)
                                        + 0.333 / (Math.pow(flighttimeBucket - distanceI, 2) + 1);

                        oldValue *= 0.95; // decay existing value -> prefer fresher data
                        if (oldValue < 0.00001) { // (0.95 ^ 200) * 0.33
                            oldValue = 0;
                        }
                        enemy.movementStats[flighttimeI][distanceI][angleI]
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
    private Point2D.Double predictInterception(MovementWave wave, int direction) {
        Point2D.Double result = new Point2D.Double(getX(), getY());
        double resultVelocity = getVelocity();
        double resultHeading = getHeadingRadians();

        int simulatedTicks = 0;
        boolean intercepted = false;
        do {
            // calculate desired angle (to check), 90 degrees to center of wave
            double ninetyDegAngle = calculateBearing(wave.center, result) + (direction * SLIGHTLY_BELOW_NINETY_DEG_RAD);

            // adjust angle to not run into walls
            double moveAngle = wallAndEnemySmoothing(result, ninetyDegAngle, direction) - resultHeading;

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

    private double getRiskForDirection(MovementWave wave, int direction) {
        // predict position for direction
        Point2D.Double predictedPosition = predictInterception(wave, direction);

        // determine bucket for this position,
        // and with that how risky such an angle is
        int angleBucket = getAngleBinForWaveHit(wave, predictedPosition);
        int distanceBucket = getDistanceBinForWaveHit(wave, predictedPosition);
        int flighttimeBucket = getFlighttimeBinForWave(wave);
        return wave.robotData.movementStats[flighttimeBucket][distanceBucket][angleBucket];
    }

    // surf by moving either left or right in relation to the center of the wave
//   - direction depends on which is less risky according to previous hits on the angle
//     the predicted position in the chosen direction would imply
    private void surf() {
        MovementWave waveToSurf = getMostDangerousWave();
        if (waveToSurf == null) return;

        // calculate risk for going left and right
        double risk1 = getRiskForDirection(waveToSurf, -1);
        double risk2 = getRiskForDirection(waveToSurf, 1);

        // calculate left or right move angle, depending on which is lower risk
        double waveAngle = calculateBearing(waveToSurf.center, myPos);
        int betterDirection = risk1 < risk2 ? -1 : 1;
        double moveAngle = wallAndEnemySmoothing(
                myPos,
                waveAngle + SLIGHTLY_BELOW_NINETY_DEG_RAD * betterDirection,
                betterDirection
        );
        moveAtAbsoluteAngle(moveAngle, 100); // TODO: tweak?
    }

    private void stopAndGo() {
        MovementWave waveToFool = getMostDangerousWave();
        if (waveToFool == null) return;

        ticksToJustGo--;
        if (ticksToJustGo > 0) return; // wait until updating movement again

        double distance = RANDOM.nextDouble(70, 120);
        double waveAngle = calculateBearing(waveToFool.center, myPos);
        double moveAngle = wallAndEnemySmoothing(
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

        onBulletHitGun(event, enemy);
    }


    private void onBulletHitGun(BulletHitEvent event, RobotData enemy) {
        var targetingMode = bullets.get(event.getBullet().hashCode());
        var v = enemy.hitStats.getOrDefault(targetingMode, new RobotData.HitStatValue());

        v.hit++;
        enemy.hitStats.put(targetingMode, v);
        enemy.hitsTotal++;

        out.println(enemy + " - hit with " + targetingMode.toString());
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
//        g.setColor(Color.green);
//        g.drawLine(
//                (int) getX(),
//                (int) getY(),
//                (int) (getX() + Math.sin(getHeading()) * 100),
//                (int) (getX() + Math.cos(getHeading()) * 100)
//        );
//        g.setColor(Color.red);
//        g.drawLine(
//                (int) getX(),
//                (int) getY(),
//                (int) (getX() + Math.sin(getGunHeading()) * 100),
//                (int) (getX() + Math.cos(getGunHeading()) * 100)
//        );

        g.setColor(Color.WHITE);
        enemies.values()
                .stream()
                .filter(r -> !r.isDead && r.scannedThisRound)
                .forEach(robot -> {
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
//            for (MovementWave wave : robot.movementWaves) {
//                // draw wave
//                g.drawOval(
//                        (int) (wave.center.x - wave.progress),
//                        (int) (wave.center.y - wave.progress),
//                        (int) (wave.progress * 2),
//                        (int) (wave.progress * 2)
//                );
//            }
                });

        enemies.values()
                .stream()
                .filter(r -> !r.isDead)
                .forEach(robot -> {
                    for (GunWave wave : robot.gunWaves) {
                        wave.print(g);
                    }
                });

        // most dangerous wave
//        MovementWave mostDangerousWave = getMostDangerousWave();
//        if (mostDangerousWave != null) {
//            g.setColor(Color.pink);
//            drawWave(mostDangerousWave, g);
//        }

        // evil wave (which just hit us :rage:)
//        if (evilWaveJustHitUsReeee != null) {
//            g.setColor(Color.GREEN);
//            drawWave(evilWaveJustHitUsReeee, g);
//        }

        // Movement Mode
        g.setColor(Color.WHITE);
        String movementText = "Using " + movementType;
        movementText += switch (movementType) {
            case STOP_AND_GO -> " for %d more ticks".formatted(stopAndGoEndTime - getTime());
            default -> " with %d hits".formatted(calcTimesPreviouslyHitWhileSurfing());
        };
        g.drawString(movementText, 10, 10);

        // Mode
        g.setColor(Color.WHITE);
        String modeText = "Mode: " + mode.toString();
        g.drawString(modeText, 10, 25);

        // Mode
        g.setColor(Color.WHITE);
        String targetingModeText = "Last Targeting Mode: " + lastTargetingMode.toString();
        g.drawString(targetingModeText, 10, 40);
    }

    private static void drawWave(MovementWave wave, Graphics2D g) {
        g.drawOval(
                (int) (wave.center.x - wave.progress),
                (int) (wave.center.y - wave.progress),
                (int) (wave.progress * 2),
                (int) (wave.progress * 2)
        );
    }

    static class MovementWave {
        RobotData robotData;
        Point2D.Double center = new Point2D.Double();
        Point2D.Double myPosOnShoot = new Point2D.Double();
        long startTicks;
        double velocity, angle /* from center to where we were when enemy fired */, progress;
        int direction /* we were moving when enemy fired */;
        double shotPower;

        public MovementWave(RobotData robotData, Point2D.Double center, Point2D.Double myPosOnShoot, long startTicks,
                            double velocity, double angle, int direction, double progress, double shotPower) {
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

    static class GunWave {
        RobotData robotData;
        Point2D.Double center = new Point2D.Double();
        double bulletVelocity;
        double progress;
        double bearingOffset /* to direct angle to enemy when we shot */;
        int lateralDirection /* of enemy towards us when we shot */;
        int distanceIndex;
        int velocityIndex;
        int lastVelocityIndex;
        boolean hasShot = false;
        boolean canBeRemoved = false;

        public GunWave(RobotData robotData, Point2D.Double center, double bulletVelocity,
                       double bearingOffset, int lateralDirection, boolean hasShot) {
            this.robotData = robotData;
            this.center = center;
            this.bulletVelocity = bulletVelocity;
            this.bearingOffset = bearingOffset;
            this.lateralDirection = lateralDirection;
            this.distanceIndex = robotData.getDistanceIndex();
            this.velocityIndex = robotData.getVelocityIndex();
            this.lastVelocityIndex = robotData.getLastVelocityIndex();
            this.hasShot = hasShot;
        }

        public void update() {
            progress += bulletVelocity;

            if (canBeRemoved) {
                return;
            }

            if (progress > NoobSlayer2500.battlefieldWidth) {
                canBeRemoved = true;
                return;
            }

            var angleIndex = -1;
            var currentMinDistance = Double.MAX_VALUE;
            for (int i = 0; i < GUN_NUM_BINS_ANGLE; i++) {
                double angle = bearingOffset + lateralDirection * (i - GUN_BINS_ANGLE_MIDDLE) * GUN_BIN_WIDTH_ANGLE;
                double radius = progress;
                Point2D.Double binPoint = offsetPointByAngle(center, angle, radius);

                var binEnemyDistance = robotData.pos.distance(binPoint);
                if (binEnemyDistance < 18 && binEnemyDistance < currentMinDistance) {
                    currentMinDistance = binEnemyDistance;
                    angleIndex = i;
                }
            }

            if (angleIndex >= 0) {
                int[] shotSmoothing = {2, 6, 12, 6, 2};
                int[] noShotSmoothing = {1, 3, 6, 3, 1};
                int[] smoothing = hasShot ? shotSmoothing : noShotSmoothing; // relative weights
                int centerOffset = 2; // position of the central value (corresponds to +6)
                for (int i = -2; i <= 2; i++) {
                    int targetAngleIndex = angleIndex + i;
                    if (targetAngleIndex >= 0 && targetAngleIndex < robotData.gunStats[distanceIndex][velocityIndex][lastVelocityIndex].length) {
                        robotData.gunStats[distanceIndex][velocityIndex][lastVelocityIndex][targetAngleIndex] += smoothing[i + centerOffset];
                    }
                }
            }
        }

        public void print(Graphics2D g) {

            int[] angleStats = robotData.gunStats[distanceIndex][velocityIndex][lastVelocityIndex];

            // Find max count for normalization
            int maxCount = 0;
            for (int count : angleStats) {
                if (count > maxCount) maxCount = count;
            }
            for (int i = 0; i < angleStats.length; i++) {
                double angle = bearingOffset + lateralDirection * (i - GUN_BINS_ANGLE_MIDDLE) * GUN_BIN_WIDTH_ANGLE;
                Point2D.Double binPoint = offsetPointByAngle(center, angle, progress);

                // Normalize intensity [0, 1]
                double intensity = 0;
                if (maxCount > 0) {
                    intensity = (double) angleStats[i] / maxCount;
                }

                // Convert intensity to color (heatmap style)
                Color color = getHeatmapColor(intensity);
                g.setColor(color);
                g.fillOval((int) binPoint.x - 3, (int) binPoint.y - 3, 6, 6);

            }
        }

        private static Color getHeatmapColor(double value) {
            value = Math.max(0.0, Math.min(1.0, value)); // Clamp between 0 and 1

            float hue = (float) (0.66 - (0.66 * value)); // 0.66 (blue) to 0.0 (red)
            float saturation = 1.0f;
            float brightness = 1.0f;

            return Color.getHSBColor(hue, saturation, brightness);
        }
    }

    static class RobotData {
        // movement stuff
        double[][][] movementStats = new double[MOV_NUM_BINS_FLIGHTTIME][MOV_NUM_BINS_DISTANCE][MOV_NUM_BINS_ANGLE];
        List<MovementWave> movementWaves = new ArrayList<>();
        List<RobotData.TimestampedEntry<Integer>> surfDirections = new LinkedList<>();
        List<RobotData.TimestampedEntry<Double>> surfAngles = new LinkedList<>();
        List<RobotData.TimestampedEntry<Double>> energyLevels = new LinkedList<>();

        // gun stuff
        private int[][][][] gunStats = new int[GUN_NUM_BINS_DISTANCE][GUN_NUM_BINS_VELOCITY][GUN_NUM_BINS_VELOCITY][GUN_NUM_BINS_ANGLE];
        List<GunWave> gunWaves = new ArrayList<>();
        HashMap<TargetingMode, HitStatValue> hitStats = Arrays.stream(TargetingMode.values()).collect(
                HashMap::new,
                (m2, v2) -> m2.put(v2, new HitStatValue()),
                HashMap::putAll
        );
        int bulletsFired;
        static int hitsTotal = 0;

        Point2D.Double pos = new Point2D.Double();
        double distance;
        long lastSeen;
        double heading;
        double velocity, lastVelocity;
        double gunHeat;

        boolean scannedThisRound;
        boolean isDead;

        public void reset() {
            scannedThisRound = false;
            isDead = false;

            velocity = lastVelocity = 0;
            lastSeen = 0;
            distance = 0;
            gunHeat = 3;

            movementWaves = new ArrayList<>();
            surfDirections = new LinkedList<>();
            surfAngles = new LinkedList<>();
            energyLevels = new LinkedList<>();

            gunWaves = new ArrayList<>();

            pos = new Point2D.Double();
        }

        static int ticks = 0;
        public void update() {
            for (GunWave gunWave : gunWaves) {
                gunWave.update();
            }

            if (ticks > 200) {
                for (int a = 0; a < gunStats.length; a++) {
                    for (int b = 0; b < gunStats[a].length; b++) {
                        for (int c = 0; c < gunStats[a][b].length; c++) {
                            for (int d = 0; d < gunStats[a][b][c].length; d++) {
                                gunStats[a][b][c][d] = Math.max(1, --gunStats[a][b][c][d]);
                            }
                        }
                    }
                }
                ticks = 0;
            }

        }

        public TargetingMode getNextTargetingMode() {

            var calcStat = hitStats.get(TargetingMode.CALC);

            if (calcStat.shot < 15 || calcStat.hit >= 5) {
                return TargetingMode.CALC;
            }

            return TargetingMode.MOST_VISITED;
        }

        private int rouletteWheel(int[] stats) {
            var weights = new HashMap<Integer, Integer>();

            for (var i = 0; i < stats.length; i++) {
                weights.put(i, 1 + stats[i]);
            }

            var totalWeight = weights.values().stream().mapToInt(Integer::intValue).sum();
            var randomValue = RANDOM.nextInt(totalWeight);

            for (var entry : weights.entrySet()) {
                randomValue -= entry.getValue();
                if (randomValue < 0) {
                    return entry.getKey();
                }
            }

            return RANDOM.nextInt(stats.length);
        }

        // Method to calculate the bearing offset where the target was most frequently encountered
        double getBearingOffset(int lateralDirection, int binIndex) {
            return (lateralDirection * GUN_BIN_WIDTH_ANGLE) * (binIndex - GUN_BINS_ANGLE_MIDDLE);
        }

        int getDistanceIndex() {
            return Math.min(GUN_NUM_BINS_DISTANCE - 1, (int) (distance / ((double) GUN_MAX_BULLET_DISTANCE / GUN_NUM_BINS_DISTANCE)));
        }

        int getVelocityIndex() {
            return (int) Math.abs(velocity / 2);
        }

        int getLastVelocityIndex() {
            return (int) Math.abs(lastVelocity / 2);
        }

        int[] getAngleStats() {
            return gunStats[getDistanceIndex()][getVelocityIndex()][getLastVelocityIndex()];
        }

        public int nMostVisitedBin(int n) {
            int[] angleStats = getAngleStats();
            PriorityQueue<Integer> pq = new PriorityQueue<>(Comparator.comparingInt(i -> angleStats[i]));

            for (int i = 0; i < angleStats.length; i++) {
                pq.offer(i);
            }

            int result = GUN_BINS_ANGLE_MIDDLE;
            int currentIndex = 0;
            do {
                var binIndex = pq.poll();
                if (binIndex == null)
                    return GUN_BINS_ANGLE_MIDDLE;

                if (angleStats[binIndex] <= 0)
                    continue;

                result = binIndex;
                currentIndex++;
            } while (currentIndex < n);

            return result;
        }

        public int leastVisitedGapCenter() {
            int[] angleStats = getAngleStats();
            PriorityQueue<List<Integer>> gaps = new PriorityQueue<>(new GapComparator(angleStats));

            var min = Arrays.stream(angleStats).min().orElse(0);
            var max = Arrays.stream(angleStats).max().orElse(0);

            int thresh = min + (int) (0.02 * max);

            List<Integer> currentList = new ArrayList<>();
            gaps.add(currentList);
            var isEdge = true;

            for (int i = 0; i < angleStats.length; i++) {

                if (isEdge && angleStats[i] <= thresh) {
                    continue;
                }

                isEdge = false;

                if (angleStats[i] <= thresh) {
                    currentList.add(i);
                } else if (!currentList.isEmpty()) {
                    currentList = new ArrayList<>();
                    gaps.add(currentList);
                }
            }

            if (angleStats[angleStats.length - 1] <= thresh) {
                gaps.remove(currentList);
            }

            var gap = gaps.poll();
            if (gap == null || gap.isEmpty()) {
                return nMostVisitedBin(1);
            }

            int centerIndex = gap.size() / 2;
            return gap.get(centerIndex);
        }

        private class GapComparator implements Comparator<List<Integer>> {

            private final int[] angleStats;

            GapComparator(int[] angleStats) {
                this.angleStats = angleStats;
            }

            @Override
            public int compare(List<Integer> o1, List<Integer> o2) {
                int sum1 = o1.stream().mapToInt(i -> angleStats[i]).sum();
                int sum2 = o2.stream().mapToInt(i -> angleStats[i]).sum();

                return -Integer.compare(sum1, sum2);
            }

            @Override
            public boolean equals(Object obj) {
                if (this == obj) return true;
                if (obj == null || getClass() != obj.getClass()) return false;
                GapComparator that = (GapComparator) obj;
                return Arrays.equals(angleStats, that.angleStats);
            }
        }

        public int mostVisitedBinSmoothed() {
            int[] angleStats = getAngleStats();
            return rouletteWheel(angleStats);
        }

        public int leastVisitedClosestToMostVisitedBin() {
            int[] angleStats = getAngleStats();
            int mostVisited = nMostVisitedBin(1);

            int leastVisited = -1;
            int minVisits = Integer.MAX_VALUE;
            int minDistance = Integer.MAX_VALUE;

            for (int i = 0; i < GUN_NUM_BINS_ANGLE; i++) {
                int visits = angleStats[i];
                int idxDistance = Math.abs(i - mostVisited);

                if (visits < minVisits || (visits == minVisits && idxDistance < minDistance)) {
                    leastVisited = i;
                    minVisits = visits;
                    minDistance = idxDistance;
                }
            }

            return leastVisited;
        }

        static class TimestampedEntry<T> {
            T value;
            long timestamp;

            public TimestampedEntry(long timestamp, T value) {
                this.timestamp = timestamp;
                this.value = value;
            }
        }

        public static class HitStatValue {
            public int hit;
            public int shot;

            public double getHitRatio() {
                if(shot == 0) return 1;
                return (double)hit / shot;
            }
        }
    }

    private enum MovementType {
        TRUE_SURFING,
        STOP_AND_GO
    }

    private enum Mode {
        ONE_VS_ONE, MELEE
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
// also considers enemies
    public double wallAndEnemySmoothing(Point2D.Double botLocation, double angle, int orientation) {
        // robot is 36x36 according to wiki
        final double collisionRadius = 100;
        int iterations = 0;

        while (++iterations < 100) {
            Point2D.Double wallDest = offsetPointByAngle(botLocation, angle, TARGET_AREA_PADDING);
            Point2D.Double enemyDest = offsetPointByAngle(botLocation, angle, 20);

            boolean targetAreaCheck = TARGET_AREA.contains(wallDest);
            boolean enemyBotCheck = true;
            for (RobotData enemy : enemies.values()) {
                if (enemy.isDead || !enemy.scannedThisRound) continue;

                Point2D.Double pos = enemy.pos;
                double left = pos.x - collisionRadius;
                double right = pos.x + collisionRadius;
                double top = pos.y - collisionRadius;
                double bottom = pos.y + collisionRadius;
                if (enemyDest.x > left && enemyDest.x < right
                        && enemyDest.y > top && enemyDest.y < bottom) {
                    enemyBotCheck = false;
                    break;
                }
            }

            if (targetAreaCheck && enemyBotCheck) break;
            angle += orientation * (enemyBotCheck ? 0.05 : mode == Mode.MELEE ? 0.2 : (Math.PI / 2));
        }
        return Utils.normalAbsoluteAngle(angle);
    }


    private double getBulletPower(ScannedRobotEvent e, RobotData enemy) {
        double bulletPower = 2.95;
        var maxEnemyEnergy = enemy.energyLevels.stream().max(Comparator.comparingDouble(x -> x.timestamp)).get().value;
        bulletPower = Math.min(bulletPower, (maxEnemyEnergy + 0.01) / 4);

        var minDist = e.getDistance();
        var botEnergy = getEnergy();

        if (minDist > 150 && getOthers() == 1)
            bulletPower = Math.min(bulletPower, 1.95);

        if (minDist > 150)
            bulletPower = Math.min(bulletPower, botEnergy / 20);
        else
            bulletPower = Math.min(bulletPower, botEnergy - 0.1);

        return Math.min(bulletPower, 1300 / minDist);
    }

    private double getCalculatedGunBearingAdjustment(ScannedRobotEvent e) {
        var enemy = enemies.get(e.getName());
        var bulletPower = getBulletPower(e, enemy);
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
        if (targetHeading < 0)
            targetHeading = targetHeading + 360;

        var delta = targetHeading - getGunHeading();

        return degreesToRadians(Utils.normalRelativeAngleDegrees(delta));
    }

    private double degreesToRadians(double angleInDegrees) {
        return angleInDegrees * Math.PI / 180;
    }

    // calculation from wiki
    private static double calcGunHeat(double bulletPower) {
        return 1.0 + (bulletPower / 5.0);
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
