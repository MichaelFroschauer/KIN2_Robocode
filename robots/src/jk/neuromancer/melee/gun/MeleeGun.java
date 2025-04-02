package jk.neuromancer.melee.gun;

import java.util. * ;
import java.awt.geom. * ;
import java.awt.Color;

import jk.neuromancer.tree.KDTree;
import jk.neuromancer.math.FastTrig;
import jk.neuromancer.melee.Neuromancer;

import robocode. * ;
import robocode.util. * ;

public class MeleeGun {

    public static final boolean TC = false;

    static Hashtable < String, EnemyInfo > enemies = new Hashtable < String, EnemyInfo > ();
    Neuromancer bot;
    Point2D.Double myLocation;
    double MAX_X, MAX_Y;
    boolean painting = false;

    ArrayList < Point2D.Double > targetPoints = new ArrayList < Point2D.Double > ();
    ArrayList < Point2D.Double > observePoints = new ArrayList < Point2D.Double > ();

    ArrayList < DelayWave > delayWaves = new ArrayList < DelayWave > ();
    double peakProbability = 0.1;
    long lastFireTime = -10000;

    public MeleeGun(Neuromancer _bot) {
        bot = _bot;

        MAX_X = bot.getBattleFieldWidth();
        MAX_Y = bot.getBattleFieldHeight();

        Enumeration < EnemyInfo > e = enemies.elements();
        while (e.hasMoreElements()) {
            EnemyInfo eInfo = e.nextElement();

            eInfo.alive = true;
            if (eInfo.lastScan != null){
                eInfo.lastScan.nearest = null;
                eInfo.lastScan = null;
            }
        }
    }
    public void onTick() {
        myLocation = new Point2D.Double(bot.getX(), bot.getY());
        Point2D.Double myNextLocation = project(myLocation, bot.getHeadingRadians(), bot.getVelocity());
        double maxEnemyEnergy = 0;
        double minDist = Double.POSITIVE_INFINITY;
        {
            Enumeration < EnemyInfo > en = enemies.elements();
            while (en.hasMoreElements()) {
                EnemyInfo ei = en.nextElement();
                if (!ei.alive || ei.lastScan == null) continue;

                maxEnemyEnergy = Math.max(ei.lastScan.energy, maxEnemyEnergy);
                minDist = Math.min(minDist, ei.lastScan.location.distance(myNextLocation));
            }
        }
        {
            int time = (int) bot.getTime();
            for (int i = 0; i < delayWaves.size(); i++) {
                DelayWave dw = delayWaves.get(i);
                EnemyInfo ei = enemies.get(dw.payload.name);

                double bulletDist = (time - dw.fireTime) * dw.velocity;

                boolean add = !ei.alive && bulletDist > 200;
                if (!add && ei.lastScan != null) {
                    double dist = ei.lastScan.location.distance(dw.fireLocation);
                    add = bulletDist > dist;
                }

                if (add) {
                    if (ei.infoTree == null)
                        ei.infoTree = new KDTree.Manhattan < EnemyScan > (new EnemyScan().treeLocation().length);
                    ei.infoTree.addPoint(dw.payload.treeLocation(), dw.payload);
                    delayWaves.remove(i);
                    i--;
                }

            }

        }
        if(bot.getOthers() == 1){
            double coolDelay = bot.getGunHeat() / bot.getGunCoolingRate();
                        Enumeration < EnemyInfo > en = enemies.elements();
            double absAngle = bot.getGunHeadingRadians();
            while (en.hasMoreElements()) {
                EnemyInfo ei = en.nextElement();
                if (!ei.alive || ei.lastScan == null) continue;

                absAngle = absoluteAngle(myNextLocation, ei.lastScan.location);
            }
            double maxGunTurn = Math.abs(FastTrig.normalRelativeAngle(absAngle - bot.getGunHeadingRadians())) + maxEscapeAngle(3);
            double minTurnRate = Rules.GUN_TURN_RATE_RADIANS - Rules.MAX_TURN_RATE_RADIANS;
            if (coolDelay > 1 + maxGunTurn/minTurnRate || bot.getEnergy() < 0.1){
                bot.setTurnGunRightRadians(FastTrig.normalRelativeAngle(absAngle - bot.getGunHeadingRadians()));
                return;
            }
        }

        double strongerEnemyPower = strongerEnemyBulletPower(20);
        double bulletPower = baseBulletPower(minDist, maxEnemyEnergy, bot.getEnergy(), strongerEnemyPower);
        for (int i = 0; ! TC && i < 3; i++) { //adapt bullet power for bots in GF +-1
            double maxAimEnergy = 0;
            double maxEscapeAngle = maxEscapeAngle(20 - 3 * bulletPower);
            double gunHeading = bot.getGunHeadingRadians();
            minDist = Double.POSITIVE_INFINITY;
            Enumeration < EnemyInfo > en = enemies.elements();
            while (en.hasMoreElements()) {
                EnemyInfo ei = en.nextElement();
                if (!ei.alive || ei.lastScan == null) continue;

                double gunOffset = Math.abs(
                Utils.normalRelativeAngle(
                gunHeading - absoluteAngle(myNextLocation, ei.lastScan.location)));

                if (gunOffset < maxEscapeAngle) {
                    maxAimEnergy = Math.max(ei.lastScan.energy, maxAimEnergy);
                    minDist = Math.min(minDist, ei.lastScan.location.distance(myNextLocation));
                }
            }
            double newBp = baseBulletPower(minDist, maxAimEnergy, bot.getEnergy(), strongerEnemyPower);
            if (Math.abs(newBp - bulletPower) < 0.001) break;
            bulletPower = newBp;
        }


        double[] aimScores = new double[2024];
        long[] aimMasks = new long[2024];
        simulateEnemyMoves(bulletPower, aimScores, aimMasks, myNextLocation);

        int maxIndex = (int) limit(0, (bot.getGunHeadingRadians() * aimScores.length * (1 / (2 * Math.PI))), aimScores.length - 1);

        for (int i = 0; i < aimScores.length; i++) {
            if (aimScores[i] > aimScores[maxIndex]) maxIndex = i;
        }
        //System.out.println(aimScores[maxIndex]);
        peakProbability = aimScores[maxIndex];
        long mask = aimMasks[maxIndex];
        Enumeration < EnemyInfo > en = enemies.elements();
        int j = -1;
        double maxAimEnergy = 0;
        minDist = Double.POSITIVE_INFINITY;
        while (en.hasMoreElements()) {
            j++;
            EnemyInfo eInfo = en.nextElement();
            if (eInfo.alive && eInfo.lastScan != null && (mask & (1l << j)) > 0) {
                minDist = Math.min(eInfo.lastScan.location.distance(myNextLocation), minDist);
                maxAimEnergy = Math.max(eInfo.lastScan.energy, maxAimEnergy);
            }
        }
        double newBp = baseBulletPower(minDist, maxAimEnergy, bot.getEnergy(), strongerEnemyPower);
        if (Math.abs(newBp - bulletPower) > 0.001) {
            bulletPower = newBp;
            Arrays.fill(aimScores, 0.);
            Arrays.fill(aimMasks, 0l);
            //          aimScores = new double[2024];
            //          aimMasks = new long[2024];
            simulateEnemyMoves(bulletPower, aimScores, aimMasks, myNextLocation);

            maxIndex = (int) limit(0, (bot.getGunHeadingRadians() * aimScores.length * (1 / (2 * Math.PI))), aimScores.length - 1);

            for (int i = 0; i < aimScores.length; i++) {
                if (aimScores[i] > aimScores[maxIndex]) maxIndex = i;
            }
            peakProbability = aimScores[maxIndex];
        }

        // Find the beginning and end of the peak, since maxIndex could be in the middle or on the side
        int lastIndex = maxIndex;
        int counter = aimScores.length;
        while (aimScores[lastIndex] >= aimScores[maxIndex] && counter-->0)
            lastIndex = (lastIndex + 1) % aimScores.length;
        lastIndex = (lastIndex + aimScores.length - 1) % aimScores.length;

        int firstIndex = lastIndex;
        counter = aimScores.length;
        while (aimScores[firstIndex] >= aimScores[maxIndex] && counter-->0)
            firstIndex = (firstIndex + aimScores.length - 1) % aimScores.length;
        firstIndex = (firstIndex + 1) % aimScores.length;

        double firstAngle = (firstIndex) * (2 * Math.PI) / aimScores.length;
        double lastAngle = (lastIndex) * (2 * Math.PI) / aimScores.length;
        double fireAngle = firstAngle + 0.5 * Utils.normalRelativeAngle(lastAngle - firstAngle);

        if (aimScores[maxIndex] > 0) {
            double gunHeading = bot.getGunHeadingRadians();
            int gunIndex = (int) Math.round(gunHeading / (2 * Math.PI) * (aimScores.length)) % aimScores.length;

            if ((aimScores[gunIndex] > 0.8 * aimScores[maxIndex] || (bot.getGunTurnRemaining() == 0 && aimScores[gunIndex] > 0))
            && (TC || bulletPower < bot.getEnergy() - 1) && bot.getGunHeat() == 0){
                Bullet b = bot.setFireBullet(bulletPower);
                if (b != null){
                    bot.bulletFired(b);
                    lastFireTime = bot.getTime();

                }
            }

            bot.setTurnGunRightRadians(Utils.normalRelativeAngle(fireAngle - bot.getGunHeadingRadians()));

        }

    }
    double baseBulletPower(double minDist, double maxEnemyEnergy, double botEnergy, double strongerEnemyBulletPower) {
        if (TC) return Math.min(botEnergy, 3);

        double bulletPower = 2.95;
        bulletPower = Math.min(bulletPower, (maxEnemyEnergy + 0.01) / 4);

        if (minDist > 150 && bot.getOthers() == 1) bulletPower = Math.min(bulletPower, 1.95);

        if (minDist > 150) bulletPower = Math.min(bulletPower, botEnergy / 20);
        //           bulletPower = Math.min(bulletPower, Math.max(25*0.1,botEnergy-20+0.5*limit(-10, botEnergy - maxEnemyEnergy ,30))/25.);
        else bulletPower = Math.min(bulletPower, botEnergy - 0.1);

        if (minDist > 350 && bot.getOthers() == 1)  bulletPower = Math.min(bulletPower, strongerEnemyBulletPower);

        bulletPower = Math.min(bulletPower, 1300 / minDist);

        return bulletPower;
    }
    double strongerEnemyBulletPower(long equaliseShots){
        // limit bullet power to what bots with more energy than me shoot, so we can out-survive them
        Hashtable<String, Double> enemyPowers = bot.predictAllBulletPowers();
        Enumeration < EnemyInfo > en = enemies.elements();
        double botEnergy = bot.getEnergy();
        double minPower = Double.POSITIVE_INFINITY;
        while (en.hasMoreElements()) {
            EnemyInfo ei = en.nextElement();
            if (!ei.alive || ei.lastScan == null) continue;
            if (ei.lastScan.energy > Math.max(botEnergy+10, botEnergy*1.2)) return Double.POSITIVE_INFINITY;
            //TODO: be smart about this, using projected hitrates. If there is not chance we win, try to get bullet damage
            double targetEnergyDiff = ei.lastScan.energy + 10 - botEnergy;
            if (targetEnergyDiff < 0) continue;
            Double power = enemyPowers.get(ei.name);
            if (power != null){
                double targetPower = limit(0.01, power - targetEnergyDiff/equaliseShots, 3);
                if (targetPower < minPower)
                    minPower = targetPower;
                    // System.out.println(ei.name + ": energy: " + ei.lastScan.energy + " bp:" + power + " target:" + targetPower);
            }
        }
        return minPower;
    }
    public void simulateEnemyMoves(double bulletPower, double[] aimScores, long[] aimMask, Point2D.Double myNextLocation) {
        double MEA = FastTrig.asin(8 / (20 - 3 * bulletPower));
        double bulletVelocity = 20 - 3 * bulletPower;

        Hashtable < EnemyInfo, Integer > enemyIndex = new Hashtable < EnemyInfo, Integer > ();
        Enumeration < EnemyInfo > en = enemies.elements();
        ArrayList < EnemyInfo > enemyList = new ArrayList < EnemyInfo > (enemies.size());
        double minDist = Double.POSITIVE_INFINITY;
        int k = 0;
        while (en.hasMoreElements()) {
            EnemyInfo testE = en.nextElement();
            if (testE.alive && testE.lastScan != null) {
                testE.distanceToMe = testE.lastScan.location.distance(myLocation);
                minDist = Math.min(testE.distanceToMe, minDist);
            }
            enemyIndex.put(testE, k);
            enemyList.add(testE);
            k++;
        }
        double others = bot.getOthers();
        double gunHeat = bot.getGunHeat();
        double[] botAimScores = new double[aimScores.length];

        Collections.sort(enemyList);

        targetPoints.clear();
        long time = bot.getTime();
        for (EnemyInfo testE: enemyList) {

            if (testE.alive && testE.lastScan != null && testE.infoTree != null) {
                EnemyScan s = testE.lastScan;
                long timeAdvance = time - s.time;
                if (s.nearest == null) {
                    s.nearest = testE.infoTree.nearestNeighbours(
                    s.treeLocation(), (int) limit(0, 400 / (others + 1), Math.min(50, Math.sqrt(testE.infoTree.size()))));
                }
                Arrays.fill(botAimScores, 0.0);
                double botWeight = 0;

                double startDist = myNextLocation.distance(s.location);
                double startAngle = absoluteAngle(myNextLocation, s.location);

                double coolDelay = Math.ceil(gunHeat / bot.getGunCoolingRate());
                double gunTurnDelay = Math.max(0, Math.ceil(( - MEA + Math.abs(Utils.normalRelativeAngle(startAngle - bot.getGunHeadingRadians())) / (Rules.GUN_TURN_RATE_RADIANS))));

                int pifHits = 0;
                if (s.energy == 0.0) {
                    //System.out.println("Waiting on gunheat in 1v1");
                    pifHits = s.nearest.size();
                    logHit(botAimScores, startAngle, 18.0 / startDist, 1);
                    botWeight = 1;
                }
                gunTurnDelay = Math.max(gunTurnDelay, coolDelay);

                double avgDist = 0;
                Iterator < KDTree.SearchResult < EnemyScan >> it = s.nearest.iterator();
                while (it.hasNext() && pifHits < s.nearest.size()) {
                    KDTree.SearchResult < EnemyScan > v = it.next();
                    avgDist += v.distance;
                }
                avgDist /= s.nearest.size();
                it = s.nearest.iterator();
                while (it.hasNext() && pifHits < s.nearest.size()) {
                    KDTree.SearchResult < EnemyScan > v = it.next();

                    EnemyScan replayStart = v.payload;
                    boolean velocityFlip = Math.signum(s.velocity) != Math.signum(replayStart.velocity);
                    boolean relativeFlip = Math.signum(s.latVelToNearest) != Math.signum(replayStart.latVelToNearest);

                    //double headingDiff = replayStart.heading - s.heading;
                    //double simRotate =  headingDiff + (velocityFlip?Math.PI:0);
                    // double simStartFireBearing = Math.PI + startAngle + simRotate;  
                    double theta = startAngle - s.heading;
                    double simStartFireBearing = replayStart.heading;

                    if (velocityFlip) simStartFireBearing += Math.PI;
                    if (relativeFlip) simStartFireBearing -= theta;
                    else simStartFireBearing += theta;

                    Point2D.Double observeLocation = project(
                    replayStart.location, simStartFireBearing, -startDist);

                    double radius;
                    double[] fastList = replayStart.fastList.array;

                    int listIndex = replayStart.listIndex;
                    double replayStartTime = fastList[listIndex + 2] + timeAdvance + gunTurnDelay - 1;
                    int maxIndex = Math.min(listIndex + 3 * 150, replayStart.fastList.size - 3);
                    double x, y, t;

                    do {
                        x = fastList[listIndex];
                        y = fastList[listIndex + 1];
                        t = fastList[listIndex + 2];

                        radius = (t - replayStartTime) * bulletVelocity;
                    } while ( observeLocation . distanceSq ( x , y ) > sqr(radius) && (listIndex += 3) < maxIndex);

                    if (listIndex >= maxIndex || listIndex < 3) continue;

                    //EnemyScan replayBefore = replay.previous;
                    double dt = t - fastList[listIndex - 3 + 2];
                    Point2D.Double simEndLocation;
                    if (dt > 1) {
                        double dx = (x - (x = fastList[listIndex - 3])) / dt;
                        //System.out.println("dx: " + dx);
                        double dy = (y - (y = fastList[listIndex - 3 + 1])) / dt;
                        //x = fastList[listIndex-3];
                        //y = fastList[listIndex-3+1];
                        radius = (fastList[listIndex - 3 + 2] - replayStartTime) * bulletVelocity;
                        int ticker = (int) dt + 1;
                        while (observeLocation.distanceSq(x, y) > sqr(radius) && ticker-->0) {
                            x += dx;
                            y += dy;
                            //fineTime++;
                            radius += bulletVelocity;
                        }

                        //simEndLocation = new Point2D.Double(x,y);
                    }
                    //else
                    simEndLocation = new Point2D.Double(x, y); //replay.location;
                    double simEndFireBearing = absoluteAngle(observeLocation, simEndLocation);
                    double endDist = observeLocation.distance(simEndLocation);
                    double endDiff = simEndFireBearing - simStartFireBearing;
                    if (relativeFlip) endDiff = -endDiff;

                    double endFireBearing = Utils.normalAbsoluteAngle(startAngle + endDiff);

                    Point2D.Double endLocation = project(myNextLocation, endFireBearing, endDist);

                    if (endLocation.x < 18 | endLocation.x > MAX_X - 18 | endLocation.y < 18 | endLocation.y > MAX_Y - 18) continue;

                    if (painting) targetPoints.add(endLocation);
                    // targetPoints.add(s.location);
                    double hitWidth = 36 / endDist;
                    double hitWeight = FastTrig.exp(-0.5*sqr(v.distance/avgDist));

                    botWeight += hitWeight;

                    logHit(botAimScores, endFireBearing, hitWidth, hitWeight);
                    pifHits++;

                }
                if (botWeight > 0) {

                    double distRatio = limit(1, startDist / Math.min(minDist, 150), 10);
                    double dataRatio = Math.min(pifHits, 10) * (1 / 10d);

                    //make the maximum possible height be 1, if they were all in the same place at dist 100
                    double dataWeight = dataRatio / (botWeight * distRatio);

                    long mask = 1l << enemyIndex.get(testE);

                    for (int i = 0; i < aimScores.length; i++)
                    if (botAimScores[i] != 0) {
                        //simple OR for each angle
                        aimScores[i] += dataWeight * botAimScores[i] * (1 - aimScores[i]);

                        //track which bots are at this angle
                        aimMask[i] |= mask;
                    }

                }
            }
        }

    }

    public void onScannedRobot(ScannedRobotEvent e) {
        myLocation = new Point2D.Double(bot.getX(), bot.getY());
        String eName = e.getName();

        EnemyInfo eInfo;
        if ((eInfo = enemies.get(eName)) == null) {
            enemies.put(eName, eInfo = new EnemyInfo());
            eInfo.name = eName;
        }
        eInfo.alive = true;

        EnemyScan et = new EnemyScan();
        if (eInfo.lastScan != null) {
            eInfo.lastScan.next = et;
            eInfo.lastScan.nearest = null;
            et.previous = eInfo.lastScan;
            // et.roundList = eInfo.lastScan.roundList;
            et.fastList = eInfo.lastScan.fastList;
        }
        else {
            // et.roundList = new ArrayList<EnemyScan>();
            et.fastList = new ContiguousDoubleArrayList();
        }
        eInfo.lastScan = et;
        et.listIndex = et.fastList.size;
        //et.roundList.add(et);
        et.time = bot.getTime();
        et.location = project(myLocation, bot.getHeadingRadians() + e.getBearingRadians(), e.getDistance());
        et.fastList.add(et.location.x).add(et.location.y).add(et.time);
        et.heading = e.getHeadingRadians();
        et.velocity = e.getVelocity();
        et.energy = e.getEnergy();
        et.enemiesAlive = bot.getOthers();
        et.round = bot.getRoundNum();

        long tenAgo = Math.max(0, et.time - 10);
        EnemyScan beforeTenAgo = et.previous,
        afterTenAgo = et;
        while (beforeTenAgo != null && beforeTenAgo.time > tenAgo) {
            afterTenAgo = beforeTenAgo;
            beforeTenAgo = beforeTenAgo.previous;
        }
        if (beforeTenAgo == null) et.distLast10 = et.location.distance(afterTenAgo.location);
        else {
            long beforeDiff = tenAgo - beforeTenAgo.time,
            afterDiff = afterTenAgo.time - tenAgo;
            long gap = beforeDiff + afterDiff;
            double br = beforeDiff / (double) gap,
            ar = 1 - br;
            Point2D.Double midPoint = new Point2D.Double(
            beforeTenAgo.location.x * br + afterTenAgo.location.x * ar, beforeTenAgo.location.y * br + afterTenAgo.location.y * ar);
            et.distLast10 = et.location.distance(midPoint);
        }
        Point2D.Double[] corners = new Point2D.Double[] {
            new Point2D.Double(0, 0),
            new Point2D.Double(0, MAX_Y),
            new Point2D.Double(MAX_X, 0),
            new Point2D.Double(MAX_X, MAX_Y)
        };
        double velX = et.velocity*FastTrig.sin(et.heading);
        double velY = et.velocity*FastTrig.cos(et.heading);
        
        et.distToWall = et.location.x;
        et.advVelToNearestWall = -velX;
        if (MAX_X - et.location.x < et.distToWall)
        {
            et.distToWall = MAX_X - et.location.x;
            et.advVelToNearestWall = velX;
        }
        if (et.location.y < et.distToWall)
        {
            et.distToWall = et.location.y;
            et.advVelToNearestWall = -velY;
        }
        if (MAX_Y - et.location.y < et.distToWall)
        {
            et.distToWall = MAX_Y - et.location.y;
            et.advVelToNearestWall = velY;
        }

        if (et.previous == null) eInfo.timeSinceReverse = 0;
        else if (Math.signum(et.velocity) != Math.signum(et.previous.velocity)) eInfo.timeSinceReverse = (int) Math.abs(et.velocity);
        //TODO: use a window on the decel side as well to make more accurate
        else eInfo.timeSinceReverse += (et.time - et.previous.time);


        et.timeToFromFire = Math.min(bot.getTime() - lastFireTime, bot.getGunHeat()/bot.getGunCoolingRate());
        
        et.timeSinceReverse = eInfo.timeSinceReverse;

        if (et.previous == null) {
            et.accel = 0;
            eInfo.timeSinceDecel = 0;
        }
        else {
            et.accel = (Math.abs(et.velocity) - Math.abs(et.previous.velocity)) / (et.time - et.previous.time);
            if (et.accel < 0) eInfo.timeSinceDecel = 0;
            else eInfo.timeSinceDecel += (et.time - et.previous.time);

            et.timeSinceDecel = eInfo.timeSinceDecel;
        }

        et.distToCorner = Double.POSITIVE_INFINITY;
        for (int i = 0; i < 4; i++)
        et.distToCorner = Math.min(et.location.distanceSq(corners[i]), et.distToCorner);
        et.distToCorner = Math.sqrt(et.distToCorner);

        EnemyInfo nearest = new EnemyInfo();
        nearest.lastScan = new EnemyScan();
        nearest.lastScan.location = myLocation;
        nearest.lastScan.energy = bot.getEnergy();
        double nearestDist = myLocation.distance(et.location);
        Enumeration < EnemyInfo > en = enemies.elements();
        while (en.hasMoreElements()) {
            EnemyInfo testE = en.nextElement();
            if (testE.alive && testE.lastScan != null && testE != eInfo) {
                double distance = et.location.distance(testE.lastScan.location);
                if (distance < nearestDist) {
                    nearestDist = distance;
                    nearest = testE;
                }
            }
        }
        et.distToNearest = nearestDist;
        double bearingFromNearest = absoluteAngle(nearest.lastScan.location, et.location);
        et.latVelToNearest = et.velocity * FastTrig.sin(et.heading - bearingFromNearest);
        et.advVelToNearest = et.velocity * FastTrig.cos(et.heading - bearingFromNearest);

        et.nearestEnergyRatio = et.energy / nearest.lastScan.energy;
        et.nearestEnemyEnergy = nearest.lastScan.energy;

        et.name = eName;

        DelayWave dw = new DelayWave();
        dw.velocity = 19;
        dw.fireLocation = myLocation;
        dw.fireTime = (int) bot.getTime();
        dw.payload = et;
        delayWaves.add(dw);
    }

    static Point2D.Double project(Point2D.Double location, double angle, double distance) {
        return new Point2D.Double(location.x + distance * FastTrig.sin(angle), location.y + distance * FastTrig.cos(angle));
    }
    static double absoluteAngle(Point2D source, Point2D target) {
        return FastTrig.atan2(target.getX() - source.getX(), target.getY() - source.getY());
    }
    public void onDeath(DeathEvent e) {
        endRound();
    }
    public void onWin(WinEvent e) {
        endRound();

    }
    void endRound() {

        int time = (int) bot.getTime();
        for (int i = 0; i < delayWaves.size(); i++) {
            DelayWave dw = delayWaves.get(i);
            EnemyInfo ei = enemies.get(dw.payload.name);
            if (ei == null) continue;

            double bulletDist = (time - dw.fireTime) * dw.velocity;

            boolean add = !ei.alive && bulletDist > 200;
            if (!add) {
                add = bulletDist > 200;
            }

            if (add) {
                if (ei.infoTree == null)
                    ei.infoTree = new KDTree.Manhattan < EnemyScan > (new EnemyScan().treeLocation().length);
                ei.infoTree.addPoint(dw.payload.treeLocation(), dw.payload);
                delayWaves.remove(i);
                i--;
            }

        }
        delayWaves.clear();

    }

    public void onRobotDeath(RobotDeathEvent e) {
        EnemyInfo ei = enemies.get(e.getName());
        if (ei != null) ei.alive = false;
    }

    public void onPaint(java.awt.Graphics2D g) {
        painting = true;

        g.setColor(Color.red);
        //System.out.println("targets: " + targetPoints.size());
        for (Point2D.Double p: targetPoints)
        g.drawRect((int) p.x - 18, (int) p.y - 18, 36, 36);

        g.setColor(Color.white);

        for (Point2D.Double p: observePoints)
        g.drawRect((int) p.x - 18, (int) p.y - 18, 36, 36);

        g.drawRect((int) myLocation.x - 18, (int) myLocation.y - 18, 36, 36);
    }

    static void logHit(double[] aimScores, double endFireBearing, double hitWidth, double hitWeight) {
        double halfWidth = hitWidth * 0.5;
        double lowAngle = Utils.normalAbsoluteAngle(endFireBearing - halfWidth),
        highAngle = Utils.normalAbsoluteAngle(endFireBearing + halfWidth);
        int lowIndex = ((int) Math.round(lowAngle * aimScores.length * (1.0 / (2 * Math.PI)))) % aimScores.length;
        int highIndex = ((int) Math.round(highAngle * aimScores.length * (1.0 / (2 * Math.PI)))) % aimScores.length;
        if (lowIndex <= highIndex) for (int i = lowIndex; i <= highIndex; i++) aimScores[i] += hitWeight;
        else {
            for (int i = lowIndex; i < aimScores.length; i++)
            aimScores[i] += hitWeight;
            for (int i = 0; i <= highIndex; i++)
            aimScores[i] += hitWeight;
        }
    }
    public static double limit(double min, double value, double max) {
        if (value > max) return max;
        if (value < min) return min;

        return value;
    }
    public static int limit(int min, int value, int max) {
        if (value > max) return max;
        if (value < min) return min;

        return value;
    }
    public static long limit(long min, long value, long max) {
        if (value > max) return max;
        if (value < min) return min;

        return value;
    }
    public static double sqr(double d) {
        return d * d;
    }
    public static double maxEscapeAngle(double velocity) {
        return FastTrig.asin(8.0 / velocity);
    }

    class Indice implements Comparable < Indice > {
        double position,
        height;
        public int compareTo(Indice o) {
            return (int) Math.signum(position - o.position);
        }
    }
    class DelayWave {
        int fireTime;
        Point2D.Double fireLocation;
        double velocity;

        EnemyScan payload;

    }
    class EnemyInfo implements Comparable < EnemyInfo > {
        boolean alive = true;
        String name;
        int timeSinceReverse, timeSinceDecel;

        EnemyScan lastScan;
        KDTree < EnemyScan > infoTree;
        double distanceToMe;

        public int compareTo(EnemyInfo o) {
            return (int) Math.signum(distanceToMe - o.distanceToMe);
        }
    }
    class ContiguousDoubleArrayList {
        double[] array;
        int size;
        ContiguousDoubleArrayList() {
            array = new double[300];
        }
        ContiguousDoubleArrayList(int size) {
            array = new double[size];
        }
        ContiguousDoubleArrayList add(double d) {

            if (size + 1 >= array.length) {
                //double[] newArray = new double[array.length*2];
                //System.arraycopy(array,0,newArray,0,size);
                array = Arrays.copyOf(array, array.length * 2);
            }

            array[size++] = d;
            //size += 1;
            return this;
        }

    }
    class EnemyScan {
        String name;
        //ArrayList<EnemyScan> roundList;
        ContiguousDoubleArrayList fastList;
        int listIndex;
        EnemyScan previous;
        EnemyScan next;

        List < KDTree.SearchResult < EnemyScan >> nearest;

        long time;
        Point2D.Double location;
        double heading;
        double velocity;
        double energy;

        double nearestEnemyEnergy;

        double distLast10;
        double distToWall;
        double distToCorner;
        double distToNearest;
        double latVelToNearest;
        double advVelToNearest;
        double advVelToNearestWall;
        double nearestEnergyRatio;
        double enemiesAlive;
        double accel;

        int timeSinceReverse;
        int timeSinceDecel;
        double timeToFromFire;
        long round;

        double[] treeLocation(){
            double BFT = distToNearest/14;
            return new double[]{
                /*value                   scaling to 1     weighting     genetic*/
                Math.abs(latVelToNearest)      *(1./8)        *10       *0.60,
                (advVelToNearest + 8)          *(1./16)       *2        *1.44,
                (accel+2)                      *(1./2)        *10       *0.16,
                distToWall                     *(1./500)      *5        *1.05,
                (advVelToNearestWall + 8)      *(1./16)       *5        *1.06,
                distToCorner                   *(1./700)      *2        *1.25,
                distToNearest                  *(1./1400)     *5        *0.96,
                distLast10                     *(1./80)       *3        *0.69,
                1.0/(1+2*timeSinceDecel/BFT)                  *3        *0.65,
                1.0/(1+2*timeSinceReverse/BFT)                *3        *0.89,
                1.0/(1.0+nearestEnergyRatio)                  *2        *0.6,
                1.0/(1.0+enemiesAlive)         *2             *10       *0.6,
                };
        }
    }
}