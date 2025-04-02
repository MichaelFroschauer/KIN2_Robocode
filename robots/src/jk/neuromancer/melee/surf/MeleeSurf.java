package jk.neuromancer.melee.surf;

import robocode.*;
import robocode.util.*;

import jk.neuromancer.math.FastTrig;
import jk.neuromancer.tree.KDTree;
import jk.neuromancer.mega.BulletPowerPredictor;

import java.util.*;
import java.awt.Color;
import java.awt.geom.*;

public class MeleeSurf{


    static  KDTree.WeightedManhattan<MeleeScan> GF_0_tree = new KDTree.WeightedManhattan<MeleeScan>(new EnemyInfo().targetDescriptor().length);
    static{
        GF_0_tree.addPoint(new double[new EnemyInfo().targetDescriptor().length], new MeleeScan());
    }
    static Hashtable<String,EnemyInfo> enemies = new Hashtable<String,EnemyInfo>();
    static Hashtable<String,EnemyInfo> deadEnemies = new Hashtable<String,EnemyInfo>();

    HistoryLog log = new HistoryLog();
    ArrayList<MeleeWave> waves = new ArrayList<MeleeWave>();
    ArrayList<Bullet> bullets = new ArrayList<Bullet>();
    AdvancedRobot bot;
    public static Rectangle2D.Double fieldRect;
    public static double MAX_X, MAX_Y;
    ArrayList<Point2D.Double> bestHitPoints;
    EnemyInfo me, lastMe, lastLastMe;
    ArrayList<Point2D.Double>[] hitPoints;
    Point2D.Double bestGoPoint;
    double[] dangers;
    ArrayList<PredictionPath> predictionPaths;


    public MeleeSurf(AdvancedRobot abot){
        bot = abot;

        enemies.putAll(deadEnemies);
        deadEnemies.clear();

        if(enemies.get(bot.getName()) == null){
            EnemyInfo me = new EnemyInfo();
            me.name = bot.getName();
            enemies.put(me.name,me);
        }

        Enumeration<EnemyInfo> e = enemies.elements();
        while(e.hasMoreElements()){
            EnemyInfo eInfo = e.nextElement();
            eInfo.lastScanTime = 0;
        }
        // System.out.println();
        MAX_X = bot.getBattleFieldWidth();
        MAX_Y = bot.getBattleFieldHeight();
        fieldRect = new Rectangle2D.Double(18,18,MAX_X - 36,MAX_Y - 36);
    }

    public void onTick(){
        lastMe = me;

        me = new EnemyInfo();

        me.location = new Point2D.Double(bot.getX(), bot.getY());
        me.heading = bot.getHeadingRadians();
        me.velocity = bot.getVelocity();
        me.energy = bot.getEnergy();
        me.lastScanTime = (int)bot.getTime();
        me.name = bot.getName();

        log.put(me.name, me.lastScanTime, me.location, me.heading, me.velocity, me.energy);

        if(lastMe != null)
            enemies.put(me.name,lastMe);
        else
            enemies.put(me.name,me);

        Point2D.Double[] corners = {new Point2D.Double(0,0), new Point2D.Double(0,MAX_Y),
                                    new Point2D.Double(MAX_X,0),  new Point2D.Double(MAX_X,MAX_Y)};
        long time = bot.getTime();

        //create any new gunheat waves
        if (bot.getOthers() == 1){
            double coolingRate = bot.getGunCoolingRate();
            Enumeration<EnemyInfo> gen = enemies.elements();
            while(gen.hasMoreElements()){
                EnemyInfo ei = gen.nextElement();
                if(ei.name.equals(me.name) || ei.lastScanTime == 0)
                    continue;
                double predictedGunheat = ei.virtualGunHeat - (time - ei.lastScanTime)*coolingRate;
                if (predictedGunheat <= 0.0001){
                    addWave(ei, ei.location, predictBulletPower(ei), ei.gunHeat, true);
                }
            }
        }
        // Remove old bullets
        Iterator<Bullet> bit = bullets.iterator();
        while(bit.hasNext()){
            Bullet b = bit.next();
            if (!b.isActive())
                bit.remove();
        }

        //update waves
        Iterator<MeleeWave> wit = waves.iterator();
        while(wit.hasNext()){
            MeleeWave mw = wit.next();
            if(me.location.distance(mw.fireLocation) - 18 < mw.bulletVelocity*(time - mw.fireTime))
                mw.surfable = false;

            boolean remove = true;
            if (time <= mw.fireTime + 2 || !mw.gunHeatWave){
                double radiusSq = sqr(mw.bulletVelocity*(time - mw.fireTime));
                Point2D.Double centre = mw.fireLocation;
                for (int i = 0; i < 4; i++)
                    if (radiusSq < centre.distanceSq(corners[i])){
                        remove = false;
                        break;
                    }
            }
            if (remove)
                wit.remove();
            else if(mw.surfable)
                mw.updateIfNecessary(log, me.location);

        }
        //somehow work out which movement option intersects all waves the safest....

        //generate directions to check distance/parallism with all bots as this is a first criteria
        double closestEnemy = Double.POSITIVE_INFINITY;
        Enumeration<EnemyInfo> clen = enemies.elements();
        while(clen.hasMoreElements()){
            EnemyInfo ei = clen.nextElement();
            if(ei == me || ei == lastMe)
                continue;
            double d = ei.location.distance(me.location);
            if(d < closestEnemy)
                closestEnemy = d;
        }

        if(bestGoPoint == null)
            bestGoPoint = me.location;

        //do some surfing!
        ArrayList<PredictionPath> possiblePaths = new ArrayList<PredictionPath>();

        int circularPaths = 16;

        double pointDist = limit(48,closestEnemy*8/14,160);
        double wallStick = Math.max(pointDist, 80);
        for(int i = 0; i < circularPaths; i++){
            double angle = (i)*2*Math.PI/(double)(circularPaths);
            if(!fieldRect.contains(project(me.location,angle,wallStick)))
                continue;

            PredictionPath ppath = new PredictionPath();
            ppath.gotoPoint = project(me.location,angle,pointDist);
            ppath.path = futureStatus(me.location,ppath.gotoPoint,me.velocity,me.heading);;

            possiblePaths.add(ppath);
        }

        // Add another set of paths, this time with destination points futher out
        pointDist = limit(148,closestEnemy*8/11,300);
        wallStick = limit(80, pointDist, 160);
        for(int i = 0; i < circularPaths; i++){
            double angle = (i+0.5)*2*Math.PI/(double)(circularPaths);
            if(!fieldRect.contains(project(me.location,angle,wallStick)))
                continue;

            PredictionPath ppath = new PredictionPath();
            ppath.gotoPoint = project(me.location,angle,pointDist);
            ppath.path = futureStatus(me.location,ppath.gotoPoint,me.velocity,me.heading);;

            possiblePaths.add(ppath);
        }

        for (PredictionPath path : possiblePaths)
        {
            Enumeration<EnemyInfo> en = enemies.elements();
            while(en.hasMoreElements()){
                EnemyInfo ei = en.nextElement();
                if(ei.name.equals(me.name))
                    continue;

                double dist = Math.max(0,path.gotoPoint.distance(ei.location)-30);
                double closest = 1;
                Enumeration<EnemyInfo> ens = enemies.elements();
                double minEDist = Double.POSITIVE_INFINITY;
                while(ens.hasMoreElements()){
                    EnemyInfo eni = ens.nextElement();
                    if(eni.name.equals(me.name) | eni == ei)
                        continue;
                    double eDist = eni.location.distance(ei.location);
                    if(eDist  < minEDist)
                        minEDist = eDist;
                    if(dist < eDist*1.2)
                        closest++;

                }
                closest = limit(1,closest - enemies.size() + 1, 3);
                if(new Line2D.Double(path.gotoPoint, me.location).ptSegDist(ei.location) < 1.3*minEDist){

                    double anglediff = absoluteBearing(me.location,path.gotoPoint) - absoluteBearing(ei.location,me.location);

                    path.enemyDanger += ei.energy/(dist*dist) * closest * (1 + Math.abs(Math.cos(anglediff)));
                }
                else{
                    path.enemyDanger += ei.energy/(dist*dist) * closest ;
                }
            }

            Iterator<MeleeWave> it = waves.iterator();
            while(it.hasNext()){
                MeleeWave w = it.next();

                if(
                !w.surfable
                ||
                me.location.distance(w.fireLocation) - 18 < w.bulletVelocity*(time - w.fireTime) )
                    continue;

                Point2D.Double hitPoint = futureStatus(path.path,time,w);

                if(!fieldRect.contains(hitPoint)){
                    path.waveDanger = Double.POSITIVE_INFINITY;
                    break;
                }

                double bearing = Utils.normalAbsoluteAngle(absoluteBearing(w.fireLocation,hitPoint));

                double botWidth = 40/w.fireLocation.distance(hitPoint);

                double scale = 180/Math.PI;
                double a1 = Utils.normalAbsoluteAngle(bearing - botWidth*0.5);
                double a2 = Utils.normalAbsoluteAngle(bearing + botWidth*0.5);
                double waveDanger =  botWidth*averageDanger((int)(scale*a1), (int)(scale*a2), w.bins, w.botShadowBins);

                double tth = me.location.distance(w.fireLocation)/w.bulletVelocity - (time - w.fireTime);
                double waveWeight = w.bulletDamage*Math.pow(0.8, tth);

                path.waveDanger += waveWeight*waveDanger;
            }

        }

        double[] enemyDangers = new double[possiblePaths.size()];
        double[] waveDangers = new double[possiblePaths.size()];

        for (int i = 0; i < possiblePaths.size(); i++){
            enemyDangers[i] = possiblePaths.get(i).enemyDanger;
            waveDangers[i] = possiblePaths.get(i).waveDanger;
        }

        medianNormalize(enemyDangers);
        medianNormalize(waveDangers);

        double minDanger = Double.POSITIVE_INFINITY;
        double[] dangerWeights = {limit(1e-5, bot.getOthers() - 1, 5), bot.getOthers()*2 + 1};

        dangers = new double[possiblePaths.size()];
        predictionPaths = possiblePaths;
        for(int i = 0; i < possiblePaths.size(); i++){
            dangers[i] =  dangerWeights[0]*enemyDangers[i] + dangerWeights[1]*waveDangers[i];

            if(dangers[i] < minDanger){
                minDanger = dangers[i];

                bestGoPoint = possiblePaths.get(i).gotoPoint;
            }
        }

       goTo(bestGoPoint);
    }

    public double averageDanger(int i1, int i2, double[] bins, double[] enableBins){
        double d = 0;
        if(i1 < i2){
            for(int i = i1; i <= i2; i++)
                d += bins[i]*enableBins[i];
            d /= i2 - i1 + 1;
        }
        else{
            for(int i = i1; i < bins.length; i++)
                d += bins[i]*enableBins[i];
            for(int i = 0; i <= i2; i++)
                d += bins[i]*enableBins[i];
            d /= bins.length - i1 + i2 + 1;
        }
        return d;
    }

    public static void areaNormalize(double[] bins){
        double total = 0;
        //double min = Double.POSITIVE_INFINITY;
        for(int i = 0; i < bins.length; i++){
            if(bins[i] != Double.POSITIVE_INFINITY){
                total += bins[i];
                //if(bins[i] < min)
                //min = bins[i];
            }
        }
        if(total != 0){
            total = 1/total;
            for(int i = 0; i < bins.length; i++)
                if(bins[i] != Double.POSITIVE_INFINITY){
                //bins[i] = (bins[i] - min)* total;
                bins[i] *= total;
                }
        }
    }

    public static void medianNormalize(double[] bins){
        double[] sortedBins = new double[bins.length];
        System.arraycopy(bins,0,sortedBins,0,bins.length);
        Arrays.sort(sortedBins);
        int minReal = 0;
        while(minReal < bins.length && sortedBins[minReal] == 0)minReal++;
        int maxReal = minReal;
        while(maxReal < bins.length && !Double.isInfinite(sortedBins[maxReal]))maxReal++;


        double scale = 1/(1e-30 + sortedBins[(int)limit(0,(minReal + maxReal)/2 - 1,bins.length - 1)]);

        for(int i = 0; i < bins.length; i++)
            if(bins[i] != Double.POSITIVE_INFINITY){
                //bins[i] = (bins[i] - min)* total;
                bins[i] *= scale;
            }
    }

    public static void normalize(double[] bins){
        double max = 0, min = Double.POSITIVE_INFINITY;
        for(int i = 0; i < bins.length; i++){
            if(bins[i] != Double.POSITIVE_INFINITY && bins[i] > max)
                max = bins[i];
            if(bins[i] < min)
                min = bins[i];
        }
        if(max != min){
            max = 1/(max-min);
            for(int i = 0; i < bins.length; i++)
                if(bins[i] != Double.POSITIVE_INFINITY)
                bins[i] = (bins[i]-min)*max;
        }
    }

    public void onPaint(java.awt.Graphics2D g){
        g.setColor(Color.green);
        g.drawOval((int)me.location.x - 20, (int)me.location.y - 20, 40,40);
        long time = bot.getTime();
        Iterator<MeleeWave> wit = waves.iterator();
        while(wit.hasNext()){
            MeleeWave w = wit.next();
//             if (!w.gunHeatWave)
//                 continue;
            if (!w.surfable)
                continue;
            w.updateIfNecessary(log, me.location);
            double bDist = w.bulletVelocity*(time - w.fireTime-1);

            g.setColor(Color.orange);


            // // for(int i = 0; i < w.bins.length; i++){
                // Point2D.Double p = project(w.startFiredFrom,i*(Math.PI*2), sDist);
                // g.drawOval((int)p.x - 1, (int)p.y - 1, 2,2);
            // }
            g.drawOval((int)w.fireLocation.x - (int)bDist, (int)w.fireLocation.y - (int)bDist, 2*(int)bDist, 2*(int)bDist);
            g.setColor(Color.white);


            EnemyInfo meInfoAtFire = w.snapshot.get(me.name);

            if(meInfoAtFire == null)
                System.out.println("me not in snapshot, snapshot size = " + w.snapshot.size());
            Point2D.Double meAtFire = meInfoAtFire.location;

            double GF0 = absoluteBearing(w.fireLocation, meAtFire);
            double MEA = maxEscapeAngle(w.bulletVelocity);
            double a1 = GF0 - MEA;
            double a2 = GF0 + MEA;

            int index1 = (int)((180/Math.PI)*a1 + 360)%360;
            int index2 = (int)((180/Math.PI)*a2 + 360)%360;

            if(index1 < index2){
                double maxD = 0;
                for(int i = 0; i < w.bins.length; i++){
                    if(w.bins[i]*w.botShadowBins[i] > maxD)
                        maxD = w.bins[i]*w.botShadowBins[i];
                    }
                    for(int i = index1; i < index2; i++){
                    double relDanger = w.bins[i]*w.botShadowBins[i]/maxD;

                    boolean paint = false;
                    if(relDanger == 1){
                        g.setColor(Color.red);
                        paint =true;}
                    else if(relDanger > 0.8){
                        g.setColor(Color.orange);
                        paint =true;}
                    else if(relDanger > 0.6){
                        g.setColor(Color.yellow);
                        paint =true;}
                    else if(relDanger > 0.4){
                        g.setColor(Color.green);
                        paint =true;}
                    else if(relDanger > 0.2)
                        g.setColor(Color.blue);
                    else
                        g.setColor(Color.black);
                    if (w.botShadowBins[i] == 0)
                        paint = true;
                    if(paint){
                        Point2D.Double p = project(w.fireLocation,i*Math.PI/180,bDist);
                        g.drawOval((int)p.x - 3, (int)p.y - 3, 6, 6);
                    }
                }
            }
            else{
                double maxD = 0;
                for(int i = 0; i < w.bins.length; i++){
                if(w.bins[i] > maxD)
                    maxD = w.bins[i];
                }
                for(int i = index1; i <w.bins.length; i++){
                double relDanger = w.bins[i]/maxD;
                boolean paint = false;
                if(relDanger == 1){
                    g.setColor(Color.red);
                    paint =true;}
                else if(relDanger > 0.8){
                    g.setColor(Color.orange);
                    paint =true;}
                else if(relDanger > 0.6){
                    g.setColor(Color.yellow);
                    paint =true;}
                else if(relDanger > 0.4){
                    g.setColor(Color.green);
                    paint =true;}
                else if(relDanger > 0.2)
                    g.setColor(Color.blue);
                else
                    g.setColor(Color.black);
                if(paint){
                    Point2D.Double p = project(w.fireLocation,i*Math.PI/180,bDist);
                    g.drawOval((int)p.x - 3, (int)p.y - 3, 6, 6);
                }
                }
                for(int i = 0; i <= index2; i++){
                double relDanger = w.bins[i]/maxD;

                boolean paint = false;
                if(relDanger == 1){
                    g.setColor(Color.red);
                    paint =true;}
                else if(relDanger > 0.8){
                    g.setColor(Color.orange);
                    paint =true;}
                else if(relDanger > 0.6){
                    g.setColor(Color.yellow);
                    paint =true;}
                else if(relDanger > 0.4){
                    g.setColor(Color.green);
                    paint =true;}
                else if(relDanger > 0.2)
                    g.setColor(Color.blue);
                else
                    g.setColor(Color.black);
                if(paint){
                    Point2D.Double p = project(w.fireLocation,i*Math.PI/180,bDist);
                    g.drawOval((int)p.x - 3, (int)p.y - 3, 6, 6);
                }               }
            }

        }
        double maxDanger = 0;
        for(int i = 0; i < dangers.length; i++)
            if(dangers[i] > maxDanger && dangers[i] != Double.POSITIVE_INFINITY)
                maxDanger = dangers[i];
        for(int i = 0; i < predictionPaths.size(); i++){
            double relDanger = dangers[i]/maxDanger;
            if(relDanger == Double.POSITIVE_INFINITY)
                g.setColor(Color.magenta);
            else if(relDanger == 1)
                g.setColor(Color.red);
            else if(relDanger > 0.8)
                g.setColor(Color.orange);
            else if(relDanger > 0.6)
                g.setColor(Color.yellow);
            else if(relDanger > 0.4)
                g.setColor(Color.green);
            else if(relDanger > 0.2)
                g.setColor(Color.blue);
            else
                g.setColor(Color.black);

            for(Point2D.Double pp : predictionPaths.get(i).path)
                g.drawOval((int)pp.x - 2, (int)pp.y - 2, 4, 4);
        }

        g.setColor(Color.white);
        if(hitPoints != null)
            for(int i = 0; i < hitPoints.length; i++)
                for(int j = 0; hitPoints[i] != null && j < hitPoints[i].size(); j++){
                    Point2D.Double p = hitPoints[i].get(j);
                    g.drawOval((int)p.x - 1, (int)p.y - 1, 2, 2);
                }

    }



    public void onScannedRobot(ScannedRobotEvent e){
        EnemyInfo eInfo = enemies.get(e.getName());
        double absBearing = e.getBearingRadians() + bot.getHeadingRadians();
        Point2D.Double newELocation = project(new Point2D.Double(bot.getX(), bot.getY()),absBearing,e.getDistance());

        if(eInfo == null){
            enemies.put(e.getName(), eInfo = new EnemyInfo());
            eInfo.name = e.getName();
            eInfo.virtualGunHeat = eInfo.gunHeat = bot.getGunHeat() - bot.getGunCoolingRate();
            eInfo.defaultAim = new KDTree.WeightedManhattan<MeleeScan>(new EnemyInfo().targetDescriptor().length);

        //    add a default GF0 shot
            MeleeScan ms = new MeleeScan();
            ms.GF = 0;
            ms.weight = 1e-20;
            eInfo.defaultAim.addPoint(new double[new EnemyInfo().targetDescriptor().length],ms);

            eInfo.targets = new Hashtable<String,KDTree.WeightedManhattan<MeleeScan>>();
            eInfo.heading = e.getHeadingRadians();
            eInfo.velocity = e.getVelocity();
            log.put(e.getName(), bot.getTime()-1, newELocation, eInfo.heading, eInfo.velocity, e.getEnergy());


            // set up nxn targeting
            Enumeration<EnemyInfo> gen = enemies.elements();
            while(gen.hasMoreElements()){
                EnemyInfo ei = gen.nextElement();
                eInfo.targets.put(ei.name,new KDTree.WeightedManhattan<MeleeScan>(new EnemyInfo().targetDescriptor().length));
                if (ei.name != me.name)
                    ei.targets.put(eInfo.name,new KDTree.WeightedManhattan<MeleeScan>(new EnemyInfo().targetDescriptor().length));
            }
        }
        else{
            if(eInfo.lastScanTime == 0)
                eInfo.gunHeat = eInfo.virtualGunHeat = bot.getGunHeat() - bot.getGunCoolingRate(); // 1 tick ago, so decayed

            double lastGunHeat = eInfo.gunHeat;
            double deltaE = eInfo.energy - e.getEnergy();
            long deltaT = Math.max(0,bot.getTime() -eInfo.lastScanTime);
            eInfo.heading = e.getHeadingRadians();
            eInfo.velocity = e.getVelocity();

            log.put(e.getName(), bot.getTime()-1, newELocation, eInfo.heading, eInfo.velocity, e.getEnergy());

            if(eInfo.lastScanTime != 0 && deltaT != 0 ){


                //check gunheat: if (gunheat> 0) fired = false;
                eInfo.gunHeat -= deltaT*bot.getGunCoolingRate();
                eInfo.virtualGunHeat -= deltaT*bot.getGunCoolingRate();
                boolean fired = eInfo.gunHeat <= -bot.getGunCoolingRate() + 0.0001; // by the time we see the drop

                // check for overlapping waves
                MeleeWave bestOverlap = getBestOverlapExcluding(newELocation,deltaE,eInfo.name,false);
                if (bestOverlap != null && Math.abs(deltaE-bestOverlap.bulletDamage) < 0.01){
                    logWaveHit(bestOverlap, newELocation, eInfo);
                }

                else
                if(
                        (bestOverlap == null || deltaE < bestOverlap.bulletDamage)
                        &&
                        fired && 0.01 < deltaE && deltaE <= 3 )//they fired!
                    addWave(eInfo,newELocation,deltaE, lastGunHeat, false);

                else{
                    bestOverlap = getBestOverlapExcluding(newELocation,deltaE,eInfo.name,true);

                    if(fired && bestOverlap != null &&  bestOverlap.bulletDamage < deltaE && deltaE <= bestOverlap.bulletDamage + 3){

                        addWave(eInfo,newELocation,deltaE - bestOverlap.bulletDamage, lastGunHeat, false);
                        logWaveHit(bestOverlap, newELocation, eInfo);
                    }

                    else if(bestOverlap == null);//can't find wave!
                }
            }

            //bot shadows... like bullet shadows but bigger!
            if(eInfo.location == null){
                eInfo.location = project(newELocation,e.getHeadingRadians(),-e.getVelocity());
                eInfo.lastScanTime = (int)bot.getTime() - 1;
            }
            int endTime = (int)bot.getTime();
            int interpolateTime = endTime - eInfo.lastScanTime + 1;
            Point2D.Double[] interpPoints = new Point2D.Double[interpolateTime];
            double startx = eInfo.location.x, starty = eInfo.location.y;
            double dx = (newELocation.x - startx)/interpolateTime;
            double dy = (newELocation.y - starty)/interpolateTime;
            for(int i = 0; i < interpolateTime; i++)
                interpPoints[i] = new Point2D.Double(startx + i*dx, starty + i*dy);
            for(MeleeWave w : waves){
                if(!w.firedBy.equals(eInfo.name))
                for(int i = 0; i < interpolateTime; i++){
                    Point2D.Double testPoint = interpPoints[i];
                    int testTime = eInfo.lastScanTime + i;
                    double testDist = testPoint.distance(w.fireLocation);
                    double waveRadius = w.bulletVelocity*(testTime - w.fireTime);
                    if(waveRadius > testDist + 18 ||
                    (i == 0 && waveRadius < testDist - 18 - (interpolateTime - i)*(8 + w.bulletVelocity)))
                        break;

                    if(waveRadius < testDist - 18)
                        continue;

                    //must be intersecting!
                    double angle = absoluteBearing(w.fireLocation,testPoint);
                    w.logShadow(angle, 36/testDist);

                }
            }
        }
        eInfo.lastEnergy = eInfo.energy;
        eInfo.energy = e.getEnergy();
        eInfo.lastScanTime = (int)bot.getTime();
        eInfo.location = newELocation;
    }
    public void onRobotDeath(RobotDeathEvent ev){
        EnemyInfo e = enemies.remove(ev.getName());
        if(e != null)
            deadEnemies.put(e.name,e);
        log.onRobotDeath(e.name, bot.getTime());
    }
    public void onHitByBullet(HitByBulletEvent ev){
        Bullet b = ev.getBullet();
        logBullet(b, true);
        EnemyInfo e = enemies.get(b.getName());
        if(e != null){
            e.energy += b.getPower()*3;
        }
    }

    public void onBulletHitBullet(BulletHitBulletEvent e){
        logBullet(e.getHitBullet(), false);
    }

    public void onBulletHit(BulletHitEvent ev){
        Bullet b = ev.getBullet();
        EnemyInfo e = enemies.get(ev.getName());
        if(e != null){
            double power = b.getPower();
            double damage = 4*power;
            if(power > 1)
                damage += 2*(power - 1);
            e.energy -= damage;
            double deltaE = e.energy - ev.getEnergy();
            double nowGunHeat = e.gunHeat - bot.getGunCoolingRate()*(bot.getTime() - e.lastScanTime);
            if(deltaE <= 3 && deltaE > 0.01 &&  nowGunHeat <= 0 && bot.getOthers() != 1){
                Point2D.Double newLocation = e.lastScanTime == bot.getTime() ? e.location : new Point2D.Double(b.getX(),b.getY());
                addWave(e,newLocation,deltaE, e.gunHeat, false);
            }
            e.energy -= Math.min(e.energy, damage);
        }
    }

    public void bulletFired(Bullet b){
        long time = bot.getTime();
        bullets.add(b);

        for (MeleeWave w : waves){
            Point2D.Double currentbP = new Point2D.Double(b.getX(), b.getY());
            double heading = b.getHeadingRadians(), velocity = b.getVelocity();

            w.logBulletForShadows(currentbP, heading, velocity, time);
        }
    }

    public double predictBulletPower(EnemyInfo firer){
        double closest = Double.POSITIVE_INFINITY;
        double closestEnergy = -1;

        if (firer.bulletPowerPredictor != null){
            Enumeration<EnemyInfo> ens = enemies.elements();
            while(ens.hasMoreElements()){
                EnemyInfo eni = ens.nextElement();
                if(eni.name.equals(firer.name) | eni == firer)
                    continue;
                double eDist = eni.location.distance(firer.location);
                if(eDist  < closest){
                    closest = eDist;
                    closestEnergy = eni.energy;
                }
            }
            return firer.bulletPowerPredictor.predictBulletPower(firer.energy, closestEnergy, closest);
        }
        return 2;
    }

    public Hashtable<String, Double> predictAllBulletPowers(){
        Hashtable<String, Double> powers = new Hashtable<String, Double>();
        Enumeration<EnemyInfo> ens = enemies.elements();
        while(ens.hasMoreElements()){
            EnemyInfo eni = ens.nextElement();
            if(eni.name.equals(me.name))
                continue;
            powers.put(eni.name, predictBulletPower(eni));
        }
        return powers;
    }

    public void addWave(EnemyInfo firer, Point2D.Double newELocation, double deltaE, double lastGunHeat, boolean gunHeatWave){

        long time = bot.getTime();
        if(!gunHeatWave){
            // remove any gunheat waves that this invalidates
            Iterator<MeleeWave> wit = waves.iterator();
            while(wit.hasNext()){
                MeleeWave mw = wit.next();
                if (mw.gunHeatWave && mw.firer.name.equals(firer.name) && mw.fireTime > firer.lastScanTime)
                        wit.remove();
            }
        }
        long gunDelay = (long)Math.max(0,Math.round(lastGunHeat/bot.getGunCoolingRate()));

        long latestFire = time - 2;
        long earliestFire = Math.min(firer.lastScanTime + gunDelay - 1, latestFire);

        if (gunHeatWave)
            firer.virtualGunHeat = Rules.getGunHeat(deltaE);
        else{
        
            double wallDistance = Math.min(Math.min(firer.location.x,MAX_X - firer.location.x),
                                            Math.min(firer.location.y,MAX_Y - firer.location.y)) - 18;
            long moveTime = latestFire - earliestFire + 2;
            double moveDist = 8.*Math.max(0.,moveTime-8.) + Math.min(8., moveTime)*(Math.min(8., moveTime)+1.)*0.5;
            if (moveDist < wallDistance)
                firer.virtualGunHeat = firer.gunHeat = Rules.getGunHeat(deltaE) - bot.getGunCoolingRate()*(time - earliestFire);
        }

        MeleeWave w = new MeleeWave();
        w.firedBy = firer.name;
        if (gunHeatWave)
            w.fireTime = time;
        else
            w.fireTime = (earliestFire + latestFire)/2;

        //System.out.println("Now: " + time + "  w.fireTime: " + w.fireTime + " gunheat wave: " + gunHeatWave);

        w.gunHeatWave = gunHeatWave;

        double travelTick = firer.location.distance(newELocation)/(time - firer.lastScanTime);
        double headingTick = absoluteBearing(firer.location,newELocation);
        Point2D.Double earliestLoc = project(firer.location,headingTick,travelTick*gunDelay);

        Point2D.Double latestLoc = project(newELocation, firer.heading, -firer.velocity);

        w.fireLocation =//
                    new Point2D.Double(0.5*(earliestLoc.x + latestLoc.x),
                    0.5*(earliestLoc.y + latestLoc.y));

        w.bulletPower = deltaE;
        w.bulletVelocity = 20 - 3*deltaE;

        w.bulletDamage = Rules.getBulletDamage(deltaE);

        //create a set of bins that cover 360 degrees
        w.bins = new double[360];
        w.botShadowBins = new double[360];
        Arrays.fill(w.botShadowBins, 1.);

        if(firer.targets == null)
            firer.targets = new Hashtable<String,KDTree.WeightedManhattan<MeleeScan>>();

        w.firer = new EnemyInfo();
        w.firer.location = (Point2D.Double)firer.location.clone();
        w.firer.energy = firer.energy;
        w.firer.velocity = firer.velocity;
        w.firer.heading = firer.heading;
        w.firer.name = firer.name;
        w.firer.targets = firer.targets;
        w.firer.defaultAim = firer.defaultAim;
        for(Bullet b : bullets){
            Point2D.Double currentbP = new Point2D.Double(b.getX(), b.getY());
            double heading = b.getHeadingRadians(), velocity = b.getVelocity();

            w.logBulletForShadows(currentbP, heading, velocity, time);
        }
        waves.add(w);
    }
    public void logWaveHit(MeleeWave bestOverlap, Point2D.Double newELocation, EnemyInfo eInfo){

        if(bestOverlap.needsSnapshotRebuild)
            bestOverlap.buildSnapshot(log);
                //log hit
                ////- find offset from who we think was being targeted (closest bot within GF +-1)
                //increase energy of targeter bot

        EnemyInfo firer = enemies.get(bestOverlap.firedBy);
        if(firer == null)
            firer = deadEnemies.get(bestOverlap.firedBy);
        if(firer == null)
            return;

        double closest = Double.POSITIVE_INFINITY;

        Enumeration<EnemyInfo> it = bestOverlap.snapshot.elements();
        while(it.hasMoreElements()){

            EnemyInfo firedTargetInfo = it.nextElement();
            double cdist = firedTargetInfo.location.distance(bestOverlap.fireLocation);
            if(cdist < closest)
                closest = cdist;
        }
        double inv_closest = 1/closest;

        it = bestOverlap.snapshot.elements();
        while(it.hasMoreElements()){

            EnemyInfo firedTargetInfo = it.nextElement();
            if (firedTargetInfo.name.equals(firer.name))
                continue;

            double distRatio = firedTargetInfo.location.distance(bestOverlap.fireLocation)*inv_closest;
            if(distRatio > 1.2)
                continue;

            double fireBearing = absoluteBearing(bestOverlap.fireLocation,
                            firedTargetInfo.location);
            double hitBearing = absoluteBearing(bestOverlap.fireLocation,
                            newELocation);

            double offset = Utils.normalRelativeAngle(hitBearing - fireBearing);

            double latVel = firedTargetInfo.velocity*FastTrig.sin(firedTargetInfo.heading - fireBearing);

            double GF = offset*Math.signum(latVel)/maxEscapeAngle(bestOverlap.bulletVelocity);
            if(GF > 1 || GF < -1)
                continue;

            if(firedTargetInfo.enemiesAlive == 0){
                System.out.println("tree location is null!");
                continue;
            }

            MeleeScan ms = new MeleeScan();
            ms.GF = GF;
            ms.weight = FastTrig.exp(-2*distRatio);

            KDTree.WeightedManhattan<MeleeScan> firerTree = firer.targets.get(eInfo.name);

            if(firerTree == null ){
                System.out.println("Found a missing firerTree!");
                firerTree = new KDTree.WeightedManhattan<MeleeScan>(new EnemyInfo().targetDescriptor().length);
                firer.targets.put(eInfo.name,firerTree);
            }
            firerTree.addPoint(firedTargetInfo.targetDescriptor(),ms);
            firer.defaultAim.addPoint(firedTargetInfo.targetDescriptor(),ms);

            if (firer.bulletPowerPredictor == null){
                firer.bulletPowerPredictor = new BulletPowerPredictor();
            }
            firer.bulletPowerPredictor.train(bestOverlap.firerEnergy, firedTargetInfo.energy, 
                                             closest,bestOverlap.bulletPower);
        }

        if(firer.energy - firer.lastEnergy - bestOverlap.bulletPower*3 < -0.01)
            firer.energy += bestOverlap.bulletPower*3;

        waves.remove(bestOverlap);
        recalcWaveDangersFor(firer.name);

    }
    void recalcWaveDangersFor(String name){
        for(MeleeWave w : waves){
            if(w.firedBy.equals(name))
                w.needsDangerRecalc = true;
        }

    }

    public void logBullet(Bullet b, boolean bulletHitMe){

        Point2D.Double bulletLocation = new Point2D.Double(b.getX(), b.getY());
        MeleeWave bestOverlap = getBestOverlapBy(bulletLocation, Rules.getBulletDamage(b.getPower()), b.getName());
        EnemyInfo eInfo = enemies.get(bot.getName());
        double closest = Double.POSITIVE_INFINITY;
        if(bestOverlap == null || bestOverlap.snapshot == null){
            System.out.println("Unknown bullet! time: " + bot.getTime() + " power:" + b.getPower() + " firedBy:" + b.getName());
            return;
        }
        if (bulletHitMe)
            bestOverlap.checkShadows(bulletLocation);

        if(bestOverlap.needsSnapshotRebuild)
            bestOverlap.buildSnapshot(log);

        Enumeration<EnemyInfo> it = bestOverlap.snapshot.elements();
        while(it.hasMoreElements()){

            EnemyInfo firedTargetInfo = it.nextElement();
            double cdist = firedTargetInfo.location.distance(bestOverlap.fireLocation);
            if(cdist < closest)
                closest =cdist;
        }
        double inv_closest = 1/closest;

        double hitBearing = b.getHeadingRadians();

        it = bestOverlap.snapshot.elements();
        while(it.hasMoreElements()){

            EnemyInfo firedTargetInfo = it.nextElement();
            if(firedTargetInfo.name.equals(bestOverlap.firedBy))
                continue;
            double distRatio = firedTargetInfo.location.distance(bestOverlap.fireLocation)*inv_closest;
            if(distRatio > 1.2)
                continue;

            double fireBearing = absoluteBearing(bestOverlap.fireLocation,
                            firedTargetInfo.location);

            double offset = Utils.normalRelativeAngle(hitBearing - fireBearing);

            double latVel = firedTargetInfo.velocity*FastTrig.sin(firedTargetInfo.heading - fireBearing);

            double GF = offset/maxEscapeAngle(bestOverlap.bulletVelocity);
            if(GF > 1 || GF < -1){
                // System.out.println("hitwave out of gf range for " + firedTargetInfo.name);
                continue;
            }
            EnemyInfo firer = enemies.get(bestOverlap.firedBy);
            if(firer == null)
                firer = deadEnemies.get(bestOverlap.firedBy);
            if(firer != null ){
                if(firer.targets == null)
                    firer.targets = new Hashtable<String,KDTree.WeightedManhattan<MeleeScan>>();
                KDTree.WeightedManhattan<MeleeScan> firerTree = firer.targets.get(eInfo.name);
                if(firerTree == null && firedTargetInfo.enemiesAlive != 0){
                    firerTree = new KDTree.WeightedManhattan<MeleeScan>(new EnemyInfo().targetDescriptor().length);
                    firer.targets.put(eInfo.name,firerTree);
                }
                if(firedTargetInfo.enemiesAlive == 0)
                    System.out.println("Fired Target Info NULL");
                MeleeScan ms = new MeleeScan();
                ms.GF = GF*Math.signum(latVel);
                ms.weight = FastTrig.exp(-2*distRatio);
                if(firedTargetInfo.enemiesAlive != 0)
                    firerTree.addPoint(firedTargetInfo.targetDescriptor(),ms);

                if(firer.defaultAim == null && firedTargetInfo.enemiesAlive != 0)
                    firer.defaultAim = new KDTree.WeightedManhattan<MeleeScan>(new EnemyInfo().targetDescriptor().length);
                if(firedTargetInfo.enemiesAlive != 0)
                    firer.defaultAim.addPoint(firedTargetInfo.targetDescriptor(),ms);
            }
            else
                System.out.println("Got wave but not firer...");
        }

        //System.out.println("firer: " + bestOverlap.firedBy);
        EnemyInfo firer = enemies.get(bestOverlap.firedBy);
        if(firer != null){
    //          if(firer.energy - firer.lastEnergy - bestOverlap.bulletPower*3 < -0.01)
            //firer = deadEnemies.get(bestOverlap.firedBy);
            firer.energy += bestOverlap.bulletPower*3;
        }

        waves.remove(bestOverlap);
        if(firer != null)
            recalcWaveDangersFor(firer.name);
    }
    MeleeWave getBestOverlapBy(Point2D.Double location, double bulletDamage, String name){


        MeleeWave bestOverlap = null;
        double dist = Double.POSITIVE_INFINITY;
        for(int i = 0, k = waves.size(); i < k; i++){
            MeleeWave overlap = waves.get(i);
            if(overlap.firedBy.equals(name)
            &&
            Math.abs(overlap.fireLocation.distance(location) -
                (bot.getTime() - overlap.fireTime)*overlap.bulletVelocity)
                            < 8*overlap.bulletVelocity
                && (bot.getOthers() != 1 || sqr(overlap.bulletDamage - bulletDamage) < 0.01)
                ){
                //rank overlapping waves in order of likelyhood, incorporating:
                ////- how near deltaE is to this wave's damage (mostly)
                ////- how long the wave had to travel (less is better) (weighted just as tie-breaker)
                        ////TODO: use all, and eliminate the ones whose bots do not get a hit bonus
                double thisDist =  sqr(overlap.bulletDamage - bulletDamage) + (bot.getTime() - overlap.fireTime)*0.001;
                if(thisDist < dist){
                    bestOverlap = overlap;
                    dist = thisDist;
                }

            }
        }
        return bestOverlap;

    }
    MeleeWave getBestOverlapExcluding(Point2D.Double location, double bulletDamage, String excludeName, boolean allow1Fire){


        MeleeWave bestOverlap = null;
        double dist = Double.POSITIVE_INFINITY;
        for(int i = 0, k = waves.size(); i < k; i++){
            MeleeWave overlap = waves.get(i);
            if(!overlap.firedBy.equals(excludeName)
            &&
            Math.abs(overlap.fireLocation.distance(location) -
                (bot.getTime() - overlap.fireTime)*overlap.bulletVelocity)
                            < 8*overlap.bulletVelocity
                && (bot.getOthers() != 1 || sqr(overlap.bulletDamage - bulletDamage) < 0.01)
                ){
                //rank overlapping waves in order of likelyhood, incorporating:
                ////- how near deltaE is to this wave's damage (mostly)
                ////- how long the wave had to travel (less is better) (weighted just as tie-breaker)
                        ////TODO: use all, and eliminate the ones whose bots do not get a hit bonus
                double thisDist =

                ((0 < (bulletDamage - overlap.bulletDamage)
                && (bulletDamage - overlap.bulletDamage) <= 3
                && allow1Fire)
                ?0:Math.abs(bulletDamage - overlap.bulletDamage))

                + (bot.getTime() - overlap.fireTime)*0.01;
                if(thisDist < dist){
                    bestOverlap = overlap;
                    dist = thisDist;
                }

            }
        }
        return bestOverlap;

    }

    public static void smoothAround(double[] bins, int index, int width, double weight){

        int minIndex = (index - 2*width + 2*bins.length)%bins.length;
        int maxIndex = (index + 2*width                )%bins.length;
        double invWidth = 1.0/width;

        if(minIndex > index){
            for(int i = minIndex; i < bins.length ; i++)
                bins[i] += weight/(sqr(i - bins.length - index)*invWidth + 1);

            for(int i = 0; i < index; i++)
                bins[i] += weight/(sqr(i - index)*invWidth + 1);
        }
        else
            for(int i = minIndex; i < index; i++)
                bins[i] += weight/(sqr(i - index)*invWidth + 1);

        if(maxIndex < index){
            for(int i = index; i < bins.length; i++)
                bins[i] += weight/(sqr(i - index)*invWidth + 1);

            for(int i = 0; i <= maxIndex; i++)
                bins[i] += weight/(sqr(i + bins.length - index)*invWidth + 1);
        }
        else
            for(int i = index; i <= maxIndex; i++)
                bins[i] += weight/(sqr(i - index)*invWidth + 1);


    }
    public static int sqr(int i){
        return i*i;}
    public static double sqr(double d){
        return d*d;}

    static class PredictionStatus{
        double  finalHeading, finalVelocity, distanceRemaining;
        long time;
        Point2D.Double endPoint;
        boolean debug;
    }

    class PredictionPath
    {
        Point2D.Double gotoPoint;
        public ArrayList<Point2D.Double> path;
        double waveDanger;
        double enemyDanger;
    }

            //3 optimized methods from the new robocode engine
    private static double getNewVelocity(double velocity, double distance) {
        final double goalVel = Math.min(getMaxVelocity(distance), 8);

        if(velocity >= 0)
            return limit(velocity - 2,
                goalVel, velocity + 1);

        return limit(velocity - 1,
            goalVel, velocity + maxDecel(-velocity));
    }

    final static double getMaxVelocity(double distance) {
        final double decelTime =  Math.max(1,Math.ceil(
            //sum of 0... decelTime, solving for decelTime
            //using quadratic formula, then simplified a lot
            Math.sqrt(distance + 1) - 0.5));

        final double decelDist = (decelTime) * (decelTime-1) ;
            // sum of 0..(decelTime-1)
            // * Rules.DECELERATION*0.5;

        return ((decelTime - 1) * 2) + ((distance - decelDist) / decelTime);
    }

    private static final double maxDecel(double speed) {
        return limit(1,speed*0.5 + 1, 2);
    }
    ArrayList<Point2D.Double> futureStatus(Point2D.Double fromLocation, Point2D.Double toLocation, double initialVelocity, double initialHeading){
        ArrayList<Point2D.Double> path = new ArrayList<Point2D.Double>();
        double bearing = absoluteBearing(fromLocation,toLocation);
        double velocity = initialVelocity;
        double distanceRemaining = fromLocation.distance(toLocation);;
        //long time = currentTime - wave.fireTime;
        double heading = initialHeading;

        Point2D.Double endPoint = (Point2D.Double)fromLocation.clone();
        int counter = 91;//5 + (int)Math.ceil(endPoint.distance(wave.fireLocation)/(wave.bulletVelocity-8)) - time;;
        double sinVal = 0, cosVal=0;
        boolean inline = false;
        do{

            if(!inline && (distanceRemaining > 1 | Math.abs(velocity) > 0.1))
            {
                double maxTurn = Math.PI/18 - Math.PI/240*Math.abs(velocity);
                bearing = absoluteBearing(endPoint,toLocation);
                double offset = FastTrig.normalRelativeAngle(bearing - heading);
                if(-Math.PI/2 > offset | offset > Math.PI/2){
                    offset = FastTrig.normalRelativeAngle(offset + Math.PI);
                    velocity = -velocity;
                    heading += Math.PI;
                }
                offset = limit(-maxTurn,offset,maxTurn);
                heading += offset;
                sinVal = FastTrig.sin(heading);
                cosVal = FastTrig.cos(heading);
                if(-0.0001 < offset & offset < 0.0001)
                    inline = true;
            }

            velocity = getNewVelocity(velocity, distanceScale(distanceRemaining, FastTrig.normalRelativeAngle(bearing - heading)));

            endPoint.x += sinVal*velocity;
            endPoint.y += cosVal*velocity;

            path.add((Point2D.Double)endPoint.clone());

            if(!fieldRect.contains(endPoint))
                break;

            if(velocity > distanceRemaining)
                inline = false;
            if(inline)
                distanceRemaining = Math.abs(distanceRemaining - velocity);
            else
                distanceRemaining = endPoint.distance(toLocation);

        } while( (Math.abs(distanceRemaining) > 0.1 || Math.abs(velocity) > 0.1) & --counter != 0 );

        return path;
    }

    public static Point2D.Double futureStatus(ArrayList<Point2D.Double> path, long currentTime, MeleeWave wave){
        long time = currentTime - wave.fireTime;
        int i = 0;
        int max = path.size();
        Point2D.Double endPoint;
        do{
            endPoint = path.get(i++);
            time++;

        } while((endPoint.distanceSq(wave.fireLocation) > sqr(wave.bulletVelocity*(time))) & i < max);

        return endPoint;
    }

  private void goTo(Point2D.Double place) {
      double distance = me.location.distance(place);
      double dir = 1;
      double angle = FastTrig.normalRelativeAngle(absoluteBearing(me.location, place) - bot.getHeadingRadians());
      if(-1 < distance & distance < 1 )
         angle = 0;

      if (Math.abs(angle) > Math.PI/2) {
         dir = -1;
         if (angle > 0) {
            angle -= Math.PI;
         }
         else {
            angle += Math.PI;
         }
      }

      bot.setTurnRightRadians(angle);
      bot.setAhead(distanceScale(distance, angle)*dir);
   }

   private double distanceScale(double oldDistance, double changeAngle)
   {
       return Math.min(sqr(sqr(Math.cos(changeAngle)))*19, oldDistance);
   }

    // CREDIT: from CassiusClay, by PEZ
    //   - returns point length away from sourceLocation, at angle
    // robowiki.net?CassiusClay
    public static Point2D.Double project(Point2D.Double sourceLocation, double angle, double length) {
        return new Point2D.Double(sourceLocation.x + FastTrig.sin(angle) * length,
                sourceLocation.y + FastTrig.cos(angle) * length);
    }

    // got this from RaikoMicro, by Jamougha, but I think it's used by many authors
    //  - returns the absolute angle (in radians) from source to target points
    public static double absoluteBearing(Point2D.Double source, Point2D.Double target) {
        return FastTrig.atan2(target.x - source.x, target.y - source.y);
    }


    public static double limit(double min, double value, double max) {
        if(value > max)
            return max;
        if(value < min)
            return min;

        return value;
    }

    public static double bulletVelocity(double power) {
        return (20D - (3D*power));
    }

    public static double maxEscapeAngle(double velocity) {
        return FastTrig.asin(8.0/velocity);
    }

    static double rollingAverage(double value, double newEntry, double depth, double weighting ) {
        return (value * depth + newEntry * weighting)/(depth + weighting);
    }

    public static int getIndex(double[] slices, double value){
        int index = 0;
        while(index < slices.length && value >= slices[index])
            index++;
        return index;
    }


    //CREDIT: Simonton
    static double HALF_PI = Math.PI / 2;
    static double WALL_MARGIN = 18;

    // eDist  = the distance from you to the enemy
    // eAngle = the absolute angle from you to the enemy
    // oDir   =  1 for the clockwise orbit distance
    //          -1 for the counter-clockwise orbit distance
    // returns: the positive orbital distance (in radians) the enemy can travel
    //          before hitting a wall (possibly infinity).
    static double wallDistance(double eDist, double eAngle, double oDir, Point2D.Double fireLocation) {

        return Math.min(Math.min(Math.min(
            distanceWest(MAX_Y - WALL_MARGIN - fireLocation.y, eDist, eAngle - HALF_PI, oDir),
            distanceWest(MAX_X - WALL_MARGIN - fireLocation.x, eDist, eAngle + Math.PI, oDir)),
            distanceWest(fireLocation.y - WALL_MARGIN, eDist, eAngle + HALF_PI, oDir)),
            distanceWest(fireLocation.x - WALL_MARGIN, eDist, eAngle, oDir));
    }

    static double distanceWest(double toWall, double eDist, double eAngle, double oDir) {
        if (eDist <= toWall) {
            return Double.POSITIVE_INFINITY;
        }
        double wallAngle = FastTrig.acos(-oDir * toWall / eDist) + oDir * HALF_PI;
        return Utils.normalAbsoluteAngle(oDir * (wallAngle - eAngle));
    }

}
