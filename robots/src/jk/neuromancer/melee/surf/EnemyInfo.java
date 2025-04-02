package jk.neuromancer.melee.surf;

import java.awt.geom.*;
import java.util.*;
import jk.neuromancer.tree.KDTree;
import jk.neuromancer.mega.BulletPowerPredictor;

public class EnemyInfo{
    String name;
    double heading, velocity;
    int lastScanTime;
    int lastHitByTime;
    double energy, lastEnergy, gunHeat, virtualGunHeat;
    Point2D.Double location;

    Hashtable<String,KDTree.WeightedManhattan<MeleeScan>> targets;
    KDTree.WeightedManhattan<MeleeScan> defaultAim;

    BulletPowerPredictor bulletPowerPredictor;

    double latVel, advVel, accel, distLast10, distToE, distToNearest, distToWall, distToCorner;
    int enemiesAlive, timeSinceReverse;

    double[] targetDescriptor(){
        double BFT = distToNearest/14;
        return new double[]{// value                       normalize    weighting
                        Math.abs(latVel)                     *(1/8.0)        *10,
                        (accel+2)                            *(1/3.)         *3,
                        (advVel + 8)                         *(1/16.0)       *2,
                        distLast10                           *(1/80.)        *2,
                        1./(1+2*timeSinceReverse/BFT)        *(1)            *2,
                        distToE                              *(1/1200.0)     *1,
                        distToNearest                        *(1/1200.0)     *2,
                        distToWall                           *(1/500.0)      *2,
                        1./(1.+enemiesAlive)                 *(1.)           *4,
                        distToCorner                         *(1/700.0)      *2
                        };
    }
}