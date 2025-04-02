package jk.neuromancer.melee;

import java.util.*;

import robocode.*;
import robocode.util.*;
import java.awt.geom.*;

import jk.neuromancer.math.FastTrig;

public class MeleeRadar {

    Hashtable<String,EnemyInfo> enemies = new Hashtable<String,EnemyInfo>();
    AdvancedRobot bot;
    Point2D.Double myLocation;
    public MeleeRadar(AdvancedRobot _bot){
        bot = _bot;

        enemies.clear();
        
    }
    public void onTick(){
        if(bot.getRadarTurnRemaining() == 0)
            bot.setTurnRadarRightRadians(Double.POSITIVE_INFINITY);
                
        myLocation =  new Point2D.Double(bot.getX(), bot.getY());   
    }

    public void onScannedRobot(ScannedRobotEvent e){   
        String eName = e.getName();
                            
        EnemyInfo eInfo;
        if((eInfo = enemies.get(eName)) == null){
            enemies.put(eName,eInfo = new EnemyInfo());
            eInfo.name = eName;
        }
            
        eInfo.lastScanTime = (int)bot.getTime();
        double otherAngle;// = getHeadingRadians() + e.getBearingRadians();
        eInfo.location = project(myLocation,otherAngle = bot.getHeadingRadians() + e.getBearingRadians(), e.getDistance());
            
            
            
        if(bot.getOthers() <= enemies.size()){
            Enumeration<EnemyInfo> all = enemies.elements();
            int oldestScan = eInfo.lastScanTime;
            while(all.hasMoreElements()){
                EnemyInfo tmp = all.nextElement();
                if(tmp.lastScanTime < oldestScan){
                otherAngle = absoluteAngle(myLocation,tmp.location);
                oldestScan = tmp.lastScanTime;
                }
            }
            if(bot.getOthers() == 1 && oldestScan == eInfo.lastScanTime){
                double angle = Utils.normalRelativeAngle(otherAngle - bot.getRadarHeadingRadians());
                bot.setTurnRadarRightRadians(Math.signum(angle)*limit(0,Math.abs(angle) + (Math.PI/4 - Math.PI/8 - Math.PI/18),Math.PI/4));
                
            }
            else
                bot.setTurnRadarRightRadians(Utils.normalRelativeAngle(otherAngle - bot.getRadarHeadingRadians())*Double.POSITIVE_INFINITY);
        }
        else
            bot.setTurnRadarRightRadians(Double.POSITIVE_INFINITY);
        
    }

    static  Point2D.Double project(Point2D.Double location, double angle, double distance){
        return new Point2D.Double(location.x + distance*FastTrig.sin(angle), location.y + distance*FastTrig.cos(angle));
    }
    static double absoluteAngle(Point2D source, Point2D target) {
        return FastTrig.atan2(target.getX() - source.getX(), target.getY() - source.getY());
    }

        
    public void onRobotDeath(RobotDeathEvent e){

        enemies.remove(e.getName());
    }

    public static double limit(double min, double value, double max) {
        if(value > max)
            return max;
        if(value < min)
            return min;
        
        return value;
    }

    class EnemyInfo{
        String name;
        int lastScanTime;
        Point2D.Double location = new Point2D.Double();
    }
}
