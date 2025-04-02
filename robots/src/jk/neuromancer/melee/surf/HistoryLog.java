package jk.neuromancer.melee.surf;

import java.util.*;
import java.awt.geom.*;
import jk.neuromancer.math.FastTrig;
	
public class HistoryLog{


    HashMap<String, TreeMap<Long, LogEntry>> data = new HashMap<>();
    HashMap<String, HashSet<LogListener>> listeners = new HashMap<>();
    HashMap<String, Long> deathTimes = new HashMap<>();

    public void put(String botName, long tick, Point2D.Double location, double heading, double velocity, double energy){
        TreeMap<Long, LogEntry> botData = data.get(botName);
        if(botData == null){
            botData = new TreeMap<>();
            data.put(botName, botData);
        }
        LogEntry entry = new LogEntry(location, heading, velocity, energy);
        botData.put(tick, entry);

        notifyNewData(botName);
    }
    public void onRobotDeath(String botName, long tick){
        deathTimes.put(botName, tick);
    }
    public void updateOnNewData(String botName, LogListener listener){
        HashSet<LogListener> botListeners = listeners.get(botName);
        if(botListeners == null){
            botListeners = new HashSet<>();
            listeners.put(botName, botListeners);
        }
        botListeners.add(listener);
    }
    public InterpolatedLogEntry getInterpolatedNearest(String botName, long ticks){
        LogPair pair = getNearest(botName, ticks);
        if (pair.after == null){
            if (pair.before == null)
                return null;
            else if (isAliveAt(botName, ticks))
                return new InterpolatedLogEntry(pair.before.location,
                                                pair.before.heading,
                                                pair.before.velocity,
                                                pair.before.energy,
                                                true);
            else
                return null;
        }
        else if (pair.before == null || pair.afterTime == ticks)
            return new InterpolatedLogEntry(pair.after.location,
                                            pair.after.heading,
                                            pair.after.velocity,
                                            pair.after.energy,
                                            false);
        else
        { //do an actual interpolation!
            double dx = pair.after.location.x - pair.before.location.x;
            double dy = pair.after.location.y - pair.before.location.y;
            double dv = pair.after.velocity - pair.before.velocity;
            double de = pair.after.energy - pair.before.energy;
            double dh = FastTrig.normalRelativeAngle(pair.after.heading - pair.before.heading);
            
            double dt = (ticks - pair.beforeTime)/(pair.afterTime - pair.beforeTime);
            Point2D.Double interpolatedLocation = new Point2D.Double(pair.before.location.x + dx*dt, pair.before.location.y + dy*dt);
            
            return new InterpolatedLogEntry(interpolatedLocation,
                                            FastTrig.normalAbsoluteAngle(pair.before.heading + dh*dt),
                                            pair.before.velocity + dv*dt,
                                            pair.before.energy + de*dt,
                                            false);
        }
    }
    
    
    
    public HashMap<String, InterpolatedLogEntry> getAllNearest(long ticks){
        HashMap<String, InterpolatedLogEntry> nearest = new HashMap<>();
        for(String botName : data.keySet()){
            InterpolatedLogEntry entry = getInterpolatedNearest(botName, ticks);
            if(entry != null)
                nearest.put(botName, entry);
        }
        
        return nearest;
    }
    
    boolean isAliveAt(String botName, long ticks){
        Long deathTime = deathTimes.get(botName);
        return deathTime == null || deathTime > ticks;
    }
    
    
    LogPair getNearest(String botName, long ticks){
        TreeMap<Long, LogEntry> botData = data.get(botName);
        if(botData == null)
            return new LogPair(null, null, -1, -1);

        long beforeTime = -1, afterTime = -1;
        LogEntry beforeEntry = null, afterEntry = null;
        
        Map.Entry<Long, LogEntry> after = botData.ceilingEntry(ticks);
        Map.Entry<Long, LogEntry> before = botData.lowerEntry(ticks);
        
        if (before != null){
            beforeEntry = before.getValue();
            beforeTime = before.getKey();
        }
        if (after != null){
            afterEntry = after.getValue();
            afterTime = after.getKey();
        }
            
        return new LogPair(beforeEntry, afterEntry, beforeTime, afterTime);
    }
    
    private void notifyNewData(String botName){
        HashSet<LogListener> botListeners = listeners.get(botName);
        if(botListeners != null){
            Iterator<LogListener> iter = botListeners.iterator();
            while(iter.hasNext())
            {
                iter.next().newData(botName);
                iter.remove();
            }
        }
    }
    public interface LogListener {
        public void newData(String botName);
    }
    
    public class LogEntry{
        Point2D.Double location;
        double heading; 
        double velocity; 
        double energy;
        
        LogEntry(Point2D.Double location, double heading, double velocity, double energy){
            this.location = location;
            this.heading = heading;
            this.velocity = velocity;
            this.energy = energy;
        }
    }
    public class InterpolatedLogEntry extends LogEntry{
        boolean waitingOnData;
        InterpolatedLogEntry(Point2D.Double location, double heading, double velocity, double energy, boolean waitingOnData){
            super(location, heading, velocity, energy);
            this.waitingOnData = waitingOnData;
        }
    }
    public class LogPair{
        LogEntry before, after;
        long beforeTime, afterTime;
        public LogPair(LogEntry before, LogEntry after, long beforeTime, long afterTime){
            this.before = before;
            this.after = after;
            this.beforeTime = beforeTime;
            this.afterTime = afterTime;
        }
    }

}