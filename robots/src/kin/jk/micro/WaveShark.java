package jk.micro;
import robocode.*;
import robocode.util.Utils;
import java.util.*;
import java.awt.geom.*;

/**
 * WaveShark, by Kevin Clark (Kev).
 * Micro-sized bot with wave surfing
 * See http://robowiki.net/cgi-bin/robowiki?WaveShark for more information.
 */
public class WaveShark extends AdvancedRobot {
    static final int HIT_FACTORS = 31;
    static final int MIDDLE_FACTOR = (HIT_FACTORS - 1) / 2;

    static double enemyEnergy;
    static double lastVelocity;
    static Point2D.Double myLocation;

    static double[][] hits = new double[5][HIT_FACTORS];
    static LinkedList waves;

    public void run() {
        setAdjustRadarForGunTurn(true);
        setAdjustGunForRobotTurn(true);
        waves = new LinkedList();

        do {
            turnRadarLeftRadians(1);
        } while(true);
    }

    public void onScannedRobot(ScannedRobotEvent e) {
        Wave w = new Wave();
        int temp = -1;
        double temp2;

        double distance = e.getDistance();

        if((temp2 = enemyEnergy - e.getEnergy()) > 0 && (w.speed = Rules.getBulletSpeed(temp2)) >= 11.0) {
            w.hits = hits[(int)(Math.abs(lastVelocity)) >> 1];
            w.orbitDirection = getVelocity() > 0 ? MIDDLE_FACTOR : -MIDDLE_FACTOR;
            waves.add(w);
            addCustomEvent(w);
        }
        lastVelocity = getVelocity();

        myLocation = new Point2D.Double(getX(), getY());
        w.source = projectMotion(temp2 = w.absoluteBearing = getHeadingRadians() + e.getBearingRadians(), distance);

        try {
            double maxRisk = Double.POSITIVE_INFINITY;
            Wave surfingWave = (Wave)(waves.getFirst());

            while(temp <= 1) {
                Point2D.Double projectedLocation;
                double smoothedHeading = temp2 + Math.PI/2 + (temp / 5.0);
                while(!new Rectangle2D.Double(30.0, 30.0, 740.0, 540.0).contains(projectedLocation = projectMotion(smoothedHeading -= (temp * 0.01), 170 * temp)));

                double risk = surfingWave.hits[surfingWave.hitBin(projectMotion(smoothedHeading, ((int)predictionDistances.charAt(17 * surfingWave.impactTime + (temp * (int)lastVelocity)) - 12) * temp))] / projectedLocation.distance(w.source);
                if(risk < maxRisk) {
                    maxRisk = risk;
                    setAhead(temp * 100);
                    setTurnRightRadians(Utils.normalRelativeAngle(smoothedHeading - getHeadingRadians()));
                }

                temp += 2;
            }
        } catch (Exception ex) {}

        setTurnRadarRightRadians(2 * Utils.normalRelativeAngle(temp2 - getRadarHeadingRadians()));

        enemyEnergy = e.getEnergy();

        //Gun Lifted Directly From WeekendObseesion
        int matchLen = 30;
        int i;

        enemyHistory = String.valueOf((char)(e.getVelocity() * Math.sin(e.getHeadingRadians() - temp2))).concat(enemyHistory);

        while(
                (temp =
                        enemyHistory.indexOf(
                                enemyHistory.substring(0, matchLen--),
                                i = 2 + (int)(distance / 14)))
                        < 0);

        do {
            temp2 += (short)enemyHistory.charAt(--temp) / distance;
        } while (--i > 0);

        //if((int)(getEnergy()) > 2) {
        setFire(2 + (int)(100 / distance));
        //}

        setTurnGunRightRadians(Utils.normalRelativeAngle(0.001 + temp2 - getGunHeadingRadians()));
    }

    public void onHitByBullet(HitByBulletEvent e) {
        try {
            Wave w = (Wave)(waves.getFirst());

            int i = 0;
            do {
                w.hits[i] += 1.0 / (1 + Math.abs(i++ - w.hitBin(myLocation)));
            } while(true);
        } catch (Exception ex) {}
    }

    static Point2D.Double projectMotion(double heading, double distance) {
        return new Point2D.Double(myLocation.x + (distance * Math.sin(heading)), myLocation.y + (distance * Math.cos(heading)));
    }

    public class Wave extends Condition {
        int ticksInAir;
        int impactTime;
        int orbitDirection;
        double speed;
        double absoluteBearing;
        Point2D.Double source;

        double[] hits;

        public boolean test() {
            if((impactTime = (int)(source.distance(myLocation) / speed) - (++ticksInAir)) < 0) {
                waves.remove(this);
            }

            return false;
        }

        public int hitBin (Point2D.Double location) {
            return MIDDLE_FACTOR + (int)Math.round(orbitDirection * Utils.normalRelativeAngle(absoluteBearing - Math.atan2(source.getX() - location.getX(), source.getY() - location.getY())) / Math.asin(8 / speed));
        }
    }

    static String predictionDistances = "" + (char)(1+12) +     (char)(1+12) +     (char)(1+12) +     (char)(1+12) +     (char)(1+12) +     (char)(1+12) +     (char)(1+12) +     (char)(1+12) +     (char)(1+12) +
            (char)(1+12) +     (char)(1+12) +     (char)(1+12) +     (char)(1+12) +     (char)(1+12) +     (char)(1+12) +     (char)(1+12) +     (char)(1+12) +     (char)(1+12) +     (char)(1+12) +     (char)(1+12) +     (char)(1+12) +     (char)(1+12) +     (char)(1+12) +     (char)(1+12) +     (char)(1+12) +     (char)(1+12) +
            (char)(-6+12) +    (char)(-5+12) +    (char)(-4+12) +    (char)(-3+12) +    (char)(-2+12) +    (char)(-1+12) +    (char)(1+12) +     (char)(1+12) +     (char)(1+12) +     (char)(2+12) +     (char)(3+12) +     (char)(4+12) +     (char)(5+12) +     (char)(6+12) +     (char)(7+12) +     (char)(8+12) +     (char)(8+12) +
            (char)(-6-4+12) +  (char)(-5-3+12) +  (char)(-4-2+12) +  (char)(-3-1+12) +  (char)(-2+0+12) +  (char)(-1+0+12) +  (char)(0+1+12) +   (char)(0+1+12) +   (char)(1+2+12) +   (char)(2+3+12) +   (char)(3+4+12) +   (char)(4+5+12) +   (char)(5+6+12) +   (char)(6+7+12) +   (char)(7+8+12) +   (char)(8+8+12) +   (char)(8+8+12) +
            (char)(-10-2+12) + (char)(-8-1+12) +  (char)(-6+0+12) +  (char)(-4+0+12) +  (char)(-2+1+12) +  (char)(-1+1+12) +  (char)(1+2+12) +   (char)(1+2+12) +   (char)(3+3+12) +   (char)(5+4+12) +   (char)(7+5+12) +   (char)(9+6+12) +   (char)(11+7+12) +  (char)(13+8+12) +  (char)(15+8+12) +  (char)(16+8+12) +  (char)(16+8+12) +
            (char)(-12+0+12) + (char)(-9+0+12) +  (char)(-6+1+12) +  (char)(-4+1+12) +  (char)(-1+2+12) +  (char)(0+2+12) +   (char)(3+3+12) +   (char)(3+3+12) +   (char)(6+4+12) +   (char)(9+5+12) +   (char)(12+6+12) +  (char)(15+7+12) +  (char)(18+8+12) +  (char)(21+8+12) +  (char)(23+8+12) +  (char)(24+8+12) +  (char)(24+8+12) +
            (char)(-12+1+12) + (char)(-9+1+12) +  (char)(-5+2+12) +  (char)(-3+2+12) +  (char)(1+3+12) +   (char)(2+3+12) +   (char)(6+4+12) +   (char)(6+4+12) +   (char)(10+5+12) +  (char)(14+6+12) +  (char)(18+7+12) +  (char)(22+8+12) +  (char)(26+8+12) +  (char)(29+8+12) +  (char)(31+8+12) +  (char)(32+8+12) +  (char)(32+8+12) +
            (char)(-11+2+12) + (char)(-8+2+12) +  (char)(-3+3+12) +  (char)(-1+3+12) +  (char)(4+4+12) +   (char)(5+4+12) +   (char)(10+5+12) +  (char)(10+5+12) +  (char)(15+6+12) +  (char)(20+7+12) +  (char)(25+8+12) +  (char)(30+8+12) +  (char)(34+8+12) +  (char)(37+8+12) +  (char)(39+8+12) +  (char)(40+8+12) +  (char)(40+8+12) +
            (char)(-9+3+12) +  (char)(-6+3+12) +  (char)(0+4+12) +   (char)(2+4+12) +   (char)(8+5+12) +   (char)(9+5+12) +   (char)(15+6+12) +  (char)(15+6+12) +  (char)(21+7+12) +  (char)(27+8+12) +  (char)(33+8+12) +  (char)(38+8+12) +  (char)(42+8+12) +  (char)(45+8+12) +  (char)(47+8+12) +  (char)(48+8+12) +  (char)(48+8+12) +
            (char)(-6+4+12) +  (char)(-3+4+12) +  (char)(4+5+12) +   (char)(6+5+12) +   (char)(13+6+12) +  (char)(14+6+12) +  (char)(21+7+12) +  (char)(21+7+12) +  (char)(28+8+12) +  (char)(35+8+12) +  (char)(41+8+12) +  (char)(46+8+12) +  (char)(50+8+12) +  (char)(53+8+12) +  (char)(55+8+12) +  (char)(56+8+12) +  (char)(56+8+12) +
            (char)(-2+5+12) +  (char)(1+5+12) +   (char)(9+6+12) +   (char)(11+6+12) +  (char)(19+7+12) +  (char)(20+7+12) +  (char)(28+8+12) +  (char)(28+8+12) +  (char)(36+8+12) +  (char)(43+8+12) +  (char)(49+8+12) +  (char)(54+8+12) +  (char)(58+8+12) +  (char)(61+8+12) +  (char)(63+8+12) +  (char)(64+8+12) +  (char)(64+8+12) +
            (char)(3+6+12) +   (char)(6+6+12) +   (char)(15+7+12) +  (char)(17+7+12) +  (char)(26+8+12) +  (char)(27+8+12) +  (char)(36+8+12) +  (char)(36+8+12) +  (char)(44+8+12) +  (char)(51+8+12) +  (char)(57+8+12) +  (char)(62+8+12) +  (char)(66+8+12) +  (char)(69+8+12) +  (char)(71+8+12) +  (char)(72+8+12) +  (char)(72+8+12) +
            (char)(9+7+12) +   (char)(12+7+12) +  (char)(22+8+12) +  (char)(24+8+12) +  (char)(34+8+12) +  (char)(35+8+12) +  (char)(44+8+12) +  (char)(44+8+12) +  (char)(52+8+12) +  (char)(59+8+12) +  (char)(65+8+12) +  (char)(70+8+12) +  (char)(74+8+12) +  (char)(77+8+12) +  (char)(79+8+12) +  (char)(80+8+12) +  (char)(80+8+12) +
            (char)(16+8+12) +  (char)(19+8+12) +  (char)(30+8+12) +  (char)(32+8+12) +  (char)(42+8+12) +  (char)(43+8+12) +  (char)(52+8+12) +  (char)(52+8+12) +  (char)(60+8+12) +  (char)(67+8+12) +  (char)(73+8+12) +  (char)(78+8+12) +  (char)(82+8+12) +  (char)(85+8+12) +  (char)(87+8+12) +  (char)(88+8+12) +  (char)(88+8+12) +
            (char)(24+8+12) +  (char)(27+8+12) +  (char)(38+8+12) +  (char)(40+8+12) +  (char)(50+8+12) +  (char)(51+8+12) +  (char)(60+8+12) +  (char)(60+8+12) +  (char)(68+8+12) +  (char)(75+8+12) +  (char)(81+8+12) +  (char)(86+8+12) +  (char)(90+8+12) +  (char)(93+8+12) +  (char)(95+8+12) +  (char)(96+8+12) +  (char)(96+8+12) +
            (char)(32+8+12) +  (char)(35+8+12) +  (char)(46+8+12) +  (char)(48+8+12) +  (char)(58+8+12) +  (char)(59+8+12) +  (char)(68+8+12) +  (char)(68+8+12) +  (char)(76+8+12) +  (char)(83+8+12) +  (char)(89+8+12) +  (char)(94+8+12) +  (char)(98+8+12) +  (char)(101+8+12) + (char)(103+8+12) + (char)(104+8+12) + (char)(104+8+12) +
            (char)(40+8+12) +  (char)(43+8+12) +  (char)(54+8+12) +  (char)(56+8+12) +  (char)(66+8+12) +  (char)(67+8+12) +  (char)(76+8+12) +  (char)(76+8+12) +  (char)(84+8+12) +  (char)(91+8+12) +  (char)(97+8+12) +  (char)(102+8+12) + (char)(106+8+12) + (char)(109+8+12) + (char)(111+8+12) + (char)(112+8+12) + (char)(112+8+12) +
            (char)(48+8+12) +  (char)(51+8+12) +  (char)(62+8+12) +  (char)(64+8+12) +  (char)(74+8+12) +  (char)(75+8+12) +  (char)(84+8+12) +  (char)(84+8+12) +  (char)(92+8+12) +  (char)(99+8+12) +  (char)(105+8+12) + (char)(110+8+12) + (char)(114+8+12) + (char)(117+8+12) + (char)(119+8+12) + (char)(120+8+12) + (char)(120+8+12) +
            (char)(56+8+12) +  (char)(59+8+12) +  (char)(70+8+12) +  (char)(72+8+12) +  (char)(82+8+12) +  (char)(83+8+12) +  (char)(92+8+12) +  (char)(92+8+12) +  (char)(100+8+12) + (char)(107+8+12) + (char)(113+8+12) + (char)(118+8+12) + (char)(122+8+12) + (char)(125+8+12) + (char)(127+8+12) + (char)(128+8+12) + (char)(128+8+12) +
            (char)(64+8+12) +  (char)(67+8+12) +  (char)(78+8+12) +  (char)(80+8+12) +  (char)(90+8+12) +  (char)(91+8+12) +  (char)(100+8+12) + (char)(100+8+12) + (char)(108+8+12) + (char)(115+8+12) + (char)(121+8+12) + (char)(126+8+12) + (char)(130+8+12) + (char)(133+8+12) + (char)(135+8+12) + (char)(136+8+12) + (char)(136+8+12) +
            (char)(72+8+12) +  (char)(75+8+12) +  (char)(86+8+12) +  (char)(88+8+12) +  (char)(98+8+12) +  (char)(99+8+12) +  (char)(108+8+12) + (char)(108+8+12) + (char)(116+8+12) + (char)(123+8+12) + (char)(129+8+12) + (char)(134+8+12) + (char)(138+8+12) + (char)(141+8+12) + (char)(143+8+12) + (char)(144+8+12) + (char)(144+8+12) +
            (char)(80+8+12) +  (char)(83+8+12) +  (char)(94+8+12) +  (char)(96+8+12) +  (char)(106+8+12) + (char)(107+8+12) + (char)(116+8+12) + (char)(116+8+12) + (char)(124+8+12) + (char)(131+8+12) + (char)(137+8+12) + (char)(142+8+12) + (char)(146+8+12) + (char)(149+8+12) + (char)(151+8+12) + (char)(152+8+12) + (char)(152+8+12) +
            (char)(88+8+12) +  (char)(91+8+12) +  (char)(102+8+12) + (char)(104+8+12) + (char)(114+8+12) + (char)(115+8+12) + (char)(124+8+12) + (char)(124+8+12) + (char)(132+8+12) + (char)(139+8+12) + (char)(145+8+12) + (char)(150+8+12) + (char)(154+8+12) + (char)(157+8+12) + (char)(159+8+12) + (char)(160+8+12) + (char)(160+8+12) +
            (char)(96+8+12) +  (char)(99+8+12) +  (char)(110+8+12) + (char)(112+8+12) + (char)(122+8+12) + (char)(123+8+12) + (char)(132+8+12) + (char)(132+8+12) + (char)(140+8+12) + (char)(147+8+12) + (char)(153+8+12) + (char)(158+8+12) + (char)(162+8+12) + (char)(165+8+12) + (char)(167+8+12) + (char)(168+8+12) + (char)(168+8+12) +
            (char)(104+8+12) + (char)(107+8+12) + (char)(118+8+12) + (char)(120+8+12) + (char)(130+8+12) + (char)(131+8+12) + (char)(140+8+12) + (char)(140+8+12) + (char)(148+8+12) + (char)(155+8+12) + (char)(161+8+12) + (char)(166+8+12) + (char)(170+8+12) + (char)(173+8+12) + (char)(175+8+12) + (char)(176+8+12) + (char)(176+8+12) +
            (char)(112+8+12) + (char)(115+8+12) + (char)(126+8+12) + (char)(128+8+12) + (char)(138+8+12) + (char)(139+8+12) + (char)(148+8+12) + (char)(148+8+12) + (char)(156+8+12) + (char)(163+8+12) + (char)(169+8+12) + (char)(174+8+12) + (char)(178+8+12) + (char)(181+8+12) + (char)(183+8+12) + (char)(184+8+12) + (char)(184+8+12);
    static String enemyHistory = ""
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 1
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 2
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char)-1
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char) 0 + (char) 0 + (char) 0
            + (char) 0 + (char)-2 + (char)-4 + (char)-6
            + (char)-8 + (char)-8 + (char)-8 + (char)-8
            + (char)-8 + (char)-8 + (char)-8 + (char)-8
            + (char)-8 + (char)-8 + (char)-8 + (char)-8
            + (char)-8 + (char)-8 + (char)-8 + (char)-8
            + (char)-8 + (char)-8 + (char)-8 + (char)-8
            + (char)-8 + (char)-8 + (char)-8 + (char)-8
            + (char)-8 + (char)-8 + (char)-8 + (char)-8
            + (char)-8 + (char)-8 + (char)-8 + (char)-8
            + (char)-8 + (char)-8 + (char)-8 + (char)-8
            + (char)-8 + (char)-8 + (char)-8 + (char)-8
            + (char)-7 + (char)-6 + (char)-5 + (char)-4
            + (char)-3 + (char)-2 + (char)-1 + (char)0
            + (char) 2 + (char) 4 + (char) 6 + (char) 8
            + (char) 8 + (char) 8 + (char) 8 + (char) 8
            + (char) 8 + (char) 8 + (char) 8 + (char) 8
            + (char) 8 + (char) 8 + (char) 8 + (char) 8
            + (char) 8 + (char) 8 + (char) 8 + (char) 8
            + (char) 8 + (char) 8 + (char) 8 + (char) 8
            + (char) 8 + (char) 8 + (char) 8 + (char) 8
            + (char) 8 + (char) 8 + (char) 8 + (char) 8
            + (char) 8 + (char) 8 + (char) 8 + (char) 8
            + (char) 8 + (char) 8 + (char) 8 + (char) 8
            + (char) 8 + (char) 8 + (char) 8 + (char) 8
            + (char) 7 + (char) 6 + (char) 5 + (char) 4
            + (char) 3 + (char) 2 + (char) 1 + (char) 0;
}
