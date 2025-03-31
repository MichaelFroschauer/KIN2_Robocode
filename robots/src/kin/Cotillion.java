package kin;

import robocode.*;
import robocode.util.*;
import java.awt.geom.*;
import java.util.*;
import java.awt.*;

/*
*Cotillion - multiple-choice pattern-matching by Skilgannon (Julian Kent)
*Special features:
** never finds the same 'match' twice, even with different key-lengths
** weights the multiple choice depending on the match length
** finds up to N different matches
** matches on lateral velocity and advancing velocity
** matches on forward-wall
** removes out-of-bounds rebuilds
** strong StopAndGo/Random multi-movement
*/


public class Cotillion extends AdvancedRobot
{


   static final int BINS = 255;// full 360 degrees of bins - not just in the reachable range
   //static final char EOR = 65535;
   private static StringBuilder searchData = new StringBuilder();

   private static double bulletVelocity;
   private static double direction = 1;
   private static double lastEnemyEnergy;
   private static double flat;
   private static double hits;

   // public void run() {
   
   //NOT NECESSARY FOR 35 ROUND BATTLES - CAN BE REMOVED FOR 15 EXTRA CODESIZE
   // try{
   // searchData.delete(60000, 80000);
   // }
   // catch(Exception e){}
      //searchData.insert(0,EOR);
           // }
   public void onStatus(StatusEvent e){
      setAdjustRadarForGunTurn(true);
      setAdjustGunForRobotTurn(true);
   
      setTurnRadarRightRadians(1);
   }

   public void onScannedRobot(ScannedRobotEvent e) {
   
      //StringBuilder data = searchData;
      
      int tempIndex;
      double eOffset;
      double absbearing;
   
      double eDistance ;
   
   
   //movement \/  \/
      double goAngle;
      //double rammer; 
      eOffset = 2 + (tempIndex = (int)(100/(eDistance = e.getDistance())));//( Math.PI/2 + 1*0.8) - (eDistance = e.getDistance())*(0.8/600);
      Rectangle2D.Double field;
      while(! (field = new Rectangle2D.Double(18-getX(),18-getY(),764,564)).
      contains(160 * Math.sin(goAngle = (absbearing=e.getBearingRadians()+ getHeadingRadians())
      + direction * (eOffset -= .02)),160 * Math.cos(goAngle))
      );
   
   
      if((
      Math.random() + tempIndex  <  (-0.6*Math.sqrt(bulletVelocity/eDistance) + 0.04)*flat
      ) || 
      eOffset < Math.PI/4 ) {
         direction = -direction;
      }
      setTurnRightRadians(Math.tan(goAngle -= getHeadingRadians()));
   
      eOffset = (lastEnemyEnergy - (lastEnemyEnergy = e.getEnergy()));
   
      if(flat  < eOffset + tempIndex)
         setAhead(((3 + (int)(eOffset*1.999999)) << 3 )* Math.signum(Math.cos(goAngle)));
   
   // MOVEMENT /\   /\   /\  
   
   
   // GUN \/    \/   \/
   
   
      searchData.insert(0,(char)(
         ((4+(Math.round((float)(e.getVelocity()*Math.cos(eOffset = e.getHeadingRadians()- absbearing))/2f))))
         |
         ((int)Math.round(eOffset = e.getVelocity()*Math.sin(eOffset))<<11)
         |
         0x10*(int)Math.signum(field.outcode(eDistance*Math.sin(eOffset = absbearing+0.75*Math.signum(eOffset)),
         eDistance*Math.cos(eOffset)))
         //|
         //((int)(eDistance/300)<<6)
         ));
   
   
      double bulletPower = tempIndex+Math.min(2,lastEnemyEnergy/4);
      int[] index ;
      int[] bins ; 
   
   
      int keyLength = Math.min(searchData.length(),30);
   
      index = new int[50];
      tempIndex = 0;
      bins = new int[BINS]; 
      search:
      try{
         do{
            do{
               if( tempIndex < 0 )
                  keyLength--;
            
               if(keyLength <= index[0] )
                  break search;
            
               tempIndex = searchData.indexOf(
                  searchData.substring(0, keyLength),tempIndex + 2);
            
            }while(tempIndex * Arrays.binarySearch(index,tempIndex) >= 0);
         
            int iterateIndex = index[0] = tempIndex;
            Arrays.sort(index);
            double db = eOffset = 0;
            goAngle = eDistance;
         
            rebuild:
            try{
               do
               {
                  char comboChar = searchData.charAt(iterateIndex--);
               
                  eOffset += ((short)comboChar>>11)/
                     (goAngle += 2*((int)(comboChar&0X0F) - 4));    
               
                  if(
                  !field.contains(
                  goAngle*Math.sin(absbearing+eOffset),
                  goAngle*Math.cos(absbearing+eOffset)) 
                  //|| 
                  //comboChar == EOR
                  )
                     break rebuild;
               
               }while ((db+=Rules.getBulletSpeed(bulletPower)) < goAngle ); 
            
               bins[(int)(Utils.normalAbsoluteAngle(eOffset)*((BINS - 1)/(2*Math.PI)))] += keyLength;
            //targetPoints.add(project(new Point2D.Double(getX(),getY()),eOffset+absbearing,goAngle));
            }
            catch(Exception ex){}
         
         
         }while(true);
      }
      catch(Exception ex){}
   
   
      keyLength = 0;
      tempIndex = 0;
      try{
         while(true){
            if(bins[++keyLength] > bins[tempIndex])
               tempIndex = keyLength;
         }
      
      }
      catch(Exception ex){}
   
   
      if(getEnergy() > bulletPower) // * bins[tempIndex] > 0 && getGunTurnRemainingRadians() == 0)
         setFire(bulletPower);
   
      setTurnGunRightRadians(Utils.normalRelativeAngle(absbearing 
         + 0.5*(2*Math.PI/(BINS - 1))// + 0.005
         + getVelocity()*Math.sin(e.getBearingRadians())/eDistance
         + (tempIndex)*(2*Math.PI/(BINS - 1)) - getGunHeadingRadians()));
   
   
      setTurnRadarRightRadians(Utils.normalRelativeAngle(absbearing - getRadarHeadingRadians())*2);
   
   // gun /\  /\
   
   }

   public void onHitByBullet(HitByBulletEvent e){
      lastEnemyEnergy += 20 - (bulletVelocity = e.getVelocity());
      if(( hits += (5/bulletVelocity)) > 2 + getRoundNum())
         flat = -1;
   //System.out.println("margin: " + (2 + getRoundNum() - hits));
   }


   public void onBulletHit(BulletHitEvent e){
      lastEnemyEnergy -= 10;//Rules.getBulletDamage(e.getBullet().getPower());
   }
/*
  //debugging code
public static Point2D.Double project(Point2D.Double sourceLocation, double angle, double length) {
return new Point2D.Double(sourceLocation.x + Math.sin(angle) * length,
sourceLocation.y + Math.cos(angle) * length);
}
ArrayList targetPoints = new ArrayList(), pathPoints = new ArrayList();

public void onPaint(java.awt.Graphics2D g) {
g.setColor(Color.green);
Iterator it = pathPoints.iterator();
while(it.hasNext()){
Point2D.Double p = (Point2D.Double)it.next();
g.drawOval((int)p.x-1,(int)p.y-1,2,2);

}
pathPoints.clear();
g.setColor(Color.red);
it = targetPoints.iterator();
while(it.hasNext()){
Point2D.Double p = (Point2D.Double)it.next();
g.drawOval((int)p.x-1,(int)p.y-1,2,2);

}
targetPoints.clear();

}
 //  */
  
  
  
}