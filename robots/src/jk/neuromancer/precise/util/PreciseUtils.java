package jk.neuromancer.precise.util;
import java.awt.geom.*;
import java.util.*;
import jk.neuromancer.math.FastTrig;
public class PreciseUtils{
   public static final int INTERSECTION = 3, PASSED = 1, NOT_REACHED = 2;


//high speed test to determine if the full method should be run this tick
   public static int intersects(Point2D.Double botLocation, PreciseWave wave){
      double[] distSq = new double[]{
            wave.fireLocation.distanceSq(botLocation.x - 18, botLocation.y + 18),
            wave.fireLocation.distanceSq(botLocation.x + 18, botLocation.y + 18),
            wave.fireLocation.distanceSq(botLocation.x + 18, botLocation.y - 18),
            wave.fireLocation.distanceSq(botLocation.x - 18, botLocation.y - 18)};


      double compDistSq = sqr(wave.distanceTraveled);
      if ( compDistSq < distSq[0] &  compDistSq < distSq[1] &  compDistSq < distSq[2] &  compDistSq < distSq[3])
         return NOT_REACHED;

      compDistSq = sqr(wave.distanceTraveled - wave.bulletVelocity);
      if( compDistSq > distSq[0] & compDistSq > distSq[1] & compDistSq > distSq[2] & compDistSq > distSq[3])
         return PASSED;

      return INTERSECTION;
   }
   public static double[] getIntersectionRange(Point2D.Double botLocation, PreciseWave wave){
      double[] yBounds = new double[]{botLocation.y - 18, botLocation.y + 18};
      double[] xBounds = new double[]{botLocation.x - 18, botLocation.x + 18};

      double[] radii = new double[]{wave.distanceTraveled, wave.distanceTraveled - wave.bulletVelocity};

      ArrayList<Point2D.Double> intersects = new ArrayList<Point2D.Double>();
      for(int i = 0; i < 2; i++)
         for(int j = 0; j < 2; j++){
            Point2D.Double[] testPoints = vertIntersect(wave.fireLocation.x, wave.fireLocation.y, radii[i], xBounds[j]);
            for(int k = 0; k < testPoints.length; k++)
               if(inBounds(testPoints[k].y, yBounds))
                  intersects.add(testPoints[k]);
         }

      for(int i = 0; i < 2; i++)
         for(int j = 0; j < 2; j++){
            Point2D.Double[] testPoints = horizIntersect(wave.fireLocation.x, wave.fireLocation.y, radii[i], yBounds[j]);
            for(int k = 0; k < testPoints.length; k++)
               if(inBounds(testPoints[k].x, xBounds))
                  intersects.add(testPoints[k]);
         }
      for(int i = 0; i < 2; i++)
         for(int j = 0; j < 2; j++){
            Point2D.Double testCorner = new Point2D.Double(xBounds[i], yBounds[j]);
            double distSq = testCorner.distanceSq(wave.fireLocation);
            if(distSq <= sqr(radii[0]) && distSq > sqr(radii[1]))
               intersects.add(testCorner);
         }
      double antiClockAngle = 1;
      double clockAngle = -1;
      double absBearing = angle(wave.fireLocation, botLocation);

      for(int i = 0, k = intersects.size(); i<k; i++){
         Point2D.Double p = intersects.get(i);
         double angDiff = fastRelativeAngle(angle(wave.fireLocation,p) - absBearing);
         if(angDiff > clockAngle){
            clockAngle = angDiff;
         }
         if(angDiff < antiClockAngle){
            antiClockAngle = angDiff;
         }
      }
      return new double[]{fastAbsoluteAngle(antiClockAngle + absBearing), fastAbsoluteAngle(clockAngle + absBearing)};
   }
   public static Point2D.Double intersection(Point2D.Double l1, Point2D.Double l2,  PreciseWave w){
      double xd = l2.x - l1.x;
      double yd = l2.y - l1.y;
      double a = sqr(xd) + sqr(yd);
      double b = 2*(xd*(l1.x - w.fireLocation.x) + yd*(l1.y - w.fireLocation.y));
      double c = sqr(l1.x - w.fireLocation.x) + sqr(l1.y - w.fireLocation.y) - sqr(w.distanceTraveled);
      double det = b*b - 4*a*c;
      if(det < 0)
         throw new IllegalArgumentException();
      det = Math.sqrt(det);
      double t = (-b + det)/(2*a);
      if(0 <= t && t <= 1)
         return new Point2D.Double(l1.x + xd*t, l1.y + yd*t);
      t = (-b - det)/(2*a);
      if(0 <= t && t <= 1)
         return new Point2D.Double(l1.x + xd*t, l1.y + yd*t);

      throw new IllegalArgumentException("t is out of range [0;1]: " + t);
   }





   static boolean inBounds(double q ,double[] bounds){
      return  bounds[0] <= q && q <= bounds[1] ;
   }

    //assumes between -PI*2 and PI*2
   public static double fastRelativeAngle(double angle) {
      return FastTrig.normalRelativeAngle(angle);
   }

    //assumes between -PI*2 and PI*4
   public static double fastAbsoluteAngle(double angle){
      return FastTrig.normalAbsoluteAngle(angle);
   }

   static Point2D.Double[] vertIntersect(double centerX, double centerY, double r, double intersectX){
      double deltaX = centerX - intersectX;
      double sqrtVal = r*r - deltaX*deltaX;
      if(sqrtVal < 0)
         return new Point2D.Double[]{};

      // if(sqrtVal == 0)
         // return new Point2D.Double[]{
               // new Point2D.Double(intersectX, centerY)};

      sqrtVal = Math.sqrt(sqrtVal);
      return new Point2D.Double[]{
            new Point2D.Double(intersectX, centerY + sqrtVal),
            new Point2D.Double(intersectX, centerY - sqrtVal)};
   }
   static Point2D.Double[] horizIntersect(double centerX, double centerY, double r, double intersectY){
      double deltaY = centerY - intersectY;
      double sqrtVal = r*r - deltaY*deltaY;
      if(sqrtVal < 0)
         return new Point2D.Double[]{};

      // if(sqrtVal == 0)
         // return new Point2D.Double[]{
               // new Point2D.Double(centerX, intersectY)};

      sqrtVal = Math.sqrt(sqrtVal);
      return new Point2D.Double[]{
            new Point2D.Double(centerX + sqrtVal, intersectY),
            new Point2D.Double(centerX - sqrtVal, intersectY)};
   }

   public static final double sqr(double d){
      return d*d;}

   public static double angle(Point2D.Double source, Point2D.Double target) {
      return FastTrig.atan2(target.x - source.x, target.y - source.y);
   }

}
