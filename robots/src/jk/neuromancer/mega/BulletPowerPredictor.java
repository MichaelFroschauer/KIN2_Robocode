package jk.neuromancer.mega;
import java.util.*;
import jk.neuromancer.tree.KDTree;

public class BulletPowerPredictor{
   KDTree<Double> bulletPowerTree = new KDTree.Euclidean<Double>(3);
   public void train(double botEnergy, double oppEnergy, double distance, double bulletPower){
      bulletPowerTree.addPoint(
            new double[]{botEnergy/200,oppEnergy/200,distance/1200},
            bulletPower); 
   }

   public double predictBulletPower(double myEnergy, double enemyEnergy, double distance){

      List<KDTree.SearchResult<Double>> cl = bulletPowerTree.nearestNeighbours(
            new double[]{myEnergy/200, enemyEnergy/200, distance/1200},
            Math.min((int)Math.ceil(Math.sqrt(bulletPowerTree.size())),20));

      double[][] bullHist = new double[2][cl.size()];

      Iterator<KDTree.SearchResult<Double>> it = cl.iterator();
      int i = 0;
      while(it.hasNext()){
         KDTree.SearchResult<Double> p = it.next();
         double weight = 1/(p.distance + 0.1);
         bullHist[0][i] = weight;
         bullHist[1][i] = p.payload;
         i++;
      }
      double maxScore = 0;
      double maxPower = 2;
      for(int j = 0; j < i; j++){
         double score = 0;
         for(int k = 0; k < j; k++)
            score += bullHist[0][k]/(sqr(bullHist[1][j] - bullHist[1][k]) + 0.1);
         for(int k = j+1; k < i; k++)
            score += bullHist[0][k]/(sqr(bullHist[1][j] - bullHist[1][k]) + 0.1);
         score *= bullHist[0][j];

         if(score > maxScore){
            maxScore = score;
            maxPower = bullHist[1][j];
         }
      }

      return maxPower;
   }
   double sqr(double d){
      return d*d;}

}