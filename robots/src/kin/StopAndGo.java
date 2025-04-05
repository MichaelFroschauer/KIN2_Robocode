package kin;

import robocode.AdvancedRobot;
import robocode.ScannedRobotEvent;
import robocode.HitWallEvent;
import robocode.util.Utils;

/*
 * "StopAndGo" Movement Strategy with Adaptive Fire Detection
 * <p>
 * This strategy is designed to confuse enemy targeting systems by causing the robot to intermittently
 * stop (ideally when the enemy fires) and then resume lateral movement in an unpredictable manner
 * <p>
 * General Concept:
 * 1) for each detected enemy, compute the absolute bearing from your robot’s current position
 * 2) calculate a lateral movement angle by adding a 90° offset (multiplied by a directional factor)
 *    to the enemy's bearing
 * 3) adjust the calculated movement angle if the projected move (100 units ahead) would place the robot
 *    too close to the battlefield boundaries; if so -> steer toward the center of the battlefield
 * 4) Monitor the enemy’s energy by comparing the current energy level with the previous tick
 *    - when a shot is detected, toggle a flag so that the robot alternates between stopping and moving
 * 5) Execute stop-and-go behavior based on the flag:
 *    - if the flag (stopNextTick) is true, the robot remains stopped
 *    - otherwise, the robot moves forward
 * 6) alternate the lateral movement direction after each scan to add unpredictability
 * 7) upon hitting a wall, immediately reverse direction and adjust movement to avoid getting stuck
 */
public class StopAndGo extends AdvancedRobot {

    // toggles between 1 and -1 to vary the lateral movement direction.
    private double movementDirection = 1;
    // minimum distance from the battlefield walls to avoid getting stuck in a corner.
    private final double SAFE_MARGIN = 50.0;

    // state variables for detecting enemy fire:
    // lastenemyenergy stores the last observed enemy energy.
    private double lastEnemyEnergy = 100.0; // typical start value
    // stopnexttick: if true, the bot will remain stationary in the next tick.
    private boolean stopNextTick = false;

    @Override
    public void run() {
        // turn the radar independently of the robot's body
        setAdjustRadarForGunTurn(true);

        // infinite loop: continuous scanning
        while (true) {
            setTurnRadarRight(360); // full radar sweep
            execute();
        }
    }

    @Override
    public void onScannedRobot(ScannedRobotEvent event) {
        double enemyBearing = event.getBearingRadians();
        double absoluteBearing = getHeadingRadians() + enemyBearing;

        // compute the desired angle for lateral movement relative to the enemy.
        double desiredAngle = absoluteBearing + (Math.PI / 2 * movementDirection);
        // adjust the angle if the future point (100 units ahead) is too close to the battlefield boundaries.
        desiredAngle = adjustAngleForWalls(desiredAngle);
        setTurnRightRadians(Utils.normalRelativeAngle(desiredAngle - getHeadingRadians()));

        // detect enemy fire by checking for an energy drop.
        double currentEnemyEnergy = event.getEnergy();
        double energyDrop = lastEnemyEnergy - currentEnemyEnergy;
        // if the energy drop is between 0.1 and 3.0, assume that the enemy fired.
        if (energyDrop > 0.1 && energyDrop <= 3.0) {
            // toggle: when the enemy fires, alternate between stopping and moving.
            stopNextTick = !stopNextTick;
        }
        lastEnemyEnergy = currentEnemyEnergy;

        // stop-and-go behavior: if stopnexttick is true, the bot stays still; otherwise, it moves.
        if (stopNextTick) {
            setAhead(0);
        } else {
            setAhead(100);
        }

        // alternate the lateral movement direction for unpredictability.
        movementDirection = -movementDirection;
    }

    /**
     * checks if the future point (100 units ahead) is too close to the battlefield boundaries.
     * if so, adjusts the movement angle so that the robot steers towards the battlefield center.
     *
     * @param desiredAngle the originally calculated movement angle.
     * @return an adjusted, safe movement angle.
     */
    private double adjustAngleForWalls(double desiredAngle) {
        double moveDistance = 100; // assumed movement distance
        double futureX = getX() + Math.sin(desiredAngle) * moveDistance;
        double futureY = getY() + Math.cos(desiredAngle) * moveDistance;
        double battlefieldWidth = getBattleFieldWidth();
        double battlefieldHeight = getBattleFieldHeight();

        // if the future point is too close to the boundaries, steer towards the battlefield center.
        if (futureX < SAFE_MARGIN || futureX > battlefieldWidth - SAFE_MARGIN ||
                futureY < SAFE_MARGIN || futureY > battlefieldHeight - SAFE_MARGIN) {
            double centerX = battlefieldWidth / 2.0;
            double centerY = battlefieldHeight / 2.0;
            return Math.atan2(centerX - getX(), centerY - getY());
        }
        return desiredAngle;
    }

    @Override
    public void onHitWall(HitWallEvent event) {
        // upon hitting a wall, reverse the movement direction to avoid sticking.
        movementDirection = -movementDirection;
        setTurnRight(90);
        setAhead(100);
    }
}
