import robocode.AdvancedRobot;

public class ShakerBot extends AdvancedRobot {

    public void run() {

        while(true) {
            ahead(1);
            ahead(-1);
        }
    }
}
