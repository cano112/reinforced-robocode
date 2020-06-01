package iwium;
import robocode.*;
import java.util.Random;
//import java.awt.Color;

// API help : https://robocode.sourceforge.io/docs/robocode/robocode/Robot.html

/**
 * RandomRobot - a robot by (your name here)
 */
public class SimpleRobot extends Robot {
	private volatile boolean run = true;
	private volatile int fired = 0;
	private volatile int missed = 0;
	Random r = new Random();
	/**
	 * run: RandomRobot's default behavior
	 */
	public void run() {
		// Initialization of the robot should be put here

		// After trying out your robot, try uncommenting the import at the top,
		// and the next line:

		// setColors(Color.red,Color.blue,Color.green); // body,gun,radar

		// Robot main loop
		while(true) {
			// Replace the next 4 lines with any behavior you would like
			if(run){
				ahead(100);
				turnGunRight(360);
			}
		}
	}

	/**
	 * onScannedRobot: What to do when you see another robot
	 */
	public void onScannedRobot(ScannedRobotEvent e) {
		stop();
		run = false;
		//turnRight(e.getBearing());
		//turnGunRight(e.getBearing() - e.getHeading());
		while(!run){
        	fire(10);
			fired = fired + 1;
			if(r.nextBoolean()){
				ahead(50);
			}
			else{
				ahead(-50);
			}
		}
	}
	
	public void onBulletHit(BulletHitEvent event){
		
	}
	
    public void onBulletMissed(BulletMissedEvent event){
		run = true;
		missed = missed + 1;
		if(fired > 10 && missed / fired > 0.2){
			run = true;
			fired = missed = 0;
		}
	}

	/**
	 * onHitByBullet: What to do when you're hit by a bullet
	 */
	public void onHitByBullet(HitByBulletEvent e) {
		// Replace the next line with any behavior you would like
		turnRight(e.getBearing() + 90);
		ahead(100);
		run = true;
	}
	
	/**
	 * onHitWall: What to do when you hit a wall
	 */
	public void onHitWall(HitWallEvent e) {
		// Replace the next line with any behavior you would like
		if(run){
		turnRight(90);
		ahead(100);
		}
		else{
			if(r.nextBoolean()){
				ahead(50);
			}
			else{
				ahead(-50);
			}
		}
	}	
}
