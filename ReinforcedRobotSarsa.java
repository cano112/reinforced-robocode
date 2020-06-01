package iwium;
import robocode.*;
import java.awt.Color;
import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.ObjectInput;
import java.io.ObjectInputStream;
import java.io.ObjectOutput;
import java.io.ObjectOutputStream;
import java.io.OutputStream;
import java.io.Serializable;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Objects;
import java.util.Random;
import java.util.Set;

// API help : https://robocode.sourceforge.io/docs/robocode/robocode/Robot.html

/**
 * ReinforcedRobot - a robot by Kamil Noster & Grzegorz Jasiński
 */
public class ReinforcedRobotSarsa extends AdvancedRobot {
	
	private static final String KNOWLEDGE_FILE = "q.ser";
	
	private static final int  MIN_DISTANCE = 0;
	
	private static final int MAX_DISTANCE = 400;
	
	private static final int MIN_ANGLE = 0;
	
	private static final int MAX_ANGLE = 360;
	
	private static final int MIN_ENERGY = 0;
	
	private static final int MAX_ENERGY = 100;
	
	private static final int DEFAULT_DISTANCE = 20;
	
	private static final RobotAction DEFAULT_ACTION = new GoAhead(DEFAULT_DISTANCE);
	
	private static final int BUCKETS = 10;

	private static Map<Observation, Map<RobotAction, Double>> Q = new HashMap<>();	
	
	private static boolean qInitialized = false;

	private final double alpha = 0.1;

	private final double discountFactor = 0.1;
	
	private final double experimentRate = 0.1;
	
	private final Set<EventWrapper> events = new HashSet<>();
	
	private final Set<ScannedRobotEvent> scannedRobotEvents = new HashSet<>();
	
	private final Random random = new Random();
	
	private final RobotRandomActionFactory randomActionFactory = new RobotRandomActionFactory();
	
	public void run() {
		if (!qInitialized) {
			deserializeKnowledge();
			qInitialized = true;
		}
		setAdjustGunForRobotTurn(true);
		setAdjustRadarForGunTurn(true);
		setTurnRadarRight(Double.POSITIVE_INFINITY);
		setColors(Color.red, Color.blue, Color.green); // body, gun, radar
		
		Observation observation = observation();
		while (true) {
			RobotAction action = pickAction(observation);
			double reward = step(action);
			Observation newObservation = observation();
			updateKnowledge(action, pickAction(observation), observation, newObservation, reward);				
			observation = newObservation;
		}
	}
	
	private void deserializeKnowledge() {
		File q = getDataFile(KNOWLEDGE_FILE);
		if (q.exists()) {
			try(
					InputStream file = new FileInputStream(q);
					InputStream buffer = new BufferedInputStream(file);
					ObjectInput input = new ObjectInputStream(buffer)) {

				Q = (Map<Observation, Map<RobotAction, Double>>) input.readObject();
			} catch (ClassNotFoundException | IOException e){
				Q = new HashMap<>();
			}
		}
	}
	
	private void serializeKnowledge() {
		try (
				OutputStream file = new RobocodeFileOutputStream(getDataFile(KNOWLEDGE_FILE));
				OutputStream buffer = new BufferedOutputStream(file);
				ObjectOutput output = new ObjectOutputStream(buffer)) {
			output.writeObject(Q);
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}
	
	private double step(RobotAction action) {
		action.invoke(this);
		double reward = 0;
		for (EventWrapper e : events) {
			reward += e.reward;
		}
		events.clear();
		out.println("Step completed. Action: " + action.getClass().getSimpleName() + ". Reward: " + reward);
		return reward;
	}
	
	private void updateKnowledge(RobotAction oldAction, RobotAction newAction, Observation oldObservation, Observation newObservation, 
			double reward) {
		Map<RobotAction, Double> oldObservationActionToReward = Q.computeIfAbsent(oldObservation, k -> new HashMap<>());
        Map<RobotAction, Double> newObservationActionToReward = Q.computeIfAbsent(newObservation, k -> new HashMap<>());
		Double newReward = oldObservationActionToReward.getOrDefault(oldAction, 0.0) + 
				+ alpha * (reward + discountFactor * newObservationActionToReward.getOrDefault(newAction, 0.0) - oldObservationActionToReward.getOrDefault(oldAction, 0.0));
		oldObservationActionToReward.put(oldAction, newReward);
	}

	private double maxReward(Observation observation) {
		Map<RobotAction, Double> rewards = Q.get(observation);
		double maxReward = 0;
		if(rewards != null) {
			for (Double reward : rewards.values()) {
				if (reward >= maxReward) {
					maxReward = reward;
				}
			}
		}
		return maxReward;
	}
	
	private RobotAction pickAction(Observation observation) {
		
		if (random.nextDouble() < experimentRate) {
			return randomActionFactory.randomAction();
		}
		
		final Map<RobotAction, Double> actionToReward = Q.get(observation);
		if (actionToReward != null) {
			RobotAction topAction = null;
			double topReward = 0;
			for (Map.Entry<RobotAction, Double> entry : actionToReward.entrySet()) {
				final RobotAction action = entry.getKey();
				final double reward = entry.getValue();
				if (reward >=topReward) {
					topReward = reward;
					topAction = action;
				}
			}
			if (topAction != null) {
				return topAction;
			}
		}
		return randomActionFactory.randomAction();
	}
	
	private Observation observation() {
		ScannedRobotEvent nearest = null;
		for (ScannedRobotEvent scannedRobotEvent : scannedRobotEvents) {
			if (nearest == null || scannedRobotEvent.getDistance() < nearest.getDistance()) {
				nearest = scannedRobotEvent;
			}
		}
		scannedRobotEvents.clear();
		if (nearest == null) {
			return new Observation(this, getX(), getY(), getEnergy(), getHeading(),
					getGunHeading(), null, null, null, null);
		}
		return new Observation(this, getX(), getY(), getEnergy(), getHeading(), 
				getGunHeading(), nearest.getDistance(), nearest.getBearing(), 
				nearest.getHeading(), nearest.getEnergy());
	}
	
	@Override
	public void onHitByBullet(HitByBulletEvent e) {
		events.add(new EventWrapper(e));
	}

	@Override
	public void onBulletHit(BulletHitEvent e) {
		events.add(new EventWrapper(e));
	}

	@Override
	public void onDeath(DeathEvent e) {
		events.add(new EventWrapper(e));
	}

	@Override
	public void onWin(WinEvent e) {
		events.add(new EventWrapper(e));
	}

	@Override
	public void onScannedRobot(ScannedRobotEvent e) {
		scannedRobotEvents.add(e);
	}
	
	@Override
	public void onBattleEnded(BattleEndedEvent e) {
		serializeKnowledge();
	}
	
	private static class EventWrapper {
		private final Event event;
		
		private final double reward;
		
		private EventWrapper(Event event, double reward) {
			this.event = event;
			this.reward = reward;
		}

		public EventWrapper(HitByBulletEvent event) {
			this(event, -event.getPower());
		}
		
		public EventWrapper(BulletHitEvent event) {
			this(event, event.getBullet().getPower());
		}

		public EventWrapper(WinEvent event) {
			this(event, 200);
		}
		
		public EventWrapper(DeathEvent event) {
			this(event, -200);
		}
	}

	private static class Observation implements Serializable {
		
		private final HashCalculator hashCalculator;
		
		private final double posX;
		
		private final double posY;
		
		private final double energy;
		
		private final double heading;
		
		private final double gunHeading;
		
		private final Double nearestRobotDistance;
		
		private final Double nearestRobotBearing;
		
		private final Double nearestRobotHeading;
		
		private final Double nearestRobotEnergy;

		private Observation(Robot robot, double posX, double posY, double energy, double heading,
							double gunHeading, Double nearestRobotDistance, Double nearestRobotBearing, Double nearestRobotHeading, Double nearestRobotEnergy) {
			this.hashCalculator = new HashCalculator(robot, BUCKETS);
			this.posX = posX;
			this.posY = posY;
			this.energy = energy;
			this.heading = heading;
			this.gunHeading = gunHeading;
			this.nearestRobotDistance = nearestRobotDistance;
			this.nearestRobotBearing = nearestRobotBearing;
			this.nearestRobotHeading = nearestRobotHeading;
			this.nearestRobotEnergy = nearestRobotEnergy;
		}

		@Override
		public int hashCode() {
			return hashCalculator.hash(this);
		}

		@Override
		public boolean equals(Object o) {
			if (this == o) return true;
			if (!(o instanceof Observation)) return false;
			Observation that = (Observation) o;
			return Double.compare(that.posX, posX) == 0 &&
					Double.compare(that.posY, posY) == 0 &&
					Double.compare(that.energy, energy) == 0 &&
					Double.compare(that.heading, heading) == 0 &&
					Double.compare(that.gunHeading, gunHeading) == 0 &&
					Objects.equals(nearestRobotDistance, that.nearestRobotDistance) &&
					Objects.equals(nearestRobotBearing, that.nearestRobotBearing) &&
					Objects.equals(nearestRobotHeading, that.nearestRobotHeading) &&
					Objects.equals(nearestRobotEnergy, that.nearestRobotEnergy);
		}

		private static class HashCalculator implements Serializable {

			private final int minX;

			private final int maxX;

			private final int minY;
			
			private final int maxY;

			private final int buckets;
			
			
			private HashCalculator(Robot robot, int buckets) {
				this.buckets = buckets;
				minX = 0;
				maxX = (int) robot.getBattleFieldWidth();
				minY = 0;
				maxY = (int) robot.getBattleFieldHeight();
			}
			
			private int hash(Observation observation) {
				int minAngle = MIN_ANGLE;
				int maxAngle = MAX_ANGLE;
				if(observation.nearestRobotDistance != null) {
					return Objects.hash(
							bucket(minX, maxX, observation.posX),
							bucket(minY, maxY, observation.posY),
							bucket(MIN_ENERGY, MAX_ENERGY, observation.energy),
							bucket(minAngle, maxAngle, observation.heading),
							bucket(minAngle, maxAngle, observation.gunHeading),
							bucket(MIN_ENERGY, MAX_ENERGY, observation.nearestRobotEnergy),
							bucket(MIN_ANGLE, MAX_ANGLE, observation.nearestRobotBearing),
							bucket(MIN_ANGLE, MAX_ANGLE, observation.nearestRobotHeading),
							bucket(MIN_DISTANCE, MAX_DISTANCE, observation.nearestRobotDistance));
				}
				return Objects.hash(
						bucket(minX, maxX, observation.posX),
						bucket(minY, maxY, observation.posY),
						bucket(MIN_ENERGY, MAX_ENERGY, observation.energy),
						bucket(minAngle, maxAngle, observation.heading),
						bucket(minAngle, maxAngle, observation.gunHeading));
			}
			
			private int bucket(int min, int max, double value) {
				int span = max - min;
				int bucketSpan = span / buckets;
				double offset = value - min;
				return (int) offset / bucketSpan;
			}
		}
	}
	
	private static class RobotRandomActionFactory {
		
		
		private final Random random = new Random();
		
		public RobotAction randomAction() {
			int rand = random.nextInt(7);
			switch (rand) {
				default:
				case 0: 
					return new GoAhead(randomizeDistance());
				case 1:
					return new Fire(randomizePower());
				case 2:
					return new TurnGunLeft(randomizeAngle());
				case 3:
					return new TurnGunRight(randomizeAngle());
				case 4:
					return new TurnLeft(randomizeAngle());
				case 5:
					return new TurnRight(randomizeAngle());
				case 6:
					return new GoBack(randomizeDistance());
			}
		}
		
		
		private double randomizePower() {
			return random.nextDouble() * MAX_ENERGY;
		}
		
		private double randomizeDistance() {
			return random.nextDouble() * MAX_DISTANCE;
		}
		
		private double randomizeAngle() {
			return random.nextInt(MAX_ANGLE + 1);
		}
	}
	
	private interface RobotAction extends Serializable {
		
		void invoke(Robot robot);
	}
	
	private static abstract class ParametrizedAction implements RobotAction {

		protected final double value;

		protected ParametrizedAction(double value) {
			this.value = value;
		}

		@Override
		public boolean equals(Object o) {
			if (this == o)
				return true;
			if (o == null || getClass() != o.getClass())
				return false;
			ParametrizedAction that = (ParametrizedAction) o;
			return Double.compare(that.value, value) == 0;
		}

		@Override
		public int hashCode() {
			return Objects.hash(value, this.getClass().getName());
		}
	}
	
	private static class Fire extends ParametrizedAction {

		protected Fire(double power) {
			super(power);
		}

		@Override
		public void invoke(Robot robot) {
			robot.fire(value);
		}
	}
	
	private static class GoAhead extends ParametrizedAction {

		protected GoAhead(double distance) {
			super(distance);
		}

		@Override
		public void invoke(Robot robot) {
			robot.ahead(value);
		}
	}
	
	private static class GoBack extends ParametrizedAction {

		protected GoBack(double distance) {
			super(distance);
		}

		@Override
		public void invoke(Robot robot) {
			robot.back(value);
		}
	}
	
	private static class TurnGunRight extends ParametrizedAction {

		protected TurnGunRight(double angle) {
			super(angle);
		}

		@Override
		public void invoke(Robot robot) {
			robot.turnGunRight(value);
		}
	}
	
	private static class TurnGunLeft extends ParametrizedAction {

		protected TurnGunLeft(double angle) {
			super(angle);
		}

		@Override
		public void invoke(Robot robot) {
			robot.turnGunLeft(value);
		}
	}
	
	private static class TurnRight extends ParametrizedAction {

		protected TurnRight(double angle) {
			super(angle);
		}

		@Override
		public void invoke(Robot robot) {
			robot.turnRight(value);
		}
	}
	
	private static class TurnLeft extends ParametrizedAction {

		protected TurnLeft(double angle) {
			super(angle);
		}

		@Override
		public void invoke(Robot robot) {
			robot.turnLeft(value);
		}
	}
}
