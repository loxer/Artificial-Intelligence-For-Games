package s0562844;

import org.lwjgl.*;
import org.lwjgl.opengl.GL11;
import org.lwjgl.util.vector.Vector2f;

import java.awt.Polygon;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.lang.Math;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.Stack;

import javax.swing.Timer;

import lenz.htw.ai4g.ai.AI;
import lenz.htw.ai4g.ai.DriverAction;
import lenz.htw.ai4g.ai.Info;

public class SuperAI extends AI {

	// Settings
	private static final float SLOWING_RADIUS = 100;
	private static final float TURNING_TOLERANCE = 0.005F;

	private static final float LENGTH_OF_carOrientationVector = 100;

	private static final int NUMBER_OF_HELPER_COORDINATES = 729;
	private static final int MAXIMUM_DISTANCE_BETWEEN_POINTS = 5000;
	private static final int CLOSEST_RADIUS_TO_OBSTACLE = 120;

	private static final int DISTANCE_FOR_NEW_ROUTE_CHECK = 1000;

	private static final int DISTANCE_MINIMUM_FOR_EXTRA_COSTS = 1300;
	private static final int HIGHEST_EXTRA_COSTS = 7000;

	private static final int TIME_FOR_UNSTUCK = 30; // in frames
	private static final int DISTANCE_STUCK_CHECK = 20;
	private static final int TIME_FOR_REACHING_CHECKPOINT = 10; // in seconds
	// private static final float DISTANCE_REACHING_CHECK = 500;

	// Car Information Variables
	private float speed = 0;
	private float valueOfTooMuchDrift = 0;
	private float desiredSpeed = 0;
	private float drivingSpeed = 0.5F;
	private float rotatingSpeed = 0.1F;
	private float destinationAngleCar = 0;
	private float drivingDistanceToRoutePoint = 0;
	private float drivingDistanceToCheckpoint = 0;
	private float accel; // not in use yet
	private Boolean resetDestination = true;
	private Boolean keepTurning = true;
	private Boolean destinationIsLeft = true;
	private Vector2f carPos = new Vector2f();

	private float carOrientationX;
	private float carOrientationY;
	private Vector2f carOrientationVector = new Vector2f();
	private Line2D carLine = new Line2D.Float();
	private Point2D carLocation = new Point2D.Float(info.getX(), info.getY());

	// Track Information Variables
	private int width = info.getTrack().getWidth();
	private int height = info.getTrack().getHeight();
	private float orientationAngleWorld;
	private float destinationAngleWorld;

	private int amountOfObstaclePoints = 0;

	private Boolean newCheckpoint = true;
	private int archivedCheckpoints = 0;

	// Polygon Information
	private Terrain obstacles;
	private Terrain fastZones;
	private Terrain slowZones;
	private Polygon[] obstaclesPolygon = info.getTrack().getObstacles();
	private Point2D[] obstaclePoints;
	private Line2D[] obstacleLines;
	private Rectangle2D[] obstacleCorners;
	private Point2D[] fastPoints;
	private Line2D[] fastLines;
	private Point2D[] slowPoints;
	private Line2D[] slowLines;
	private Rectangle2D[] slowCorners;

	// Relevant Stuff for Graphs
	private Boolean directConnection = false;
	private Vertex[] route;
	private Boolean checkForNewRoute = false;
	private Boolean newRoutePoint = true;
	private int currentRoutePoint = 1;
	private int numberOfPotentialPoints;
	private Point2D[] helperCoordinates;
	private Vertex[] vertices = new Vertex[numberOfPotentialPoints];
	private Vertex currentDrivingDestination;
	private Vertex currentCheckpoint;
	private Vertex currentPosition = new Vertex();
	private Boolean xWasBigger = null;
	private Boolean yWasBigger = null;
	private int dijkstraCounter = 0;

	// Problem Solver
	private static final int HIGHEST_EXTRA_COSTS_FOR_PROBLEM_POINTS = 15000;
	private static final int DISTANCE_MINIMUM_FOR_PROBLEM_POINTS = 1500;
	private static final int SLOWING_RADIUS_AT_FAILURE_POINTS = 1000;

	private int distanceMinimumForProblemPoints = DISTANCE_MINIMUM_FOR_PROBLEM_POINTS;
	private float sizeOfObstacleCorners = 15;

	private ArrayList<Point2D> pointsOfFailure = new ArrayList<Point2D>();
	private Point2D carLocationBeforeFailure = new Point2D.Float(info.getX(), info.getY());
	private int resetCounter = 0;
	private int totalResets = 0;
	private int stuckCounter;

	// Timer Stuff
	private int frames = 0;
	private int seconds = 0;
	private int minutes = 0;
	private int countdown = 0;
	private float lastCheckedDistance = 0;

	// Other Variables for testCode()
	private int xWorkaround = 0;
	private int yWorkaround = 0;
	private Line2D showLine = new Line2D.Float();
	private ArrayList<Point2D> expensivePoints = new ArrayList<Point2D>();

	public SuperAI(Info info) {
		super(info);
		enlistForTournament(562844);
		doThingsFirst();
		doThingsOnce();
		prepareStart();
	}

	private void doThingsFirst() {
		amountOfObstaclePoints = Terrain.countPolygonPoints(obstaclesPolygon);
		calculateSizeOfObstacleCorners();
	}

	private void doThingsOnce() {
		// collectObstaclesInformation();
		collectInformationFromPolygons();
		fillHelperPositions();
		findConnectingPoints();
	}

	private void prepareStart() {
		setCarAndGoalVertices();
		dijkstra(currentPosition);
	}

	private void calculateSizeOfObstacleCorners() {
		if (amountOfObstaclePoints < 100) {
			sizeOfObstacleCorners = 30;
		} else {
			sizeOfObstacleCorners = 15;
		}
	}

	private void collectInformationFromPolygons() {
		this.obstacles = new Terrain(obstaclesPolygon, sizeOfObstacleCorners);
		this.obstaclePoints = obstacles.getPoints();
		this.obstacleLines = obstacles.getLines();
		this.obstacleCorners = obstacles.getCorners();

		this.fastZones = new Terrain(info.getTrack().getFastZones());
		this.fastPoints = fastZones.getPoints();
		this.fastLines = fastZones.getLines();

		this.slowZones = new Terrain(info.getTrack().getSlowZones());
		this.slowPoints = slowZones.getPoints();
		this.slowLines = slowZones.getLines();
		this.slowCorners = slowZones.getCorners();

	}

	private void printLength() {
		int counter = 0;
		for (int i = 0; i < slowLines.length; i++) {
			double length = Point2D.distanceSq(slowLines[i].getX1(), slowLines[i].getY1(), slowLines[i].getX2(),
					slowLines[i].getY2());
			if (length == 10000.0) {
				counter++;
				System.out.println(slowLines[i].getX1() + "/" + slowLines[i].getY1() + " -> " + slowLines[i].getX2()
						+ "/" + slowLines[i].getY2());
				System.out.println(length);

			}
		}
		System.out.println(counter);
		System.out.println();
	}

	private void fillHelperPositions() {
		int divider = (int) Math.sqrt(NUMBER_OF_HELPER_COORDINATES);
		numberOfPotentialPoints = divider * divider;

		ArrayList<Point2D> helperCoordinatesList = new ArrayList<Point2D>();

		float pointsDistanceInX = (float) width / divider;
		float pointsDistanceInY = (float) height / divider;

		for (int i = 0; i < divider; i++) {
			for (int j = 0; j < divider; j++) {
				Point2D potentialHelper = new Point2D.Float(width - pointsDistanceInX * i - pointsDistanceInX * 0.5f,
						height - pointsDistanceInY * j - pointsDistanceInY * 0.5f);
				if (!obstaclePointToClose(potentialHelper) && !obstacles.isInsideHere(potentialHelper)) {
					helperCoordinatesList.add(potentialHelper);
				}
			}
		}

		helperCoordinates = new Point2D[helperCoordinatesList.size()];
		vertices = new Vertex[helperCoordinates.length];

		for (int i = 0; i < helperCoordinates.length; i++) {
			helperCoordinates[i] = helperCoordinatesList.get(i);
			vertices[i] = new Vertex(helperCoordinates[i]);
		}
	}

	private void findConnectingPoints() {
		for (int i = 0; i < vertices.length; i++) {
			for (int j = i + 1; j < vertices.length; j++) {
				if (helperCoordinates[i].distanceSq(helperCoordinates[j]) < MAXIMUM_DISTANCE_BETWEEN_POINTS) {

					if (!pointsInDifferentZones(helperCoordinates[i], helperCoordinates[j])) {

						Line2D connection = new Line2D.Float(helperCoordinates[i], helperCoordinates[j]);
						if (!obstacleBetween(connection)) {
							vertices[i].setEdgeAndPoint(connection, vertices[j]);
							vertices[j].setEdgeAndPoint(connection, vertices[i]);
						}
					}
				}
			}
			vertices[i].setReady(getExtraCosts(vertices[i].getLocation()));
			// vertices[i].setReady(0);
		}
		printNumberOfConnections();
	}

	private Boolean pointsInDifferentZones(Point2D point1, Point2D point2) {
		Boolean pointOneInsideSlowZone = slowZones.isInsideHere(point1);
		Boolean pointTwoInsideSlowZone = slowZones.isInsideHere(point2);
		
		if (pointOneInsideSlowZone ^ pointTwoInsideSlowZone) {
			return true;
		}
		
		Boolean pointOneInsideFastZone = fastZones.isInsideHere(point1);
		Boolean pointTwoInsideFastZone = fastZones.isInsideHere(point2);

		if (pointOneInsideFastZone ^ pointTwoInsideFastZone) {
			return true;
		}

		if (pointOneInsideSlowZone && pointTwoInsideFastZone || pointTwoInsideSlowZone && pointTwoInsideFastZone) {			
			return true;
		}

		return false;
	}

	private float getExtraCosts(Point2D point) {
		float extraCosts = 0;
		float closestObstacle = Float.MAX_VALUE;

		for (int i = 0; i < obstaclePoints.length; i++) {
			float distanceToObstacle = (float) point.distanceSq(obstaclePoints[i]);
			if (DISTANCE_MINIMUM_FOR_EXTRA_COSTS > distanceToObstacle && distanceToObstacle < closestObstacle) {
				closestObstacle = distanceToObstacle;
				extraCosts = HIGHEST_EXTRA_COSTS * (1 - distanceToObstacle / DISTANCE_MINIMUM_FOR_EXTRA_COSTS);
				expensivePoints.add(point);
			}
		}
		return extraCosts;
	}

	private void addExtraCosts(Point2D problemPoint) {
		for (int i = 0; i < vertices.length; i++) {
			float extraCosts = 0;
			extraCosts = calculateNewCosts(vertices[i].getLocation(), problemPoint);
			extraCosts = (float) Math.pow(extraCosts, resetCounter + 1); // make it much higher if it happens again
			vertices[i].setExtraCosts(extraCosts);
		}
	}

	private float calculateNewCosts(Point2D point, Point2D problemPoint) {
		float costs = 0;
		float distanceToPoint = (float) point.distanceSq(problemPoint);
		if (distanceMinimumForProblemPoints > distanceToPoint) {
			costs = HIGHEST_EXTRA_COSTS_FOR_PROBLEM_POINTS * (1 - distanceToPoint / distanceMinimumForProblemPoints);
		}
		return costs;
	}

	private void makeAdjustmentsAfterFailure() {
		setPointOfFailure();
		addExtraCosts(pointsOfFailure.get(resetCounter));
		resetCounter++;
		distanceMinimumForProblemPoints *= resetCounter;
		totalResets++;

		sizeOfObstacleCorners *= resetCounter * 1.5f;
		this.obstacleCorners = Redo.resizeObstacleCorners(obstaclePoints, sizeOfObstacleCorners);
	}

	@Override
	public DriverAction update(boolean wasResetAfterCollision) {
		if (wasResetAfterCollision) {
			makeAdjustmentsAfterFailure();
		}

		// Gathering information
		setTimer();
		setUpLocation();
		setOrientation();
		calculateDistances();
		calculateAngleToDestination();
		calculateDestinationFromMyPerspective();
		// calculateDestinationFromMyPerspective2();
		checkDirectionOfDestination();
		setCarOrientation();
		setCarOrientationVector();
		checkForFailure();
		// timeForChange();

		// Driving methods
		lookForRoute();
		calculateAcceleration();
		checkKeepTurning();
		rotatingSpeed = RotatingAcceleration.calculate(destinationAngleCar, info.getAngularVelocity(), speed,
				drivingDistanceToRoutePoint, drivingDistanceToCheckpoint, directConnection, resetCounter);
		// align(drivingDistance);
		// correctDrivingDirection();
		// arrive();
		drivingSpeed = SpeedAcceleration.calculate(destinationAngleCar, info.getAngularVelocity(), speed,
				drivingDistanceToRoutePoint, drivingDistanceToCheckpoint, pointOfFailureIsClose(), directConnection,
				resetCounter);

		// float turningSpeed = 0;
		//
		// if (info.getVelocity().x > 28f) {
		// turningSpeed = 1f;
		// }
		// System.out.println(turningSpeed);
		// checkValues();
		// return new DriverAction(1f, turningSpeed);

		/*
		 * first value = speed second value: if positive -> drive left || if negative ->
		 * drive right
		 */

		return new DriverAction(drivingSpeed, rotatingSpeed);

	}

	private void checkValues() {
		System.out.println("getVelocity: " + info.getVelocity());
		if (info.getVelocity().x == 28) {
			System.out.println("SpeedTime: " + seconds);

		}
	}

	private void setTimer() {
		frames++;
		seconds = (frames / 30) % 60;
		minutes = (frames / 30) / 60;
	}

	private void calculateDistances() {
		// directionVertex = calculateDirectionVector(destinationPos);
		drivingDistanceToRoutePoint = calculateDrivingDistance(currentDrivingDestination);
		drivingDistanceToCheckpoint = calculateDrivingDistance(currentCheckpoint);

		if (drivingDistanceToRoutePoint < DISTANCE_FOR_NEW_ROUTE_CHECK && !directConnection) {
			checkForNewRoute = true;
		}
	}

	private void updateCurrentRoutePoint(Vertex newDestination) {
		if (currentDrivingDestination != newDestination) {
			currentDrivingDestination = newDestination;
			newRoutePoint = true;
		}
	}

	private Line2D connectionToCheckpoint() {
		setUpLocation();
		return new Line2D.Float(carLocation, currentCheckpoint.getLocation());
	}

	private void setUpLocation() {
		carLocation.setLocation(info.getX(), info.getY());
		setCarPosition();
	}

	private void setCarPosition() {
		carPos.set(info.getX(), info.getY());
	}

	private void checkForFailure() {
		if (frames % 60 == 0) { // check each 2 seconds
			carLocationBeforeFailure.setLocation(carLocation.getX(), carLocation.getY());
		}
	}

	private void setPointOfFailure() {
		pointsOfFailure.add(carLocationBeforeFailure);
	}

	private Boolean newCheckpoint() {
		Point2D checkpoint = new Point2D.Float((float) info.getCurrentCheckpoint().x,
				(float) info.getCurrentCheckpoint().y);

		if (checkpoint.distanceSq(currentCheckpoint.getLocation()) == 0) {
			return false;
		}
		archivedCheckpoints++;
		return true;
	}

	private void lookForRoute() {

		Line2D connection = new Line2D.Float(carLocation, currentCheckpoint.getLocation());

		if (!obstacleBetween(connection)) {
			directConnection = true;
			updateCurrentRoutePoint(currentCheckpoint);
		} else {
			directConnection = false;
		}

		if (newCheckpoint()) {
			for (int i = 0; i < vertices.length; i++) {
				vertices[i].resetExtraCosts();
			}
			calculateSizeOfObstacleCorners();
			this.obstacleCorners = Redo.resizeObstacleCorners(obstaclePoints, sizeOfObstacleCorners); // apparently gets
																										// done after
																										// every
																										// checkpoint,
																										// even without
																										// failures
			checkForNewRoute = true;
		}

		connection.setLine(new Line2D.Float(carLocation, route[currentRoutePoint].getLocation()));

		if (obstacleBetween(connection) || checkForNewRoute) {
			prepareDijkstra();
			resetCounter = 0;
			pointsOfFailure.clear();
			return;
		}

		if (route.length > currentRoutePoint + 1) {
			connection.setLine(carLocation, route[currentRoutePoint + 1].getLocation());
			// showLine = connection;
			if (!obstacleBetween(connection)) {
				currentRoutePoint++;
				updateCurrentRoutePoint(route[currentRoutePoint]);
			}
		}
	}

	private void prepareDijkstra() {
		setCarAndGoalVertices();
		resetAllVerticesForDijkstra();
		dijkstra(currentPosition);
		getBooleansRight();
	}

	private void setCarAndGoalVertices() {
		Point2D checkpointLocation = new Point2D.Float((float) info.getCurrentCheckpoint().getX(),
				(float) info.getCurrentCheckpoint().getY());
		currentCheckpoint = new Vertex(checkpointLocation);

		Point2D carLocation = new Point2D.Float((float) info.getX(), (float) info.getY());
		currentPosition = new Vertex(carLocation);

		findVerticesConnectedToThisVertex(currentCheckpoint);
		findVerticesConnectedToThisVertex(currentPosition);

		Line2D connection = new Line2D.Float(currentCheckpoint.getLocation(), currentPosition.getLocation());
		if (!obstacleBetween(connection)) {
			currentCheckpoint.setEdgeAndPoint(connection, currentPosition);
			currentPosition.setEdgeAndPoint(connection, currentCheckpoint);
		}
		currentCheckpoint.setReady(0);
		currentPosition.setReady(0);
	}

	private void resetAllVerticesForDijkstra() {
		for (Vertex vertex : vertices) {
			vertex.resetForQueue();
		}
	}

	private void findVerticesConnectedToThisVertex(Vertex vertex) {
		int doubleDistanceForConnections = MAXIMUM_DISTANCE_BETWEEN_POINTS * 2;
		for (int i = 0; i < vertices.length; i++) {
			if (vertex.getLocation().distanceSq(vertices[i].getLocation()) < doubleDistanceForConnections) {
				Line2D connection = new Line2D.Float(vertex.getLocation(), vertices[i].getLocation());
				if (!obstacleBetween(connection)) {
					vertex.setEdgeAndPoint(connection, vertices[i]);
				}
			}
		}
	}

	private void getBooleansRight() {
		checkForNewRoute = false;
	}

	private void setOrientation() {
		orientationAngleWorld = info.getOrientation();
	}

	private void calculateAngleToDestination() {

		destinationAngleWorld = (float) Math.atan2(currentDrivingDestination.getLocation().getY() - info.getY(),
				currentDrivingDestination.getLocation().getX() - info.getX());
	}

	private float calculateDrivingDistance(Vertex vertex) {
		return (float) carLocation.distanceSq(vertex.getLocation());
	}

	private int randomNumber(int number) {
		return (int) (Math.random() * number);
	}

	private void calculateAcceleration() {
		accel = info.getMaxAbsoluteAcceleration() - info.getAngularVelocity();
	}

	private void checkDirectionOfDestination() {
		if (Math.signum(destinationAngleCar) == -1) {
			destinationIsLeft = false;
		} else {
			destinationIsLeft = true;
		}
	}

	private void arrive() {
		float distance = (float) Math.sqrt(drivingDistanceToRoutePoint);
		if (distance < 1) {
			drivingSpeed = 0;
			resetDestination = false;
		} else {
			desiredSpeed = distance * info.getMaxVelocity() / SLOWING_RADIUS;
			float speedX = info.getVelocity().x;
			float speedY = info.getVelocity().y;
			drivingSpeed = (float) ((desiredSpeed - ((speedX * speedX) + (speedY * speedY))) / 2);
		}
	}

	private void calculateDestinationFromMyPerspective2() { // sth wrong
		if (orientationAngleWorld > Math.PI) {
			destinationAngleCar = (float) ((orientationAngleWorld - 2 * Math.PI));
		} else if (orientationAngleWorld < (-1) * (Math.PI)) {
			destinationAngleCar = (float) ((orientationAngleWorld + 2 * Math.PI));
		} else {
			destinationAngleCar = orientationAngleWorld;
		}
	}

	private void calculateDestinationFromMyPerspective() {
		double orientationCorrection;
		if (Math.signum(orientationAngleWorld) == -1) {
			orientationCorrection = 2 * Math.PI + orientationAngleWorld;
		} else {
			orientationCorrection = orientationAngleWorld;
		}

		if (Math.signum(destinationAngleWorld) == -1) {
			destinationAngleCar = (float) (2 * Math.PI - orientationCorrection + destinationAngleWorld);
		} else {
			destinationAngleCar = (float) (destinationAngleWorld - orientationCorrection);
		}

		if (destinationAngleCar > Math.PI) {
			destinationAngleCar = (float) ((-2) * Math.PI + destinationAngleCar);
		}
		if (destinationAngleCar < (-1) * Math.PI) {
			destinationAngleCar = (float) (2 * Math.PI + destinationAngleCar);
		}
	}

	private Boolean pointOfFailureIsClose() {
		if (pointsOfFailure.isEmpty()) {
			return false;
		}
		for (int i = 0; i < pointsOfFailure.size(); i++) {
			double distance = Math.pow(carLocation.getX() - pointsOfFailure.get(i).getX(), 2)
					+ Math.pow(carLocation.getY() - pointsOfFailure.get(i).getY(), 2);
			if (distance < SLOWING_RADIUS_AT_FAILURE_POINTS) {
				return true;
			}
		}
		return false;
	}

	private Boolean probablyStucked() {
		if (newRoutePoint) {
			newRoutePoint = false;
			stuckCounter = 0;
			return false;
		}
		stuckCounter++;
		if (stuckCounter % TIME_FOR_UNSTUCK == 0) {
			Boolean noChange = Math.abs(lastCheckedDistance - drivingDistanceToRoutePoint) < DISTANCE_STUCK_CHECK;
			System.out.println(noChange);
			System.out.println(lastCheckedDistance + " - " + drivingDistanceToRoutePoint + " = "
					+ Math.abs(lastCheckedDistance - drivingDistanceToRoutePoint) + " < " + DISTANCE_STUCK_CHECK);

			if (noChange) {
				return true;
			}
		}
		lastCheckedDistance = drivingDistanceToRoutePoint;
		return false;
	}

	private void velocityMatching() {
		// float accel = info.getMaxVelocity() - info.getVelocity();
	}

	private void setCarOrientation() {
		carOrientationX = (float) Math.cos(orientationAngleWorld);
		carOrientationY = (float) Math.sin(orientationAngleWorld);
	}

	private void setCarOrientationVector() {
		carOrientationVector.setX(carOrientationX * LENGTH_OF_carOrientationVector + carPos.getX());
		carOrientationVector.setY(carOrientationY * LENGTH_OF_carOrientationVector + carPos.getY());
		carLine = new Line2D.Float(carPos.x, carPos.y, carOrientationVector.x, carOrientationVector.y);
	}

	private Line2D setCarOrientation2(float angle) {
		float carOrientationX = (float) Math.cos(angle);
		float carOrientationY = (float) Math.sin(angle);

		float x = (carOrientationX * LENGTH_OF_carOrientationVector + carPos.getX());
		float y = (carOrientationY * LENGTH_OF_carOrientationVector + carPos.getY());
		Line2D newLine = new Line2D.Float(carPos.x, carPos.y, x, y);
		return newLine;
	}

	private Boolean obstacleBetween(Line2D lineToDestination) {
		for (int i = 0; i < obstacleLines.length; i++) {
			if (lineToDestination.intersectsLine(obstacleLines[i])
					|| obstacleCorners[i].intersectsLine(lineToDestination)) {
				return true;
			}
		}
		return false;
	}

	private Boolean obstacleCornerBetween(Line2D lineToDestination) {
		for (int i = 0; i < obstacleCorners.length; i++) {
			if (obstacleCorners[i].intersectsLine(lineToDestination)) {
				return true;
			}
		}
		return false;
	}

	// not working as expected, more tests necessary
	private Boolean obstaclePointToClose(Point2D point) {
		for (int i = 0; i < obstaclePoints.length; i++) {
			if (point.distanceSq(obstaclePoints[i]) < CLOSEST_RADIUS_TO_OBSTACLE) {
				return true;
			}
		}
		return false;
	}

	// private Boolean isInsideObstacle(Point2D point) {
	// for (int i = 0; i < obstaclesPolygon.length; i++) {
	// if (obstaclesPolygon[i].contains(point)) {
	// return true;
	// }
	// }
	// return false;
	// }

	private void dijkstra(Vertex start) {
		ArrayList<Vertex> queue = new ArrayList<Vertex>();

		start.setCosts(0);
		queue.add(start);

		while (!queue.isEmpty()) {
			Vertex vertex = queue.get(0);
			for (int i = 0; i < vertex.numberOfEdges(); i++) {
				Vertex connectedVertex = vertex.getConnectedPoint(i);
				if (!connectedVertex.isFinished()) {
					if (!connectedVertex.isQueued()) {
						queue.add(connectedVertex);
						connectedVertex.setQueued();
					}
					float costs = vertex.getLenght(i);
					if (vertex.getCosts() + costs < connectedVertex.getCosts()) {

						connectedVertex.setCosts(vertex.getCosts() + costs);
						connectedVertex.setPredecessor(vertex);
					}
				}
			}
			vertex.setFinished();
			queue.remove(0);
		}
		dijkstraFinish();
	}

	private void dijkstraFinish() {
		for (int i = 0; i < currentCheckpoint.numberOfEdges(); i++) {
			Vertex connectedVertex = currentCheckpoint.getConnectedPoint(i);
			float costs = connectedVertex.getCosts() + currentCheckpoint.getLenght(i);
			if (costs < currentCheckpoint.getCosts()) {
				currentCheckpoint.setCosts(costs);
				currentCheckpoint.setPredecessor(connectedVertex);
			}
		}
		setRoute();
		dijkstraCounter++;
	}

	private void setRoute() {
		ArrayList<Vertex> routeBackwards = new ArrayList<Vertex>();
		routeBackwards.add(currentCheckpoint);
		Vertex predecessor = currentCheckpoint.getPredecessor();
		while (predecessor != null) {
			routeBackwards.add(predecessor);
			predecessor = predecessor.getPredecessor();
		}

		int routePoints = routeBackwards.size();

		if (routePoints < 2) { // no route found

			routeBackwards.add(getRandomRoutePoint());
			routePoints++;
			checkForNewRoute = true;
		}

		route = new Vertex[routePoints];

		for (int i = 0; i < route.length; i++) {
			route[i] = routeBackwards.get(routePoints - 1);
			routePoints--;
		}

		currentRoutePoint = 1;
		updateCurrentRoutePoint(route[currentRoutePoint]);

	}

	private Vertex getRandomRoutePoint() {

		Point2D carLocation = new Point2D.Float((float) info.getX(), (float) info.getY());

		for (int i = 0; i < vertices.length; i++) {
			Line2D connection = new Line2D.Float(carLocation, vertices[i].getLocation());
			if (!obstacleBetween(connection)) {
				return vertices[i];
			}
		}

		Random random = new Random();
		Vertex randomPoint = vertices[random.nextInt(vertices.length)];

		return randomPoint;
	}

	private void checkKeepTurning() {
		if (makeNegativeToPositive(destinationAngleCar) < TURNING_TOLERANCE) {
			keepTurning = false;
		} else {
			keepTurning = true;
		}
	}

	private float makeNegativeToPositive(float x) { // same as Math.abs();
		if (x < 0) {
			x = -x;
		}
		return x;
	}

	@Override
	public String getName() {
		return "loxer";
	}

	@Override
	public String getTextureResourceName() {
		return "/s0562844/car.png";
	}

	@Override
	public void doDebugStuff() {

		// Show stuff
		// Show.expensivePoints(expensivePoints);
		Show.carOrientationVector(carPos, carOrientationVector);
		Show.route(route);
		Show.helperPoints(vertices);
		Show.lineToCheckpoint(carPos, info.getCurrentCheckpoint().getX(), info.getCurrentCheckpoint().getY());
		Show.lineToDestination(carPos, currentDrivingDestination);
		Show.obstacleCorners(obstacleCorners, sizeOfObstacleCorners);
		Show.aHelperPointsConnections(vertices);

		for (int i = 0; i < vertices.length; i++) {
			if (fastZones.isInsideHere(vertices[i].getLocation()) || slowZones.isInsideHere(vertices[i].getLocation())) {
				Show.lines(vertices[i].getEdges());
			}
		}

		// Show.lineOfFutureRoutePoint(route, currentRoutePoint);
		// Show.nextLine(showLine);

		// Print stuff
		printDrivingInformation();
		// printTestMethods();
		// printObstacleCoordinates();
		// printObstacleLines();
		// printDijkstra();

	}

	private void printDrivingInformation() {
		// probablyStucked();
		if (!(resetCounter > 0
				|| ((archivedCheckpoints == 9) && drivingDistanceToCheckpoint < 1000 && frames % 20 == 0))
		/* || !probablyStucked() */)
			return;
		System.out.println("---Driving Information---");
		System.out.println("carPos: " + carPos);
		System.out.println("destinationPos: " + currentDrivingDestination);
		// System.out.println("directionVector: " + directionVector);
		System.out.println("orientationAngle: " + orientationAngleWorld);
		System.out.println("destinationAngle: " + destinationAngleWorld);
		System.out.println("destinationAngleCar: " + destinationAngleCar);
		System.out.println("getAngularVelocity: " + info.getAngularVelocity());
		System.out.println("rotatingSpeed: " + rotatingSpeed);
		System.out.println("speed: " + speed);
		System.out.println("drivingSpeed: " + drivingSpeed);
		System.out.println("drivingDistanceToRoutePoint: " + drivingDistanceToRoutePoint);
		System.out.println("drivingDistanceToCheckpoint: " + drivingDistanceToCheckpoint);
		System.out.println("keepTurning: " + keepTurning);
		System.out.println("destinationIsLeft: " + destinationIsLeft);
		System.out.println("route.length: " + route.length);
		System.out.println("currentRoutePoint: " + (currentRoutePoint + 1));
		System.out.println("dijkstraCounter: " + dijkstraCounter);
		System.out.println("directConnection: " + directConnection);
		System.out.println("archivedCheckpoints: " + archivedCheckpoints);
		System.out.println("resetCounter: " + resetCounter);
		System.out.println("totalResets: " + totalResets);
		if (pointsOfFailure.size() > 0) {
			System.out.println("last pointOfFailure: " + (float) pointsOfFailure.get(resetCounter - 1).getX() + "/"
					+ (float) pointsOfFailure.get(resetCounter - 1).getY());
		}
		System.out.println("Timer: " + minutes + ":" + seconds);
		// System.out.println("probablyStucked(): " + probablyStucked());
		System.out.println();
		System.out.println("-----------------");
		System.out.println();
	}

	private void printTestMethods() {
		System.out.println("---Method Information---");
		System.out.println("getAngularVelocity: " + info.getAngularVelocity());
		System.out.println("getVelocity: " + info.getVelocity());
		System.out.println("getMaxVelocity: " + info.getMaxVelocity());
		System.out.println("getMaxBackwardVelocity: " + info.getMaxBackwardVelocity());
		System.out.println("getMaxAbsoluteAcceleration: " + info.getMaxAbsoluteAcceleration());
		System.out.println("getMaxAbsoluteAngularAcceleration: " + info.getMaxAbsoluteAngularAcceleration());
		System.out.println("getMaxAbsoluteAngularVelocity: " + info.getMaxAbsoluteAngularVelocity());
		System.out.println("accel: " + accel + " = info.getMaxAbsoluteAcceleration() - info.getAngularVelocity()");
		System.out.println("-----------------");
		System.out.println();
	}

	private void printSingleArgument(String whatIsTheArgument, float argument) {
		System.out.println("---Single Information---");
		System.out.println(whatIsTheArgument + ": " + argument);
		System.out.println("-----------------");
		System.out.println();
	}

	private void printObstacleCoordinates() {
		System.out.println("---Obstacle Coordinates---");
		for (int i = 0; i < obstaclePoints.length; i++) {
			System.out.println(obstaclePoints[i].getX() + "/" + obstaclePoints[i].getY());
		}
		System.out.println("-----------------");
		System.out.println();
	}

	private void printObstacleLines() {
		System.out.println("---Obstacle Lines---");
		for (int i = 0; i < obstacleLines.length; i++) {
			System.out.println(obstacleLines[i].getX1() + "/" + obstacleLines[i].getY1());
			System.out.println(obstacleLines[i].getX2() + "/" + obstacleLines[i].getY2());
			System.out.println();
		}
		System.out.println("-----------------");
		System.out.println();
	}

	private void printNumberOfConnections() {
		System.out.println("---Number of Connecting Lines---");
		int connections = 0;
		for (int i = 0; i < vertices.length; i++) {
			connections += vertices[i].numberOfEdges();
		}

		System.out.println("Number of vertices: " + vertices.length + " || Number of connections: " + connections);
		System.out.println("-----------------");
		System.out.println();
	}

	private void printDijkstra() {
		System.out.println();

		for (int i = 0; i < route.length; i++) {
			System.out.println("Checkpoint: " + route[i] + "   || Distance: " + currentCheckpoint.getCosts());
		}

		System.out.println(info.getCurrentCheckpoint().x + "/" + info.getCurrentCheckpoint().y);
	}

	// private double guessingShortestWay() {
	// return sqrt(() + ()); // page 9 in folien2 ("A*-Heuristik: Beispiel")
	// }

}
