package s0562844;

import org.lwjgl.util.vector.Vector2f;

import lenz.htw.ai4g.ai.Info;

public class SpeedAcceleration {

	private static final int SMALL_DISTANCE_TO_CHECKPOINT = 20000;

	private static float maxAbsoluteAngularAcceleration;
	private static float destAngleCar = 0;
	private static float absDestAngleCar = 0;
	private static float getAngularVelocity = 0;
	private static float absGetAngularVelocity = 0;
	private static float speedSq = 0;
	private static float accel = 0;
	private static float rotatingSpeed = 0;
	private static float drivingDistanceToRoutePoint = 0;
	private static float drivingDistanceToCheckpoint = 0;
	private static Boolean pointOfFailureIsClose;
	private static Boolean checkpointIsClose;
	private static Boolean routePointIsClose;
	private static Boolean connectionDirect;
	private static int resetCount;
	private static Vector2f directionVector;
	private static Vector2f velocity;

	public SpeedAcceleration(Info info) {

	}

	public static float calculate(float destAngleCarNew, float getAngularVelocityNew, float speedNew,
			float distanceToRoutePoint, float distanceToCheckpoint, Boolean failurePointIsClose,
			Boolean directConnection, int resetCount, Vector2f directionVec, Vector2f carVelocity) {
		destAngleCar = destAngleCarNew;
		absDestAngleCar = Math.abs(destAngleCar);

		getAngularVelocity = getAngularVelocityNew;
		absGetAngularVelocity = Math.abs(getAngularVelocity);

		speedSq = speedNew;
		velocity = carVelocity;
		directionVector = directionVec;
		drivingDistanceToRoutePoint = distanceToRoutePoint;
		drivingDistanceToCheckpoint = distanceToCheckpoint;

		if (drivingDistanceToCheckpoint < SMALL_DISTANCE_TO_CHECKPOINT) {
			checkpointIsClose = true;
		} else {
			checkpointIsClose = false;
		}

		if (drivingDistanceToRoutePoint < SMALL_DISTANCE_TO_CHECKPOINT) {
			routePointIsClose = true;
		} else {
			routePointIsClose = false;
		}

		pointOfFailureIsClose = failurePointIsClose;
		connectionDirect = directConnection;

		return method6(); // for testing
		// return currentSetting(resetCount);

	}

	private static float currentSetting(int resetCount) {
		if (resetCount % 4 == 0) {
			return method2();
		} else if (resetCount % 4 == 1) {
			return method1();
		} else if (resetCount % 4 == 2) {
			return method3();
		} else {
			return method4();
		}
	}

	private static float method1() {

		if (pointOfFailureIsClose) {
			accel = 0.5f;
			return accel;
		}

		accel = (drivingDistanceToCheckpoint * 2) / (speedSq * 40);
		if (accel > 1) {
			accel = 1;
		}
		accel = accel - (absGetAngularVelocity + absDestAngleCar / 2) / 1.5f;

		if (absDestAngleCar > 0.1f) {
			accel = 0.25f;
		}

		return accel;

	}

	private static float method2() {

		if (pointOfFailureIsClose) {
			accel = 0.5f;
			return accel;
		}

		if (checkpointIsClose) {
			if (absDestAngleCar > 0.3f) {
				return -0.3f;
			} else {
				return accel = drivingDistanceToCheckpoint / SMALL_DISTANCE_TO_CHECKPOINT + 0.5f;
			}
		}

		accel = (drivingDistanceToCheckpoint * 2) / (speedSq * 40);
		if (accel > 1) {
			accel = 1;
		}
		accel = accel - (absGetAngularVelocity + absDestAngleCar / 2) / 1.5f;

		if (absDestAngleCar > 0.3f) {
			accel = 0.25f;
		}

		return accel;
	}

	private static float method3() {

		if (pointOfFailureIsClose) {
			accel = 0.5f;
			return accel;
		}

		if (drivingDistanceToCheckpoint < 2000) {
			return 0.1f;
		}

		if (checkpointIsClose) {
			if (absDestAngleCar > 0.3f) {
				return -0.3f;
			} else {
				return 0.3f;
			}
		}

		accel = (absGetAngularVelocity + absDestAngleCar / 2) / 1.5f;

		if (absDestAngleCar > 0.3f) {
			accel = -0.25f;
		} else {
			accel = 1;
		}

		return accel;
	}

	private static float method4() {

		if (pointOfFailureIsClose) {
			accel = 0.5f;
			return accel;
		}

		if (connectionDirect && !RotatingAcceleration.rotatingDifferentToGoal()
				&& absDestAngleCar + absGetAngularVelocity < 0.1f) {
			if (drivingDistanceToCheckpoint < 1000) {
				return drivingDistanceToCheckpoint / 1000;
			} else {
				return 1;
			}
		}

		if (drivingDistanceToCheckpoint < 800) {
			return 0.3f;
		}

		if (routePointIsClose) {
			return (drivingDistanceToRoutePoint * 2) / (speedSq * 40);
		}

		if (checkpointIsClose) {
			if (absDestAngleCar > 0.15f) {
				return -0.3f;
			} else {
				return 1;
			}
		}

		if (absDestAngleCar > 0.15f) {
			return -0.25f;
		} else {
			return 1;
		}
	}

	private static float method5() {
		float steering = 0;
		float maxAcceleration = 6.5f;
		float maxSpeed = 28;
		float targetRadius = 5; // pixel
		float slowRadius = 100; // pixel
		float timeToTarget = 0.1f;
		float distance = (float) Math.sqrt(drivingDistanceToRoutePoint);
		float targetSpeed;		
		Vector2f targetVelocity;

		if (distance < targetRadius) {
			return 0.0f;
		}

		if (distance > slowRadius) {
			targetSpeed = maxSpeed;
		} else {
			targetSpeed = distance * maxSpeed / slowRadius;
		}

		targetVelocity = directionVector;
		targetVelocity.normalise();		
		targetVelocity.scale(targetSpeed);
		
		Vector2f.sub(targetVelocity, velocity, targetVelocity);		

		steering = (float) Math.sqrt(targetVelocity.x * targetVelocity.x + targetVelocity.y * targetVelocity.y);
		steering /= maxAcceleration;
		System.out.println("distance: " + distance);
		System.out.println("targetSpeed: " + targetSpeed);
		System.out.println("steering /= maxAcceleration: " + steering);
		
//		steering /= timeToTarget;
//		System.out.println("steering /= timeToTarget: " + steering);
		System.out.println();

		return steering;
	}
	
	private static float method6() {
		float steering = 0;
		float maxAcceleration = 6.5f;
		float maxSpeed = 28;
		float targetRadius = 5; // pixel
		float slowRadius = 25; // pixel
		float fastRadius = 75;
		float timeToTarget = 0.1f;
		float distance = (float) Math.sqrt(drivingDistanceToRoutePoint);
		float targetSpeed;
		float speed = (float) Math.sqrt(speedSq);
		float toleranceForAngularVelocity = 0.5f;
		Vector2f targetVelocity;
		
		System.out.println("distance: " + distance);
		System.out.println("speed: " + speed);
		System.out.println("getAngularVelocity: " + getAngularVelocity);

		if (distance < targetRadius) {
			return 0.0f;
		}
		
		if (distance > fastRadius && absGetAngularVelocity < toleranceForAngularVelocity) {
			return 1f;
		}

		if (distance > slowRadius) {
			targetSpeed = maxSpeed;
		} else {
			targetSpeed = distance * maxSpeed / slowRadius;
		}


		
		System.out.println("targetSpeed: " + targetSpeed);

//		steering = (float) (speed - Math.sqrt(velocity.x * velocity.x + velocity.y * velocity.y));
		
		
		
		
		steering = (targetSpeed - speed/2) / maxSpeed;
//		steering /= maxAcceleration;
		
		System.out.println("steering: " + steering);
		
		
//		steering /= timeToTarget;
		
		System.out.println();

		return steering;
	}

	private static void timer() {
		// if (newCheckpoint) {
		// if (frames - countdown < 30 * 3) { // 3 seconds
		// speed = 0.1f;
		// return;
		// } else {
		// newCheckpoint = false;
		// }
		// }
	}

}
