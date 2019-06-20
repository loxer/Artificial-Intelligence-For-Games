package s0562844;

import lenz.htw.ai4g.ai.Info;

public class SpeedAcceleration {

	private static final int SMALL_DISTANCE_TO_CHECKPOINT = 20000;

	private static float maxAbsoluteAngularAcceleration;
	private static float destAngleCar = 0;
	private static float absDestAngleCar = 0;
	private static float getAngularVelocity = 0;
	private static float absGetAngularVelocity = 0;
	private static float speed = 0;
	private static float accel = 0;
	private static float rotatingSpeed = 0;
	private static float drivingDistanceToRoutePoint = 0;
	private static float drivingDistanceToCheckpoint = 0;
	private static Boolean pointOfFailureIsClose;
	private static Boolean checkpointIsClose;
	private static Boolean routePointIsClose;
	private static Boolean connectionDirect;
	private static int resetCount;

	public SpeedAcceleration(Info info) {

	}

	public static float calculate(float destAngleCarNew, float getAngularVelocityNew, float speedNew,
			float distanceToRoutePoint, float distanceToCheckpoint, Boolean failurePointIsClose,
			Boolean directConnection, int resetCount) {
		destAngleCar = destAngleCarNew;
		absDestAngleCar = Math.abs(destAngleCar);

		getAngularVelocity = getAngularVelocityNew;
		absGetAngularVelocity = Math.abs(getAngularVelocity);

		speed = speedNew;
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

		return method4(); // for testing
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

		accel = (drivingDistanceToCheckpoint * 2) / (speed * 40);
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

		accel = (drivingDistanceToCheckpoint * 2) / (speed * 40);
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
			return (drivingDistanceToRoutePoint * 2) / (speed * 40);
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
		float maxAcceleration = 100;
		float maxSpeed = 784;
		float targetRadius = 625; // pixel
		float slowRadius = 2500; // pixel
		float timeToTarget = 0.1f;
		float distance = drivingDistanceToRoutePoint;
		float targetSpeed;

		if (distance < targetRadius) {
			return 0;
		}

		if (distance > slowRadius) {
			targetSpeed = maxSpeed;
		} else {
			targetSpeed = maxSpeed * distance / slowRadius;
		}

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
