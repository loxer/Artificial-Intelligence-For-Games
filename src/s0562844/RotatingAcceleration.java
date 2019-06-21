package s0562844;

import lenz.htw.ai4g.ai.Info;

public class RotatingAcceleration {

	private static float maxAbsoluteAngularAcceleration;
	private static float getMaxAbsoluteAngularVelocity;
	
	
	private static float destAngleCar = 0;
	private static float absDestAngleCar = 0;
	private static float getAngularVelocity = 0;
	private static float absGetAngularVelocity = 0;
	private static float speed = 0;
	private static float rotatingSpeed = 0;
	private static float drivingDistanceToRoutePoint = 0;
	private static float drivingDistanceToCheckpoint = 0;
	private static Boolean rotatingLeft;
	private static Boolean rotatingDifferentToGoal;
	private static Boolean connectionDirect;
	private static int resetCount;

	public RotatingAcceleration(Info info) {
		maxAbsoluteAngularAcceleration = info.getMaxAbsoluteAngularAcceleration();
		getMaxAbsoluteAngularVelocity = info.getMaxAbsoluteAngularVelocity();
	}

	public static float calculate(float destAngleCarNew, float getAngularVelocityNew, float speedNew,
			float distanceToRoutePoint, float distanceToCheckpoint, Boolean directConnection, int resetCount) {
		destAngleCar = destAngleCarNew;
		absDestAngleCar = Math.abs(destAngleCar);

		getAngularVelocity = getAngularVelocityNew;
		absGetAngularVelocity = Math.abs(getAngularVelocity);

		speed = speedNew;
		drivingDistanceToRoutePoint = distanceToRoutePoint;
		drivingDistanceToCheckpoint = distanceToCheckpoint;

		rotatingLeft = Math.signum(destAngleCar) == 1;
		rotatingDifferentToGoal = Math.signum(destAngleCar) != Math.signum(getAngularVelocity);

		connectionDirect = directConnection;
		
		return method6(); // for testing
//		return currentSetting(resetCount);
		

	}
	
	private static float currentSetting(int resetCount) {
		if(resetCount % 4 == 0) {
			return method2();
		} else if(resetCount % 4 == 1) {
			return method1();
		} else if(resetCount % 4 == 2) {
			return method3();
		} else {
			return method4();
		}		
	}

	private static float method1() {
		float tolerance = 0.0005f;
		float breakAngle = 0.3f;

		Boolean rotating2Fast = false;

		float gegenBremsbeschleunigung = 0.8f;
		float targetAngle;

		if (getAngularVelocity > absDestAngleCar) {
			rotating2Fast = true;
		}

		if (Math.signum(getAngularVelocity) == 1) {
			rotatingLeft = true;
		} else {
			rotatingLeft = false;
		}

		if (absDestAngleCar < tolerance) {
			rotatingSpeed = 0;
		} else {
			if (absDestAngleCar < breakAngle) {
				targetAngle = destAngleCar * 1.25f;
			} else {
				targetAngle = 1.5f * Math.signum(destAngleCar);
			}
			rotatingSpeed = (destAngleCar - getAngularVelocity);
		}

		if (rotating2Fast) {
			rotatingSpeed = gegenBremsbeschleunigung * getAngularVelocity;
			if (rotatingLeft) {
				rotatingSpeed = (-1) * rotatingSpeed;
			}
		} else {
			rotatingSpeed = (float) (destAngleCar * 2.5 / (Math.PI));
			if (speed <= 0.3f) {
				rotatingSpeed = 1;
			}
		}
		return rotatingSpeed;
	}

	// Stuff for method2

	private static float method2() {

		float tolerance = 0.0005f;

		if (absDestAngleCar < tolerance) {
			rotatingSpeed = 0;
		} else {
			rotatingSpeed = destAngleCar - getAngularVelocity;
		}
		return rotatingSpeed;
	}

	// Stuff for method3

	private static float method3() {
		float tolerance = 0.0005f;

		if (rotatingDifferentToGoal) {
			rotatingSpeed = destAngleCar - getAngularVelocity*1.5f;
		} else {
			if (absDestAngleCar + absGetAngularVelocity < tolerance) {
				rotatingSpeed = 0;
			} else {
				rotatingSpeed = 1.5f * destAngleCar - getAngularVelocity;
			}
		}
		
		rotatingSpeed += getAngularVelocity * (-1); 
		
		return rotatingSpeed;
	}
	
	private static float method4() {
		float tolerance = 0.0005f;
		
		if(connectionDirect) {
			rotatingSpeed = destAngleCar;
		}

		if (rotatingDifferentToGoal) {
			rotatingSpeed = destAngleCar - getAngularVelocity*1.5f;
		} else {
			if (absDestAngleCar + absGetAngularVelocity < tolerance) {
				rotatingSpeed = 0;
			} else {
				rotatingSpeed = 1.5f * destAngleCar - getAngularVelocity;
			}
		}
		
		rotatingSpeed += getAngularVelocity * (-1); 
		
		return rotatingSpeed;
	}
	
	
	
	public static float method5() { // book Method
	float maxAngularAcceleration = maxAbsoluteAngularAcceleration; // =1
	float maxRotation = 1f;
	float targetRadius = 0.001f;
	float slowRadius = 0.2f;
	float targetRotation = 0f;
	float timeToTarget = 0.1f;
	float rotationSize = absDestAngleCar;
	float rotationDirection = Math.signum(destAngleCar);
	float characterRotation = getAngularVelocity;
	float steering;
		
	if(rotationSize < targetRadius) {
		return 0;
	}
	
	if(rotationSize > slowRadius) {
		targetRotation = maxRotation;
	} else {
		targetRotation = maxRotation * rotationSize / slowRadius;
	}
	
	targetRotation *= rotationDirection;
	
	steering = targetRotation - characterRotation;
	steering /= timeToTarget;
		
		return steering;
	}
	
	public static float method6() { 
		float maxAngularAcceleration = maxAbsoluteAngularAcceleration; // =1
		float maxRotation = 1f;
		float targetRadius = 0.001f;
		float slowRadius = 0.2f;
		float targetRotation = 0f;
		float timeToTarget = 0.1f;
		float rotationSize = absDestAngleCar;
		float rotationDirection = Math.signum(destAngleCar);
		float characterRotation = getAngularVelocity;
		float steering;
			
		if(rotationSize < targetRadius) {
			return 0;
		}
		
		if(rotationSize > slowRadius) {
			targetRotation = maxRotation;
		} else {
			targetRotation = maxRotation * rotationSize / slowRadius;
		}
		
		targetRotation *= rotationDirection;
		
		steering = targetRotation - characterRotation;
		steering /= timeToTarget;
			
			return steering;
		}
	
	
	
	public static Boolean rotatingDifferentToGoal() {
		return rotatingDifferentToGoal;
	}
	
	
	

	// public void align(float angleToCheckpoint) {
	// float angVel = 0;
	// angleToCheckpoint = destinationAngleCar;
	// if (Math.abs(angleToCheckpoint) < TURNING_TOLERANCE) {
	// rotatingSpeed = 0;
	// // return angVel;
	// } else {
	// float desiredSpeed = angleToCheckpoint * info.getMaxVelocity() / 10;
	// angleToCheckpoint = desiredSpeed - info.getAngularVelocity();
	// angVel = (desiredSpeed - info.getAngularVelocity()) / 2f;
	// rotatingSpeed = angVel;
	// // return angVel;
	// }
	// }

}
