package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.navigation.Waypoint;

public class Navigation {

	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	private static final double TILE_SIZE = 30.48;

	private static EV3LargeRegulatedMotor leftMotor;
	private static EV3LargeRegulatedMotor rightMotor;
	private static double leftRadius;
	private static double rightRadius;
	private static double track;

	public Navigation(EV3LargeRegulatedMotor leftmotor, EV3LargeRegulatedMotor rightmotor, double leftRadius,
			double rightRadius, double track) {
		Navigation.leftMotor = leftmotor;
		Navigation.rightMotor = rightmotor;
		Navigation.leftRadius = leftRadius;
		Navigation.rightRadius = rightRadius;
		Navigation.track = track;

	}

	public static void drive(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double leftRadius,
			double rightRadius, double track) {

		// reset the motors
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(3000);
		}

		// Input waypoints
		int waypoints[][] = new int[][] { { 0, 2 }, { 1, 1 }, { 2, 2 }, { 2, 1 }, { 1, 0 } };

		// Sleep for 2 seconds
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) { // There is nothing to be done here

		}

		double absAngle = 0, prevAngle = 0, deltaAngle = 0, lenX = 0, lenY = 0, len = 0, deltaX = 0, deltaY = 0;
		// Decide whether to turn left or right
		boolean turnLeft = false;

		double temp = 0;
		for (int i = 0; i < 5; i++) {

			// Get displacement to travel on X and Y axis
			if (i != 0) {
				deltaX = waypoints[i][0] - waypoints[i - 1][0];
				deltaY = waypoints[i][1] - waypoints[i - 1][1];
				lenX = (deltaX) * TILE_SIZE;
				lenY = (deltaY) * TILE_SIZE;
			} else {
				lenX = (waypoints[i][0]) * TILE_SIZE;
				lenY = (waypoints[i][1]) * TILE_SIZE;
			}

			// Length to point (hypothenuse)
			len = Math.hypot(lenX, lenY);

			if ((deltaX) == 0) {
				if (deltaY < 0) {
					absAngle = 180;
				} else {
					absAngle = 0;
				}
			} else if ((deltaY) == 0) {
				if (deltaX < 0) {
					absAngle = 270;
				} else {
					absAngle = 90;
				}
			} else {
				absAngle = Math.toDegrees(Math.atan(lenY / lenX));
			}

			// If the value of absolute angle is negative, loop it back
			if (absAngle < 0) {
				absAngle = 360 - Math.abs(absAngle);
			}

			// Get change in angle we want
			deltaAngle = absAngle - prevAngle;

			// Check if we want to move left or right
			if (deltaAngle < 0) {
				turnLeft = true;
				deltaAngle = Math.abs(deltaAngle);
			} else {
				turnLeft = false;
			}

			// Print passed lines
			String printX = "X: " + waypoints[i][0];
			String printY = "Y: " + waypoints[i][1];
			LCD.drawString(printX, 0, 1);
			LCD.drawString(printY, 0, 2);
			// Print angle
			String printAbs = "absAngle: " + absAngle;
			String printPrev = "prevAngle: " + prevAngle;
			String printAngle = "Angle: " + deltaAngle;
			LCD.drawString(printAbs, 0, 4);
			LCD.drawString(printPrev, 0, 5);
			LCD.drawString(printAngle, 0, 6);

			// turn 90 degrees clockwise leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);

			if (turnLeft) {
				leftMotor.rotate(-convertAngle(leftRadius, track, deltaAngle - 3), true);
				rightMotor.rotate(convertAngle(rightRadius, track, deltaAngle - 3), false);
			} else {
				leftMotor.rotate(convertAngle(leftRadius, track, deltaAngle + 3), true);
				rightMotor.rotate(-convertAngle(rightRadius, track, deltaAngle + 3), false);
			}

			// Save current angle prevAngle = absAngle;

			// drive forward two tiles leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);

			leftMotor.rotate(convertDistance(leftRadius, len), true);
			rightMotor.rotate(convertDistance(rightRadius, len), false);

		}

	}

	/**
	 * This method causes the robot to travel to the absolute field location (x, y),
	 * specified in tile points
	 */
	private static double prevAngle = 0, prevX = 0, prevY = 0;

	public void travelTo(double x, double y) {
		double odometer[] = { 0, 0, 0 }, lenX = 0, absAngle = 0, len = 0, lenY = 0, deltaX = 0, deltaY = 0;

		try {
			odometer = Odometer.getOdometer().getXYT();
		} catch (OdometerExceptions e) {
			// Do nothing lol
			e.printStackTrace();
		}
		x = x * TILE_SIZE;
		y = y * TILE_SIZE;
		// Get displacement to travel on X and Y axis
		deltaX = x - odometer[0];
		deltaY = y - odometer[1];

		// Length to point (hypothenuse)
		len = Math.hypot(Math.abs(deltaX), Math.abs(deltaY));

		absAngle = Math.toDegrees(Math.atan2(deltaX, deltaY));

		// If the value of absolute angle is negative, loop it back
		if (absAngle < 0)
			absAngle = 360 - Math.abs(absAngle);

		turnTo(absAngle);

		prevX = x;
		prevY = y;

		// drive forward two tiles
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		leftMotor.rotate(convertDistance(leftRadius, len), true);
		rightMotor.rotate(convertDistance(rightRadius, len), false);

	}

	/**
	 * This method causes the robot to turn (on point) to the absolute heading theta
	 */
	public static void turnTo(double theta) {
		boolean turnLeft = false;
		double deltaAngle = 0;
		// Get change in angle we want
		deltaAngle = theta - prevAngle;

		// Check if we want to move left or right
		if (deltaAngle < 0) {
			turnLeft = true;
			deltaAngle = Math.abs(deltaAngle);
		} else {
			turnLeft = false;
		}

		// turn 90 degrees clockwise
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		if (turnLeft) {
			leftMotor.rotate(-convertAngle(leftRadius, track, deltaAngle), true);
			rightMotor.rotate(convertAngle(rightRadius, track, deltaAngle), false);
		} else {
			leftMotor.rotate(convertAngle(leftRadius, track, deltaAngle), true);
			rightMotor.rotate(-convertAngle(rightRadius, track, deltaAngle), false);
		}

		// Save current angle
		prevAngle = theta;

	}

	/**
	 * This method returns true if another thread has called travelTo() or turnTo()
	 * and the method has yet to return; false otherwise
	 */
	public static boolean isNavigating() {
		boolean navigating = false;

		
		
		
		return navigating;
	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}
