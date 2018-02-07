package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.Waypoint;

public class Navigation {

	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	private static final double TILE_SIZE = 30.48;

	private static final int bandCenter = 20; // Offset from the wall (cm)
	private static final int bandWidth = 5; // Width of dead band (cm)

	public static EV3LargeRegulatedMotor leftMotor;
	public static EV3LargeRegulatedMotor rightMotor;
	public static double leftRadius;
	public static double rightRadius;
	public static double track;

	private static final Port usPort = LocalEV3.get().getPort("S3");

	// Define motor specifications for the navigation object
	public Navigation(EV3LargeRegulatedMotor leftmotor, EV3LargeRegulatedMotor rightmotor, double leftRadius,
			double rightRadius, double track) {
		Navigation.leftMotor = leftmotor;
		Navigation.rightMotor = rightmotor;
		Navigation.leftRadius = leftRadius;
		Navigation.rightRadius = rightRadius;
		Navigation.track = track;

	}

	/**
	 * This method causes the robot to travel to the absolute field location (x, y),
	 * specified in tile points
	 */
	private static double prevAngle = 0;

	public void travelTo(double x, double y) {
		// Define variables
		double odometer[] = { 0, 0, 0 }, absAngle = 0, len = 0, deltaX = 0, deltaY = 0;

	
		
		// Set navigating to true
		navigating = true;

		// Get odometer readings
		try {
			odometer = Odometer.getOdometer().getXYT();
		} catch (OdometerExceptions e) {
			// Do nothing lol
			e.printStackTrace();
		}

		// Convert X & Y coordinates to actual length (cm)
		x = x * TILE_SIZE;
		y = y * TILE_SIZE;

		// Set odometer reading angle as prev angle as well
		prevAngle = odometer[2];

		// Get displacement to travel on X and Y axis
		deltaX = x - odometer[0];
		deltaY = y - odometer[1];

		// Displacement to point (hypothenuse)
		len = Math.hypot(Math.abs(deltaX), Math.abs(deltaY));

		// Get absolute angle the robot must be facing
		absAngle = Math.toDegrees(Math.atan2(deltaX, deltaY));

		// If the value of absolute angle is negative, loop it back
		if (absAngle < 0)
			absAngle = 360 - Math.abs(absAngle);

		// Make robot turn to the absolute angle
		turnTo(absAngle);

		// Set robot speed
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		// Move distance to the waypoint after robot has adjusted angle
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

		// If deltaAngle is negative, loop it back
		if (deltaAngle < 0) {
			deltaAngle = 360 - Math.abs(deltaAngle);
		}

		// Check if we want to move left or right
		if (deltaAngle > 180) {
			turnLeft = true;
			deltaAngle = 360 - Math.abs(deltaAngle);
		} else {
			turnLeft = false;
		}

		// Set slower rotate speed
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		// Turn motors according to which direction we want to turn in
		if (turnLeft) {
			leftMotor.rotate(-convertAngle(leftRadius, track, deltaAngle), true);
			rightMotor.rotate(convertAngle(rightRadius, track, deltaAngle), false);
		} else {
			leftMotor.rotate(convertAngle(leftRadius, track, deltaAngle), true);
			rightMotor.rotate(-convertAngle(rightRadius, track, deltaAngle), false);
		}

	}

	/**
	 * This method returns true if another thread has called travelTo() or turnTo()
	 * and the method has yet to return; false otherwise
	 */
	static boolean navigating = false;

	public static boolean isNavigating() {
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
