package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class USLocalisation extends UltraSonicPoller {

	public static EV3LargeRegulatedMotor leftMotor;
	public static EV3LargeRegulatedMotor rightMotor;

	// Initialize variables
	public static double WHEEL_RAD = 0;
	public static double TRACK = 0;

	// Difining motor class variables
	public USLocalisation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double TRACK,
			double WHEEL_RAD) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.WHEEL_RAD = WHEEL_RAD;
		this.TRACK = TRACK;
	}

	// Variables to store our threshold and noise margin
	static double d = 0, k = 0.04, prevDist = 0;

	// To check if we still want to loop
	static boolean running = true;

	// Run falling edge method
	public static void fallingEdge() {
		double dist = 0;
		double[] fallEdgeAngle = { 0, 0 }, angle = { 0, 0 }, odometer = null;
		int i = 0;

		// Slow down motors
		leftMotor.setSpeed(150);
		rightMotor.setSpeed(150);

		// Rotate robot
		leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 720), true);
		rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 720), true);

		// While robot is turning, get distance readings and add them up
		while (leftMotor.getRotationSpeed() != 0 || rightMotor.getRotationSpeed() != 0) {
			i++;
			// Upper limit of distance since sensor reads "infinity" if no object detected
			// near it
			if (distance > 2.2)
				distance = (float) 2.2;
			d += distance;
		}
		// Take average of all readings
		d = d / i;

		// Rotate the robot
		leftMotor.forward();
		rightMotor.backward();

		// While we want to loop
		while (running) {

			// Get data from odometer
			try {
				odometer = Odometer.getOdometer().getXYT();
			} catch (OdometerExceptions e) {
				// Do nothing
				e.printStackTrace();
			}

			// Set dist as the distance retrieved from the US sensor
			dist = distance;

			// If dist is less than out d+k, and if we have not detected a possible falling
			// edge already
			if (dist < d + k && prevDist >= d + k && fallEdgeAngle[0] == 0) {
				// Store the value of the current angle
				fallEdgeAngle[0] = odometer[2];
				Sound.beepSequenceUp();
				// If dist is less then d - k, we have not detected a falling edge yet and we
				// have detected a possible falling edge
			} else if (dist < d - k && fallEdgeAngle[1] == 0.0 && fallEdgeAngle[0] != 0) {
				// Save value of current angle
				fallEdgeAngle[1] = odometer[2];
				Sound.beepSequence();

				if (angle[0] == 0) {
					angle[0] = (fallEdgeAngle[0] + fallEdgeAngle[1]) / 2;

					// Rotate the robot the other way
					leftMotor.backward();
					rightMotor.forward();

					fallEdgeAngle[0] = 0;
					fallEdgeAngle[1] = 0;
					Sound.beep();

				} else if (angle[0] != 0 && angle[1] == 0) {
					leftMotor.stop(true);
					rightMotor.stop(true);

					angle[1] = (fallEdgeAngle[0] + fallEdgeAngle[1]) / 2;
					localiseAngle(angle[0], angle[1]);
				}

				// If dist > d + k, then clear all values since no falling edge is detected
			} else if (dist > d + k && (fallEdgeAngle[0] != 0 || fallEdgeAngle[1] != 0)) {
				fallEdgeAngle[0] = 0;
				fallEdgeAngle[1] = 0;
				Sound.beep();
			}

			prevDist = dist;
		}

	}

	public static void risingEdge() {
		double dist = 0;
		double[] riseEdgeAngle = { 0, 0 }, angle = { 0, 0 }, odometer = null;
		int i = 0;

		// Slow down motors
		leftMotor.setSpeed(150);
		rightMotor.setSpeed(150);

		// Rotate robot
		leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 720), true);
		rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 720), true);

		// While robot is turning, get distance readings and add them up
		while (leftMotor.getRotationSpeed() != 0 || rightMotor.getRotationSpeed() != 0) {
			i++;
			// Upper limit of distance since sensor reads "infinity" if no object detected
			// near it
			if (distance > 2.2)
				distance = (float) 2.2;
			d += distance;
		}
		// Take average of all readings
		d = d / i;

		// Rotate the robot
		leftMotor.forward();
		rightMotor.backward();

		// While we want to loop
		while (running) {

			// Get data from odometer
			try {
				odometer = Odometer.getOdometer().getXYT();
			} catch (OdometerExceptions e) {
				// Do nothing
				e.printStackTrace();
			}

			// Set dist as the distance retrieved from the US sensor
			dist = distance;

			// If dist is less than out d+k, and if we have not detected a possible falling
			// edge already
			if (dist > d - k && prevDist <= d - k && riseEdgeAngle[0] == 0) {
				// Store the value of the current angle
				riseEdgeAngle[0] = odometer[2];
				Sound.beepSequenceUp();
				// If dist is less then d - k, we have not detected a falling edge yet and we
				// have detected a possible falling edge
			} else if (dist > d + k && riseEdgeAngle[1] == 0.0 && riseEdgeAngle[0] != 0) {
				// Save value of current angle
				riseEdgeAngle[1] = odometer[2];
				Sound.beepSequence();

				if (angle[0] == 0) {
					angle[0] = (riseEdgeAngle[0] + riseEdgeAngle[1]) / 2;

					// Rotate the robot the other way
					leftMotor.backward();
					rightMotor.forward();

					riseEdgeAngle[0] = 0;
					riseEdgeAngle[1] = 0;
					Sound.beep();

				} else if (angle[0] != 0 && angle[1] == 0) {
					leftMotor.stop(true);
					rightMotor.stop();

					angle[1] = (riseEdgeAngle[0] + riseEdgeAngle[1]) / 2;
					localiseAngle(angle[0], angle[1]);
				}

				// If dist > d + k, then clear all values since no falling edge is detected
			} else if (dist < d - k && (riseEdgeAngle[0] != 0 || riseEdgeAngle[1] != 0)) {
				riseEdgeAngle[0] = 0;
				riseEdgeAngle[1] = 0;
				Sound.beep();
			}

			prevDist = dist;
		}

	}

	private static void localiseAngle(double angle1, double angle2) {
		double deltaTh = 0;
		double[] odometer = { 0, 0, 0 };

		try {
			odometer = Odometer.getOdometer().getXYT();
		} catch (OdometerExceptions e1) {
			// Do nothing
			e1.printStackTrace();
		}

		if (angle1 < angle2) {
			deltaTh = 45 - (angle1 + angle2) / 2;
		} else {
			deltaTh = 225 - (angle1 + angle2) / 2;
		}

		deltaTh += odometer[2];
		
		if (deltaTh >= 360) {
			deltaTh -= 360;
		}
		
		// Turn left
		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, deltaTh), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, deltaTh), false);

		LCD.drawString("deltaTheta: " + deltaTh, 0, 6);

		try {
			Odometer.getOdometer().setTheta(0.0);
		} catch (OdometerExceptions e) {
			// Do nothing
			e.printStackTrace();
		}

		running = false;
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
