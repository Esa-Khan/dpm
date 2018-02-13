package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;

public class LSLocalisation extends UltraSonicPoller {

	public static EV3LargeRegulatedMotor leftMotor;
	public static EV3LargeRegulatedMotor rightMotor;

	public static final Port lightPort = SensorPort.S1;

	
	// Initialize variables
	public static double WHEEL_RAD = 0;
	public static double TRACK = 0;

	// Light threshold to detect a black sensor
	private static double lightThreshold = 0.20;

	// Speeds of motors
	private static int ROTATE_SPEED = 80;
	private static int FORWARD_SPEED = 160;

	// Difining motor class variables
	public LSLocalisation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double TRACK,
			double WHEEL_RAD) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.WHEEL_RAD = WHEEL_RAD;
		this.TRACK = TRACK;
	}

	public static void lsLocalise() {
		double[] odometer = { 0, 0, 0 };
		double len = 0, centreDist = 0;

		// Set color sensor object
		@SuppressWarnings("resource") // Because we don't bother to close this resource
		final EV3ColorSensor colorSensor = new EV3ColorSensor(lightPort);
		final SampleProvider lightSample = colorSensor.getRedMode();
		
		leftMotor.forward();
		rightMotor.forward();

		// Setup sampler
		colorSensor.setFloodlight(Color.RED);
		int sampleSize = lightSample.sampleSize();
		float[] sample = new float[sampleSize];

		while (true) {
			lightSample.fetchSample(sample, 0);

			while (sample[0] > lightThreshold) {
				try {
					Thread.sleep(100);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}

			leftMotor.stop(true);
			rightMotor.stop(true);

			try {
				odometer = Odometer.getOdometer().getXYT();
			} catch (OdometerExceptions e) {
				// Do nothing
				e.printStackTrace();
			}
			// Go back to original position
			leftMotor.rotate(-convertDistance(WHEEL_RAD, odometer[1]), true);
			rightMotor.rotate(-convertDistance(WHEEL_RAD, odometer[1]), false);

			// Rotate 90 degrees
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90.0), true);
			rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90.0), false);

			// Move parallel to y-axis
			leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);
			leftMotor.forward();
			rightMotor.forward();

			while (sample[0] > lightThreshold) {
				// try-catch from ultrasonic poller
				try {
					Thread.sleep(100);
				} catch (InterruptedException e) {
					// Auto-generated catch block
					e.printStackTrace();
				}
			}

			leftMotor.stop(true);
			rightMotor.stop(true);

			// Go back to original position
			leftMotor.rotate(-convertDistance(WHEEL_RAD, odometer[0]), true);
			rightMotor.rotate(-convertDistance(WHEEL_RAD, odometer[0]), false);

			// Face origin by rotating 45 degrees
			leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 45.0), true);
			rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 45.0), false);

			// Calculate distance between centre of robot and (0,0)
			len = Math.hypot(odometer[0], odometer[1]);
			centreDist = 6.5;
			leftMotor.rotate(convertDistance(WHEEL_RAD, distance + centreDist), true);
			rightMotor.rotate(convertDistance(WHEEL_RAD, distance + centreDist), false);

			// Turn to 0 degrees
			leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 45.0), true);
			rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 45.0), false);

			// Set odometer reading
			try {
				Odometer.getOdometer().setXYT(0, 0, 0);
			} catch (OdometerExceptions e) {
				// Do nothing
				e.printStackTrace();
			}
		}
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
