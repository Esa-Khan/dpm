package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController  implements UltrasonicController{

	/* Constants */
	private static final int MOTOR_SPEED = 200;
	private static final int FILTER_OUT = 20;

	private final int bandCenter;
	private final int bandWidth;
	private int distance;
	private int filterControl;
	
	private static EV3LargeRegulatedMotor leftMotor;
	private static EV3LargeRegulatedMotor rightMotor;
	private static double leftRadius;
	private static double rightRadius;
	private static double track;


	public PController(int bandCenter, int bandwidth) {
		this.bandCenter = bandCenter;
		this.bandWidth = bandwidth;
		this.filterControl = 0;

		leftMotor = Navigation.leftMotor;
		rightMotor= Navigation.rightMotor;
		leftRadius = Navigation.leftRadius;
		rightRadius = Navigation.rightRadius;
		track = Navigation.track;

		
		
		leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
		rightMotor.setSpeed(MOTOR_SPEED);
		leftMotor.forward();
		rightMotor.forward();
	}

	@Override
	public void processUSData(int distance) {

		// rudimentary filter - toss out invalid samples corresponding to null
		// signal.
		// (n.b. this was not included in the Bang-bang controller, but easily
		// could have).
		//
		if (distance >= 255 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
		} else if (distance >= 255) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.distance = distance;
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;
			this.distance = distance;
		}

		// Delcaring local variables
		int deltaSpeed = 1;
		int error = 0;
		Boolean left = false;

		// Calculate distance error
		error = distance - bandCenter;

		// Decide to move left or right
		if ((error) < 0)
			left = true;

		// Check if too close to wall, move backwards (-28 through trial and error)
		if (error < -28) {
			leftMotor.setSpeed(MOTOR_SPEED);
			rightMotor.setSpeed(MOTOR_SPEED);
			leftMotor.backward();
			rightMotor.backward();

			// Check if error greater than bandwidth
		} else if (Math.abs(error) > bandWidth) {
			rightMotor.forward();

			error = Math.abs(error);

			// Calculate speed change of outer wheel
			deltaSpeed = (int) (10 * error);
			// Limit increase of deltaSpeed
			if (deltaSpeed > MOTOR_SPEED * 2)
				deltaSpeed = 400;

			// Go in respective direction
			if (left) {
				leftMotor.setSpeed((deltaSpeed));
				leftMotor.backward();
			} else {
				leftMotor.setSpeed(deltaSpeed);
				leftMotor.forward();

				rightMotor.setSpeed((MOTOR_SPEED));
			}
		}

	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}

}
