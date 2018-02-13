package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.lab4.Display;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Localisation extends Thread {

	// Motor Objects
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	// Initialize LCD
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();

	// Initialize variables
	public static final double WHEEL_RAD = 2.1;
	public static final double TRACK = 14.7;

	public static void main(String[] args) throws OdometerExceptions {

		lcd.drawString("Left: Falling Edge", 0, 1);
		lcd.drawString("Right: Rising Edge", 0, 2);

		// Show option to select rising edge or falling edge
		final int option = Button.waitForAnyPress();

		if (option == Button.ID_LEFT || option == Button.ID_RIGHT) {
			// clear the display
			lcd.clear();

			// Run the US Poller thread
			Thread usPoll = new UltraSonicPoller();
			(usPoll).start();

			// Odometer related objects
			Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);

			Display odometryDisplay = new Display(lcd); // No need to change

			// Start odometer and display threads
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();

			USLocalisation USLocal = new USLocalisation(leftMotor, rightMotor, TRACK, WHEEL_RAD);
			LSLocalisation LSLocal = new LSLocalisation(leftMotor, rightMotor, TRACK, WHEEL_RAD);
			
			LSLocal.lsLocalise();
			
			if (option == Button.ID_LEFT) {
				USLocal.fallingEdge();
			} else {
				USLocal.risingEdge();
			}
			Button.waitForAnyPress();
			usPoll.interrupt();


		} else {
			System.exit(0);
		}

		Button.waitForAnyPress();
		System.exit(0);
	}

}
