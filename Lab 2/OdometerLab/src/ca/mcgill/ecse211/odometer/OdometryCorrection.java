/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import java.util.ArrayList;

import lejos.hardware.Audio;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;

public class OdometryCorrection implements Runnable {
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odometer;

	// Define colorsensor
	EV3ColorSensor colSensor = new EV3ColorSensor(SensorPort.S1);
	// Define the tile size
	private static final double TILE_SIZE = 30.48;

	/**
	 * This is the default class constructor. An existing instance of the odometer
	 * is used. This is to ensure thread safety.
	 * 
	 * @throws OdometerExceptions
	 */
	public OdometryCorrection() throws OdometerExceptions {

		this.odometer = Odometer.getOdometer();

	}

	/**
	 * Here is where the odometer correction code should be run.
	 * 
	 * @throws OdometerExceptions
	 */
	// run method (required for Thread)
	public void run() {
		long correctionStart, correctionEnd;

		// Keep track of how many lines have passed
		int passedLines = 0;

		// Store previously stored values
		double origX = 0, origY = 0, origTh = 0;

		while (true) {
			correctionStart = System.currentTimeMillis();

			// TODO Trigger correction (When do I have information to correct?)
			// Check what colour the light sensor detects
			colSensor.setFloodlight(Color.RED);
			SensorMode lightVal = colSensor.getColorID();
			
			String printLight = "Light level: " + lightVal;
			LCD.drawString(printLight, 0, 3);
			
			// If detected colour is black (passes a line)
			if (false) {
				// Beep when black line passed
				Sound.beep();

				// Get current X, Y and Theta values of odometer
				double data[] = null;
				try {
					data = Odometer.getOdometer().getXYT();
				} catch (OdometerExceptions e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				}

				// TODO Calculate new (accurate) robot position
				// Check if robot has started moving (to avoid false color sensor positives)
				if (data[1] > 10) {
					// Increment when line passed
					passedLines++;

					// Print number of lines passed
					String print = "Lines Passed: " + passedLines;
					LCD.drawString(print, 0, 4);
					
					
					
					if (data[2] > 345 || data[2] < 15) {
						origTh = 0;
						if (data[1] < (TILE_SIZE/2 + 15)) {
							origY = 0;
						} else {
							origY += TILE_SIZE;
						}
					} else if (data[2] > 75 || data[2] < 105) {
						origTh = 90;
						if (data[0] < (TILE_SIZE/2 + 15)) {
							origX = 0;
						} else {
							origX += TILE_SIZE;
						}
					} else if (data[2] > 165 || data[2] < 195) {
						origTh = 180;
						if (data[1] < origY - 15) {
							origY -= TILE_SIZE / 2;
						} else {
							origY -= TILE_SIZE;
						}
					} else if (data[2] > 255 || data[2] < 285) {
						origTh = 270;
						if (data[0] < origX - 15) {
							origX -= TILE_SIZE / 2;
						} else {
							origX -= TILE_SIZE;
						}
					}

					// TODO Update odometer with new calculated (and more accurate) vales
					odometer.setXYT(origX, origY, origTh);

					// Pause thread so that it does not read the same black line twice
					try {
						Thread.sleep(20);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}

			}

			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here
				}
			}

		}
	}
}
