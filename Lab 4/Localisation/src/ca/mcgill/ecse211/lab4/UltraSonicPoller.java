package ca.mcgill.ecse211.lab4;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class UltraSonicPoller extends Thread {

	// Sensor object
	public static final Port usPort = SensorPort.S4;

	// Initialize sensors
	public static double distance = 0;

	private static int FILTER_OUT = 3;
	private int filterControl;
	
	// Get distance from US
	public void run() {
		@SuppressWarnings("resource") // Because we don't bother to close this resource
		SensorModes usSensor = new EV3UltrasonicSensor(usPort);
		SampleProvider usDistance = usSensor.getMode("Distance");
		float[] usData = new float[usDistance.sampleSize()];
		
		
		while (true) {
			usDistance.fetchSample(usData, 0);
			String print = "Distance: " + usData[0];
			LCD.drawString(print, 0, 4);
			

			double usDist = (int) usData[0];
			// using the filter that was used in lab1 (P_Controller)
			if (((usDist >= .5 && filterControl < FILTER_OUT))) {
				// Increment filter control
				filterControl++;
			} else if (usDist >= .5) {
				// We have repeated large values
				distance = usDist;
			} else {
				// distance went below 0.5 so reset filter 
				filterControl = 0;
				distance = usDist;
			}
			
			
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}

		}
	}

}
