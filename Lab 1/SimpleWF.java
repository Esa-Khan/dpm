package wf1EV3;

import lejos.hardware.Button; // All references to classes in your code need to be included here
import lejos.hardware.ev3.LocalEV3; 
import lejos.hardware.lcd.TextLCD; 
import lejos.hardware.motor.Motor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;

// A direct, procedural implementation of the simple wall follower.
// This approach is simple and good for demonstration purposes
// but violates several criteria specified in the lab grading rubric
// The Lab 1 skeleton code introduces a more object-oriented approach.

public class SimpleWF {

  // Class Constants
  public static final int WALLDIST = 30; // Standoff distance to wall
  public static final int DEADBAND = 2; // Error threshold
  public static final int FWDSPEED = 200; // Default rotational speed of wheels
  public static final int DELTASPD = 100; // Bang-bang constant

  // Class Variables
  public static int wallDist = 0; // Measured distance to wall
  public static int distError = 0; // Error

  // Objects instantiated once in this class

  static TextLCD t = LocalEV3.get().getTextLCD();
  static RegulatedMotor leftMotor = Motor.A;
  static RegulatedMotor rightMotor = Motor.B;

  // Sensor set-up

  // 1. Allocate a port for each sensor

  static Port portUS = LocalEV3.get().getPort("S1");
  static Port portTouch = LocalEV3.get().getPort("S2");

  // 2. Create an instance for each sensor

  static SensorModes myUS = new EV3UltrasonicSensor(portUS);
  static SensorModes myTouch = new EV3TouchSensor(portTouch);

  // 3. Create an instance for each sensor in the desired measurement mode.

  static SampleProvider myDistance = myUS.getMode("Distance");
  static SampleProvider myTouchStatus = myTouch.getMode(0);

  // 4. Sensors return real-valued data; need to allocate buffers for each

  static float[] sampleUS = new float[myDistance.sampleSize()];
  static float[] sampleTouch = new float[myTouchStatus.sampleSize()];

  //
  // Main entry point - set display, start motors, enter polling loop.
  // (this is a very inefficient way to do things, as it runs as fast as the processor can)
  //

  public static void main(String[] args) {

    t.clear(); // Clear display
    t.drawString("Simple Wall F", 0, 0); // Print banner
    t.drawString("Distance: ", 0, 1);

    leftMotor.setSpeed(FWDSPEED); // Start moving forward
    rightMotor.setSpeed(FWDSPEED);
    leftMotor.forward();
    rightMotor.forward();

    //
    // Main control loop: read distance, determine error, adjust speed, and repeat

    boolean traveling = true;
    int status = 0;

    // Check for stop command or collisions

    while (traveling) {
      status = Button.getButtons(); // Check for abort
      myTouchStatus.fetchSample(sampleTouch, 0); // Check for collision
      // Abort if keypad pressed or touch sensor pressed
      if ((status == Button.ID_ENTER) || (sampleTouch[0] == 1)) {

        System.exit(0);
      }

      // Get sensor reading and update display

      myDistance.fetchSample(sampleUS, 0); // Get latest reading
      wallDist = (int) (sampleUS[0] * 100.0); // Scale to integer
      t.drawInt(wallDist, 4, 11, 1); // Print current sensor reading
      
      // -- Controller section --
      
      distError = WALLDIST - wallDist; // Compute error

      if (Math.abs(distError) <= DEADBAND) { // Within limits, same speed
        leftMotor.setSpeed(FWDSPEED); // Start moving forward
        rightMotor.setSpeed(FWDSPEED);
        leftMotor.forward();
        rightMotor.forward();
      }

      else if (distError > 0) { // Too close to the wall
        leftMotor.setSpeed(FWDSPEED);
        rightMotor.setSpeed(FWDSPEED - DELTASPD);
        leftMotor.forward();
        rightMotor.forward();
      }

      else if (distError < 0) { // Too far from wall
        leftMotor.setSpeed(FWDSPEED - DELTASPD);
        rightMotor.setSpeed(FWDSPEED);
        leftMotor.forward();
        rightMotor.forward();
      }
    }
  }
}
