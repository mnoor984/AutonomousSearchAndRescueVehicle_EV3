package ca.mcgill.ecse211.project;
import static ca.mcgill.ecse211.project.Resources.*;

public class LightLocalizer {


    /**
   * Buffer (array) to store ColourSensor samples. Declared as an instance variable to avoid creating a new
   * array each time {@code readUsSample()} is called.
   */
  private static float colour[] = new float[colourSensor.sampleSize()];
 
  /**
  *stores the value of the colour returned  by the Colour sensor
  */
  private static int value_of_colour;
  
  /**
   * the value returned by the color sensor when it goes over a line
   */
  private static final int VALUE_OF_BLACK = 13;
  
  /**
   *  This method returns the colour value calculated by the colour sensor.
   *  We noticed that when the colour sensor went over the grid lines, it returned the value 13, so we 
   *  wrote code such that the colour sensor value of 13 represents the colour black.
   */
  public static void localize() {


    (new Thread() {
      public void run() {

        while(true) {

          colourSensor.fetchSample(colour, 0);
          value_of_colour = (int) colour[0];
          try {
            Thread.sleep(COLOUR_SENSOR_SLEEP);
          } catch (InterruptedException e) {

          }
        }

      }
    }).start();

  }     

 /**
 * This method assumes that the EV3 faces in the correct direction (angle of rotation  = 0 degrees from the 0 degree axis)
 * This method makes the Ev3 go straight until a line is detected, the EV3 then rotates 90 degrees clockwise, it then goes
 * straight until another black line detected, it then rotates anti-clockwise by 90 degrees so that the EV3 is in the 
 * position (1,1) and is facing straight such that the its angle from the 0 degrees axis is equal to 0 degrees.
 */
  public static void localize2() {
    (new Thread() {
      public void run() {

          while (!(LightLocalizer.get_value_of_colour() >=VALUE_OF_BLACK))
          {
          leftMotor.setSpeed(ROTATION_SPEED); 
          rightMotor.setSpeed(ROTATION_SPEED);
          leftMotor.forward();
          rightMotor.forward();
          }
          UltrasonicLocalizer.turnBy(NINETY_DEGREES);
          
          while (!(LightLocalizer.get_value_of_colour() >=VALUE_OF_BLACK))
          {
          leftMotor.setSpeed(ROTATION_SPEED); 
          rightMotor.setSpeed(ROTATION_SPEED);
          leftMotor.forward();
          rightMotor.forward();
          }
          UltrasonicLocalizer.turnBy(-NINETY_DEGREES);
         
      }
    }).start();

  }

/**
 * This method returns the colour value calculated by the colour sensor
 * @return value calculated by the colour sensor
 */
  public static int get_value_of_colour() {
    return value_of_colour;
    }


}