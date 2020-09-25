package ca.mcgill.ecse211.project;
import static ca.mcgill.ecse211.project.Resources.*;

public class LightLocalizer {


  private static float color[] = new float[colorSensor.sampleSize()];
  private static int value_of_colour;
  
  /**
   *  This method returns the colour value calculated by the colour sensor.
   *  We noticed that when the colour sensor went over the grid lines, it returned the value 13, so we 
   *  wrote code such that the colour sensor value of 13 represents the colour black.
   */
  public static void localize() {


    (new Thread() {
      public void run() {

        while(true) {

          colorSensor.fetchSample(color, 0);
          float black = color[0];
          value_of_colour = (int) black;
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

          while (!(LightLocalizer.get_value_of_colour() >=13))
          {
          leftMotor.setSpeed(100); 
          rightMotor.setSpeed(100);
          leftMotor.forward();
          rightMotor.forward();
          }
          UltrasonicLocalizer.turnBy(90.0);
          
          while (!(LightLocalizer.get_value_of_colour() >=13))
          {
          leftMotor.setSpeed(100); 
          rightMotor.setSpeed(100);
          leftMotor.forward();
          rightMotor.forward();
          }
          UltrasonicLocalizer.turnBy(-90.0);
         
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
