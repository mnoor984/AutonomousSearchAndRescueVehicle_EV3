package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import static ca.mcgill.ecse211.project.Main.sleepFor;

public class Navigation implements Runnable{
  /**
   * the x displacement of our desired destination in feet
   */
  int xDest_FT;
  /**
   * the y displacement of our desired destination in feet  
   */
  int yDest_FT;  
  /**
   * the current x displacement of the EV3 in cm
   */
  double xCurrent_CM;
  /**
   * the current y displacement of the EV3 in cm
   */
  double yCurrent_CM;
  /**
   * the current clockwise angle of the EV3 from the 0 axis
   */
  double thetaCurrent_RAD; //angle from vertical CW
  /**
   *  the displacement change required in the x-axis to reach our desired destination
   */
  double displX;
  /**
   *  the displacement change required in the y-axis to reach our desired destination
   */
  double displY;
  /**
   * the angle from the 0 axis that we need to rotate to in order to go in the direction of our desired destination
   */
  double displTheta_RAD;
  /**
   * the distance in cm we need to travel to reach our desired destination
   */
  double distance_needed_to_cover;  
 
  public void run( ) {
    odometer.setXyt(30.48, 30.48, 0);
   travelTo(1,3);
   sleepFor(NAVIGATION_SLEEP);
   travelTo(2,2);
   sleepFor(NAVIGATION_SLEEP);
   travelTo(3,3);
   sleepFor(NAVIGATION_SLEEP);
   travelTo(3,2);
   sleepFor(NAVIGATION_SLEEP);
   travelTo(2,1);
  }
  
//----------------------------------------------------------------------------------------------------
  /**
   * This method takes as input the angle we wish to turn to, it then turns the EV3 to our desired angle
   * @param angle_RAD
   */
  public void turnTo(double angle_RAD) {           //has to turn by minimal angle
    
   
    double deltaT = angle_RAD*(180/Math.PI) -  thetaCurrent_RAD*(180/Math.PI);
    
    if (deltaT >= 0 && deltaT <= 180 ) {
      UltrasonicLocalizer.turnBy(deltaT);          
    }
    else if (deltaT > 180 ) {
      UltrasonicLocalizer.turnBy(deltaT -360 );
    }
    else if (deltaT < 0 && deltaT > -180 ) {
      UltrasonicLocalizer.turnBy(deltaT);
    }
    else if (deltaT < 0 && deltaT < -180 ) {
      UltrasonicLocalizer.turnBy(deltaT + 360);
    }

       
  } //end of turnTo method
//----------------------------------------------------------------------------------------------------  
  
  /**
   * This method takes in the (x,y) coordinates of where we want to go, it then causes the EV3 to rotate and move to that specific
   * coordinate
   * @param x
   * @param y
   */
  public void travelTo(int x,int y) {   
    xDest_FT= x; //the x position (in feet) we want to reach
    yDest_FT= y; // the y position (in feet) we want to reach
    //get current position
    xCurrent_CM=odometer.getXyt()[0];   // our current x position in cm
    yCurrent_CM=odometer.getXyt()[1];   // our current y position in cm
    thetaCurrent_RAD=odometer.getXyt()[2] * RADS_PER_1DEG ;  // our current angle from the 0 degree axis
    
    displX= xDest_FT*TILE_SIZE_cm - xCurrent_CM;    //displX = the distance we need to travel in the x axis to reach where we want
    displY= yDest_FT*TILE_SIZE_cm - yCurrent_CM;    // displY = the distance we need to travel in the y axis to reach where we want
    
     if (displX != 0 && displY != 0)  {            // if we do not want to stay in the same position then..
    
       //1st quadrant 
        if (displX>0 && displY>0) {                                 
          displTheta_RAD=PI/2.0 - Math.atan(displY/displX);
          distance_needed_to_cover =  Math.sqrt((displX*displX) + (displY*displY));
          turnTo(displTheta_RAD);
          moveStraightFor(distance_needed_to_cover);
        }
        //2nd quadrant
        else if (displX<0 && displY>0)                             
        {
          displTheta_RAD=1.5*PI + Math.atan(Math.abs(displY/displX)); // pi + (pi/2+angle) 
          distance_needed_to_cover =  Math.sqrt((displX*displX) + (displY*displY));
          turnTo(displTheta_RAD);
          moveStraightFor(distance_needed_to_cover);
        }
        //3nd quadrant
        else if (displX<0 && displY<0)                             
        {
          displTheta_RAD =1.5*PI-Math.atan(Math.abs(displY/displX));  // pi + (pi/2-angle) 
          distance_needed_to_cover =  Math.sqrt((displX*displX) + (displY*displY));
          turnTo(displTheta_RAD);
          moveStraightFor(distance_needed_to_cover);
        }
        //4th quadrant 
        else                                                        
        {  
          displTheta_RAD=0.5*PI+ Math.atan(Math.abs(displY/displX));
          distance_needed_to_cover =  Math.sqrt((displX*displX) + (displY*displY));// (pi/2) + angle 
          turnTo(displTheta_RAD);
          moveStraightFor(distance_needed_to_cover);
        }
        
     } //end of if statement
     
     //vertical displacement 
    else if (displX==0)                                 
    {
        if     (displY>=0) displTheta_RAD=0;  //displacement forward
        else if(displY<0)  displTheta_RAD=PI; //displacement backward
        distance_needed_to_cover =  displY;
        turnTo(displTheta_RAD);
        moveStraightFor(distance_needed_to_cover);
    }
     //horizontal displacement
    else if (displY==0)                     
    {
      if     (displX>0)   displTheta_RAD=PI/2.0; //displacement to the right
      else if(displX<0)   displTheta_RAD=1.5*PI; //displacement to the left 
      distance_needed_to_cover =  displX;
      turnTo(displTheta_RAD);
      moveStraightFor(distance_needed_to_cover);
    }
     
  } //end of travelTo method
  
  /**
   * Moves the robot straight for the given distance.
   * 
   * @param distance in feet (tile sizes), may be negative
   */
  public static void moveStraightFor(double distance) {
    leftMotor.setSpeed(ROTATION_SPEED);
    rightMotor.setSpeed(ROTATION_SPEED);
    leftMotor.rotate(UltrasonicLocalizer.convertDistance(distance), true);
    rightMotor.rotate(UltrasonicLocalizer.convertDistance(distance), false);
  }
  
  

} //end of Navigation class
