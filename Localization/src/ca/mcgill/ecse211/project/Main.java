package ca.mcgill.ecse211.project;

//static import to avoid duplicating variables and make the code easier to read
import static ca.mcgill.ecse211.project.Resources.*;
import lejos.hardware.Button;

/**
 * The main driver class for the lab.
 */
public class Main {


  public static UltrasonicLocalizer US_Localizer_object;     
  
  public static String US_Localizer_type;
  
  /**
   * The main entry point.
   * 
   * @param args not used
   * @throws InterruptedException because we call Thread.sleep()
   */
  public static void main(String[] args) {
//    int abc;
//    abc = Button.waitForAnyPress();
//    if (abc == Button.ID_UP) {
//    LightLocalizer.localize();
//    LightLocalizer.localize2();
//    }
//    while (true) {
//    int status = Button.waitForAnyPress();
//    if (status == Button.ID_ESCAPE) {
//      System.exit(0);
//   }
//    }
    
      int Localization_choice;
      //start odometer thread
      new Thread(odometer).start(); 
     
      Localization_choice = chooseLocalizationMethod();
     
      if (Localization_choice == Button.ID_LEFT) {
        US_Localizer_type = "Falling_edge";
       
      } else if (Localization_choice == Button.ID_RIGHT) {
        US_Localizer_type = "Rising_edge";
      }
     
           
      // Start  threads
       US_Localizer_object = new UltrasonicLocalizer(US_Localizer_type);
       new Thread(US_Localizer_object).start(); 
    
       
       if( US_Localizer_type == "Rising_edge") {
        US_Localizer_object.Rising_Edge();
      }
       else if( US_Localizer_type == "Falling_edge") {
         US_Localizer_object.Falling_Edge();
       }
       //start display thread
       new Thread(new Display()).start();
       
       
       
       while (true) {
          int status = Button.waitForAnyPress();
          if (status == Button.ID_ESCAPE) {
            System.exit(0);
          }
          
          else   if (status == Button.ID_UP) {
            LightLocalizer.localize();
            LightLocalizer.localize2();
          }
        } //end of while loop
    }
 
 
  /**
   * Sleeps for the specified duration.
   * @param millis the duration in milliseconds
   */
  public static void sleepFor(long millis) {           
    try {
      Thread.sleep(millis);
    } catch (InterruptedException e) {
      // Nothing to do here
    }
  }
  
  
  private static int chooseLocalizationMethod() {
    int buttonChoice;
    Display.showText("< Left  | Right >","        |        ","Falling | Rising  "," Edge   |  Edge ");

    do {
      buttonChoice = Button.waitForAnyPress(); // left or right press
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT && buttonChoice != Button.ID_UP);
    return buttonChoice;
  }
  
}       //end of class 
