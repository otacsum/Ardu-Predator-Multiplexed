/** 
 * User Config Parameters
 */
#define TCAADDR 0x70  //Set the address for the multiplex board.
#define MOVINGAVGCOUNT 20 //How many readings to calculate for a moving-average

const bool DEBUG = true; //Enable for serial monitor logging.

//Flips sensor readings depending on normal position.  (1, -1)
const int invertPitch = 1;
const int invertHeading = 1;

//LED will give us status of the loop, if it turns off something failed.
const int LED_PIN = 13;

// Prevent the servos from rotating beyond mechanical limits 
const int maxPitch = 135;
const int minPitch = 45;
// NOTE: Yaw direction is inverted via map() method in moveServos()
const int maxHeading = 135;
const int minHeading = 45;

//used to center servos when gyros are level.  
//e.g. 0 degrees becomes 90 degrees, the middle of the servo's swing.
//Adjust this to compensate for your actual mounting position
const int servoPitchOffset = 90;
const int servoHeadingOffset = 90;

//millis between servo position write events.
const int servoDelay = 1000;
/** 
 * End User Config Parameters
 */