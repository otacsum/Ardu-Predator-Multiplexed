//Import Libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include <QueueList.h>
#include <Servo.h>

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

// Assign array instances to each of the gyro/mag sensors & initialize arr elements */
Adafruit_9DOF                 dof[2]   = {Adafruit_9DOF()};  //DOF means Degrees of freedom, i.e. 9-axis
Adafruit_LSM303_Accel_Unified accel[2] = {Adafruit_LSM303_Accel_Unified(30301)};
Adafruit_LSM303_Mag_Unified   mag[2]   = {Adafruit_LSM303_Mag_Unified(30302)};

// Queues to store sensor readings for a moving-average debounce
QueueList <float> pitchQueue[2], headingQueue[2];

//To assist in calculating moving averages for debouncing pitch readings
float pitchSUM[2] = {0.0, 0.0};  
float headingSUM[2] = {0.0, 0.0};

//Store the startup values to normalize neutral positions.
float pitchOFFSET[2], headingOFFSET[2];

// Will be used for timing events.
unsigned long currentMillis = 0;    // stores the value of millis() in each iteration of loop()
unsigned long prevMillis = 0;    // stores the value of millis() in each iteration of loop() 

/**
 * @brief Writes Queue status to serial monitor during setup
 * @details Used for validation and troubleshooting to verify the
 *          queue is being filled with valid readings before calculating
 *          moving averages.
 * 
 * @param port Index of the current sensor port on the multiplexer
 * @param currentPitch Angle in degrees from the accel/gyro.
 * @param count Index of the queue which is being populated.
 */
void logQueueStatus(int port, float currentPitch, int count, String queue) {
    if (DEBUG) {
        String logMessage = "Filling ";
        logMessage.concat(queue);
        logMessage.concat(": [");
        logMessage.concat(port);
        logMessage.concat("][");
        logMessage.concat(count);
        logMessage.concat("] Value: ");
        logMessage.concat(currentPitch);
        Serial.println(logMessage);
    }
}

/**
 * @brief Writes current pitch reading to the serial monitor.
 * 
 * @param port Index of the current sensor port on the multiplexer
 * @param currentPitch Moving average of the last N pitch readings.
 */
void logPitchValue(int port, float currentPitch) {
    if (DEBUG) {
        String logMessage = "[";
        logMessage.concat(port);
        logMessage.concat("] Pitch: ");
        logMessage.concat(currentPitch);
        logMessage.concat("; ");
        Serial.print(logMessage);
    }
}

/**
 * @brief Writes current heading reading to the serial monitor.
 * 
 * @param currentHeading Moving average of the last N heading readings.
 */
void logHeadingValue(float currentHeading) {
    if (DEBUG) {
        String logMessage = "Heading: ";
        logMessage.concat(currentHeading);
        logMessage.concat("; ");
        Serial.print(logMessage);
    }
}

/**
 * @brief Writes current heading reading to the serial monitor.
 * 
 * @param currentHeading Moving average of the last N heading readings.
 */
void logPitchOffset(int port) {
    if (DEBUG) {
        String logMessage = "pitchOFFSET: [";
        logMessage.concat(port);
        logMessage.concat("] = ");
        logMessage.concat(pitchOFFSET[port]);
        logMessage.concat("; ");
        Serial.println(logMessage);
    }
}

/**
 * @brief Writes current heading reading to the serial monitor.
 * 
 * @param currentHeading Moving average of the last N heading readings.
 */
void logHeadingOffset(int port) {
    if (DEBUG) {
        String logMessage = "headingOFFSET: [";
        logMessage.concat(port);
        logMessage.concat("] = ");
        logMessage.concat(headingOFFSET[port]);
        logMessage.concat("; ");
        Serial.println(logMessage);
    }
}

/**
 * @brief Writes current servo positions to the serial monitor.
 * 
 * @param servoPositions pitch,heading array calculated from calcServoPositions.
 */
void logServoPositions(int* servoPositions) {
    if (DEBUG) {
        String logMessage = "Servo Positions | Pitch: [";
        logMessage.concat(servoPositions[0]);
        logMessage.concat("] Heading: [");
        logMessage.concat(servoPositions[1]);
        logMessage.concat("]");
        Serial.print(logMessage);
    }
}

/**
 * @brief Select and enable the desired port on the multiplexer
 * 
 * @param i Index of the port (0-7)
 */
void tcaSelect(uint8_t i) {
    if (i > 7) return;  //bounds checking, safety net. 

    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);  //Enable the port with this index by setting HIGH.
    Wire.endTransmission();
}

/**
 * @brief Writes sensor data to the serial monitor.
 * 
 * @param port Index of the current sensor port on the multiplexer
 * @param dof Sensor reference object.
 * @param accel Sensor reference object.
 * @param mag Sensor reference object.
 */
float* getSensorData( int port,
                        Adafruit_9DOF dof,
                        Adafruit_LSM303_Accel_Unified accel,
                        Adafruit_LSM303_Mag_Unified   mag) {

    sensors_event_t accel_event;
    sensors_event_t mag_event;
    sensors_vec_t   orientation;

    float pitchHeading[2];

    /* Calculate pitch and roll from the raw accelerometer data */
    accel.getEvent(&accel_event);
    if (dof.accelGetOrientation(&accel_event, &orientation))
    {
        float currentPitch = orientation.pitch;

        while(pitchQueue[port].count() < MOVINGAVGCOUNT) {  //start filling up the queue
            fillPitchQueue(port, orientation.pitch);  //Send the current reading from this loop iteration
        }
        
        float finalPitchVal = getFinalPitchVal(port, currentPitch);

        pitchHeading[0] = finalPitchVal;
        
    }

    /* Calculate the heading using the magnetometer */
    mag.getEvent(&mag_event);
    if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
    {
        float currentHeading = orientation.heading;

        while(headingQueue[port].count() < MOVINGAVGCOUNT) {  //start filling up the queue
            fillHeadingQueue(port, orientation.heading);  //Send the current reading from this loop iteration
        }
        
        float finalHeadingVal = getFinalHeadingVal(port, currentHeading);

        pitchHeading[1] = finalHeadingVal;
    }

    return pitchHeading;

}

/*void moveServos(int servoHeading, int servoPitch) {
  //Serial.println("moveServos");

  // TODO: Determine if yaw/pitch is beyond normal servo angles i.e. > 179 or < 0
  // May not be necessary after adding 2nd gyro and using relative angles.
  // If so, assume actual gyro position and provide max servo value.
  
  // Invert Yaw values for my servo orientation.
  // Comment out this line if your yaw is moving opposite the gyro.
  int invertedYaw = map(servoYaw, 0, 179, 179, 0);

  // Prevent the servos from panning farther than my head can rotate.
  // TODO: May not be necessary after adding 2nd gyro and using relative angles.
  if (invertedYaw > maxYaw) {
    invertedYaw = maxYaw;
  }
  else if (invertedYaw < minYaw) {
    invertedYaw = minYaw; 
  }

  // Prevent the servos from tilting farther than my head can pitch.
  // TODO: May not be necessary after adding 2nd gyro and using relative angles.
  if (servoPitch > maxPitch) {
    servoPitch = maxPitch;
  }
  else if (servoPitch < minPitch) {
    servoPitch = minPitch;  
  }

  // Print out the current yaw and pitch angle values being sent to the servos.
  Serial.print("\tServo - Yaw:"); Serial.print(servoYaw);
  Serial.print("\tServo - invertYaw:"); Serial.print(invertedYaw);
  Serial.print("\tPitch:"); Serial.println(servoPitch);

  // Move the servos to the current yaw/pitch values
  yawServo.write(invertedYaw);
  pitchServo.write(servoPitch);

  // Reset the timer for comparison on the next loop
  prevMillis = currentMillis;

}

/*****  Utility Functions *****/

/**
 * @brief Fills the moving average queue and calibrates the starting position offsets. Does not run after startup.
 * 
 * @param port Index of the current sensor port on the multiplexer
 * @param currentPitch Current sensor reading
 */
void fillPitchQueue(int port, float currentPitch) {
    //Push values into the queue and sum up the various values
    pitchQueue[port].push(currentPitch);
    pitchSUM[port] += currentPitch;

    //Load offset into the static value - does not change once the queue is full
    pitchOFFSET[port] = getMovingAverage(pitchSUM[port]);

    //Write to serial monitor
    logQueueStatus(port, currentPitch, pitchQueue[port].count(), "pitchQueue");
    if (pitchQueue[port].count() == MOVINGAVGCOUNT) logPitchOffset(port);
}

/**
 * @brief Fills the moving average queue and calibrates the starting position offsets. Does not run after startup.
 * 
 * @param port Index of the current sensor port on the multiplexer
 * @param currentPitch Current sensor reading
 */
void fillHeadingQueue(int port, float currentHeading) {
    //Push values into the queue and sum up the various values
    headingQueue[port].push(currentHeading);
    headingSUM[port] += currentHeading;

    //Load offset into the static value - does not change once the queue is full
    headingOFFSET[port] = getMovingAverage(headingSUM[port]);

    //Write to serial monitor
    logQueueStatus(port, currentHeading, headingQueue[port].count(), "headingQueue");
    if (headingQueue[port].count() == MOVINGAVGCOUNT) logHeadingOffset(port);
}

/**
 * @brief Calculate a stable pitch measurement
 * @details The current reading is enqueued, the oldest reading is popped, then the moving avg value calculated
 * 
 * @param port Index of the current sensor port on the multiplexer
 * @param currentPitch Current sensor reading
 * 
 * @return Moving average value of the pitch
 */
float getFinalPitchVal(int port, float currentPitch) {
    pitchSUM[port] += currentPitch; //Add the Nth value to the SUM
    pitchQueue[port].push(currentPitch); //add the newest to the queue
    pitchSUM[port] -= pitchQueue[port].pop(); //remove the oldest from the queue, and pitchSUM.
    //Subtract the startup-offset from the MA, then invert per the user pref.
    return ((getMovingAverage(pitchSUM[port]) - pitchOFFSET[port]) * float(invertPitch));   
}

/**
 * @brief Calculate a stable heading measurement
 * @details The current reading is enqueued, the oldest reading is popped, then the moving avg value calculated
 * 
 * @param port Index of the current sensor port on the multiplexer
 * @param currentHeading Current sensor reading
 * 
 * @return Moving average value of the heading
 */
float getFinalHeadingVal(int port, float currentHeading) {
    headingSUM[port] += currentHeading; //Add the Nth value to the SUM
    headingQueue[port].push(currentHeading); //add the newest to the queue
    headingSUM[port] -= headingQueue[port].pop(); //remove the oldest from the queue, and pitchSUM.
    //Subtract the startup-offset from the MA, then invert per the user pref.
    return ((getMovingAverage(headingSUM[port]) - headingOFFSET[port]) * float(invertHeading));   
}

/**
 * @brief Calculate the moving average by dividing the sum of all items in queue by the length of queue.
 */
float getMovingAverage(float pitchOrHeadingSum) {
    return (pitchOrHeadingSum / MOVINGAVGCOUNT);
}

int calcServoPositions(float* pitch, float* heading) {

    //Get the diff, round up or down while casting to integer
    int netPitch = int((pitch[0] - pitch[1]) + servoPitchOffset + 0.5);
    int netHeading = int((heading[0] - heading[1]) + servoHeadingOffset + 0.5);

    /*/Set max/min mechanical limits if sensor readings go out of range.
    if (netPitch > maxPitch) {
        netPitch = maxPitch;
    }
    else if (netPitch < minPitch) {
        netPitch = minPitch;
    }

    if (netHeading > maxHeading) {
        netHeading = maxHeading;
    }
    else if (netHeading < minHeading) {
        netHeading = minHeading;
    }*/

    int servoPositions[2] = {netPitch, netHeading};


    for (int i = 0; i < sizeof(pitch); i++) {    
        logPitchValue(i, pitch[i]);
        logHeadingValue(heading[i]);
    }

    logServoPositions(servoPositions);

    return servoPositions;

}


/**
 * @brief General hardware setup steps.
 */
void setup() {
    //Enable the wire and serial ports on the arduino.
    Wire.begin();
    Serial.begin(115200);

    //Print some text to inform the user what's going on.
    Serial.println("9DOF Sensor Tests:"); Serial.println("");

    /* Init each sensor, adjust the index range and/or incrementer function for your actual ports */
    for (int i = 0; i < sizeof(dof); i++) {

        Serial.println("Initializing port [" + String(i) + "]...");

        tcaSelect(i);  //Enable this port.
        if(!accel[i].begin())
        {
            /* There was a problem detecting the LSM303 accelerometer */
            Serial.println("Oops, no Accelerometer detected on port [" + String(i) + "] - Check your wiring!");
            //while(1); //stall the program, do not continue.
        }

        if(!mag[i].begin())
        {
            /* There was a problem detecting the LSM303 manetometer */
            Serial.println("Oops, no Magnetometer detected on port [" + String(i) + "] - Check your wiring!");
            //while(1); //stall the program, do not continue.
        }

    }

    delay(1000); //Wait 1 sec before starting.
    
}

/**
 * @brief Main loop, repeatedly read and display sensor data.
 */
void loop() {
    //Arrays to store the fixed and dynamic sensor readings to compare and send to the servos.
    //Index 0 = fixed sensor, 1 = dynamic sensor
    float pitch[2], heading[2];

    //Loop through each connected sensor.
    for (int i = 0; i < sizeof(dof); i++) {
        tcaSelect(i);  //Enable the port

        //Read, store, and (optionally if Debug is on) display the data.
        float* pitchHeading = getSensorData(i, dof[i], accel[i], mag[i]); 
        pitch[i] = pitchHeading[0];
        heading[i] = pitchHeading[1];
    }

    //Calculate the difference in angle between the two sensors.
    int* servoPositions = calcServoPositions(pitch, heading);
    
    // Capture the latest value of millis()
    currentMillis = millis();   

    //Enough time has passed since the last move command, send another move command
    if (currentMillis >= prevMillis + servoDelay) {
        //moveServos(servoPositions);
    }

    if (DEBUG) Serial.println(); //Whitespace for readability

    delay(250);  //wait this many milliseconds before looping
}
