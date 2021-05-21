#include "libraries.h"
#include "configuration.h"
#include "declarations.hpp"
#include "serial-logging.hpp"

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
float getSensorData( int port,
                        Adafruit_9DOF dof,
                        Adafruit_LSM303_Accel_Unified accel,
                        Adafruit_LSM303_Mag_Unified   mag) {

    sensors_event_t accel_event;
    sensors_event_t mag_event;
    sensors_vec_t   orientation;

    float pitchAndHeadingArray[2];

    /* Calculate pitch and roll from the raw accelerometer data */
    accel.getEvent(&accel_event);
    if (dof.accelGetOrientation(&accel_event, &orientation))
    {
        float currentPitch = orientation.pitch;

        while(pitchQueue[port].count() < MOVINGAVGCOUNT) {  //start filling up the queue
            fillPitchQueue(port, orientation.pitch);  //Send the current reading from this loop iteration
        }
        
        float finalPitchVal = getFinalPitchVal(port, currentPitch);

        pitchAndHeadingArray[0] = finalPitchVal;
        
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

        pitchAndHeadingArray[1] = finalHeadingVal;
    }

    return pitchAndHeadingArray;

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
            /* There was a problem detecting the LSM303 magnetometer */
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
        float pitchAndHeadingArray = getSensorData(i, dof[i], accel[i], mag[i]); 
        pitch[i] = pitchAndHeadingArray[0]; 
        logPitchValue(i, pitch[i]);       
        heading[i] = pitchAndHeadingArray[1];
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
