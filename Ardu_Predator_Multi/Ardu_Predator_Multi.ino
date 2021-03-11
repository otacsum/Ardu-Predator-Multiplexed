//Import Libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include <QueueList.h>

/** 
 * User Config Parameters
 */
#define TCAADDR 0x70  //Set the address for the multiplex board.
#define MOVINGAVGCOUNT 20 //How many readings to calculate for a moving-average

bool DEBUG = true; //Enable for serial monitor logging.

//Flips sensor readings depending on normal position.
int invertPitch = 1;
int invertHeading = 1;
/** 
 * End User Config Parameters
 */

// Assign array instances to each of the gyro/mag sensors & initialize arr elements */
Adafruit_9DOF                 dof[2]   = {Adafruit_9DOF()};
Adafruit_LSM303_Accel_Unified accel[2] = {Adafruit_LSM303_Accel_Unified(30301)};
Adafruit_LSM303_Mag_Unified   mag[2]   = {Adafruit_LSM303_Mag_Unified(30302)};

// Queues to store sensor readings for a moving-average debounce
QueueList <float> pitchQueue[2], headingQueue[2];

//To assist in calculating moving averages for debouncing pitch readings
float pitchSUM[2] = {0.0, 0.0};  
float headingSUM[2] = {0.0, 0.0};

//Store the startup values to normalize neutral positions.
float pitchOFFSET[2], headingOFFSET[2]; 

/**
 * @brief Writes Queue status to serial monitor during setup
 * @details Used for validation and troubleshooting to verify the
 *          queue is being filled with valid readings before calculating
 *          moving averages.
 * 
 * @param port Index of the multiplex port being initialized.
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
 * @param port Index of the multiplex port being read.
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
        logMessage.concat(";");
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
 * @param port Int port number, switches queues and MAs
 * @param dof Sensor reference object.
 * @param accel Sensor reference object.
 * @param mag Sensor reference object.
 */
void getSensorData( int port,
                        Adafruit_9DOF dof,
                        Adafruit_LSM303_Accel_Unified accel,
                        Adafruit_LSM303_Mag_Unified   mag)
{
    sensors_event_t accel_event;
    sensors_event_t mag_event;
    sensors_vec_t   orientation;

    /* Calculate pitch and roll from the raw accelerometer data */
    accel.getEvent(&accel_event);
    if (dof.accelGetOrientation(&accel_event, &orientation))
    {
        float currentPitch = orientation.pitch;

        while(pitchQueue[port].count() < MOVINGAVGCOUNT) {  //start filling up the queue
            fillPitchQueue(port, orientation.pitch);  //Send the current reading from this loop iteration
        }
        
        float finalPitchVal = getFinalPitchVal(port, currentPitch);

        logPitchValue(port, finalPitchVal);
        
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

        logHeadingValue(finalHeadingVal);
    }

}

//Only runs until the queue is full during the first N loops.
void fillPitchQueue(int port, float currentPitch) {
    //Push values into the queue and sum up the various values
    pitchQueue[port].push(currentPitch);
    pitchSUM[port] += currentPitch;

    //Load offset into the static value - does not change once the queue is full
    pitchOFFSET[port] = getMovingAverage(pitchSUM[port]);

    //Write to serial monitor
    logQueueStatus(port, currentPitch, pitchQueue[port].count(), "pitchQueue");
}

//Only runs until the queue is full during the first N loops.
void fillHeadingQueue(int port, float currentHeading) {
    //Push values into the queue and sum up the various values
    headingQueue[port].push(currentHeading);
    headingSUM[port] += currentHeading;

    //Load offset into the static value - does not change once the queue is full
    headingOFFSET[port] = getMovingAverage(headingSUM[port]);

    //Write to serial monitor
    logQueueStatus(port, currentHeading, headingQueue[port].count(), "headingQueue");
}

float getFinalPitchVal(int port, float currentPitch) {
    pitchSUM[port] += currentPitch; //Add the Nth value to the SUM
    pitchQueue[port].push(currentPitch); //add the newest to the queue
    pitchSUM[port] -= pitchQueue[port].pop(); //remove the oldest from the queue, and pitchSUM.
    //Subtract the startup-offset from the MA, then invert per the user pref.
    return ((getMovingAverage(pitchSUM[port]) - pitchOFFSET[port]) * invertPitch);   
}

float getFinalHeadingVal(int port, float currentHeading) {
    headingSUM[port] += currentHeading; //Add the Nth value to the SUM
    headingQueue[port].push(currentHeading); //add the newest to the queue
    headingSUM[port] -= headingQueue[port].pop(); //remove the oldest from the queue, and pitchSUM.
    //Subtract the startup-offset from the MA, then invert per the user pref.
    return ((getMovingAverage(headingSUM[port]) - headingOFFSET[port]) * invertHeading);   
}

float getMovingAverage(float pitchOrHeadingSum) {
    return round((pitchOrHeadingSum / MOVINGAVGCOUNT));
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
    
}

/**
 * @brief Main loop, repeatedly read and display sensor data.
 */
void loop() {
    //Loop through each connected sensor.
    for (int i = 0; i < sizeof(dof); i++) {
        tcaSelect(i);  //Enable the port
        getSensorData(i, dof[i], accel[i], mag[i]); //Read and display the data.
    }
    Serial.println();
    delay(5);  //wait this many milliseconds before looping
}
