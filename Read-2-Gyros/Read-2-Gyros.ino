//Import Libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>

//Set the address for the multiplex board.
#define TCAADDR 0x70

/* Assign array instances to each of the gyro/mag sensors
   This creates two instances of each, despite the odd syntax */
Adafruit_9DOF                 dof[2]   = {Adafruit_9DOF()};
Adafruit_LSM303_Accel_Unified accel[2] = {Adafruit_LSM303_Accel_Unified(30301)};
Adafruit_LSM303_Mag_Unified   mag[2]   = {Adafruit_LSM303_Mag_Unified(30302)};

/**
 * @brief Select and enable the desired port on the multiplexer
 * 
 * @param i Index of the port (0-7)
 */
void tcaSelect(uint8_t i)
{
    if (i > 7) return;  //bounds checking, safety net. 

    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);  //Enable the port with this index by setting HIGH.
    Wire.endTransmission();
}

/**
 * @brief Writes sensor data to the serial monitor.
 * 
 * @param port Int port number, used for logging user-friendly data only.
 * @param dof Sensor reference var.
 * @param accel Sensor reference var.
 * @param mag Sensor reference var.
 */
void displaySensorData( int port,
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
        /* If true, 'orientation' should have valid .roll and .pitch fields */
        Serial.print(F("["));
        Serial.print(port);
        Serial.print(F("] "));

        Serial.print(F("Roll: "));
        Serial.print(orientation.roll);
        Serial.print(F("; "));

        Serial.print(F("Pitch: "));
        Serial.print(orientation.pitch);
        Serial.print(F("; "));
    }

    /* Calculate the heading using the magnetometer */
    mag.getEvent(&mag_event);
    if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
    {
        /* If true, 'orientation' should have valid .heading data */
        Serial.print(F("Heading: "));
        Serial.print(orientation.heading);
        Serial.print(F("; "));
    }

}

/**
 * @brief General hardware setup steps.
 */
void setup()
{
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
void loop()
{
    //Loop through each connected sensor.
    for (int i = 0; i < sizeof(dof); i++) {
        tcaSelect(i);  //Enable the port
        displaySensorData(i, dof[i], accel[i], mag[i]); //Read and display the data.
    }
    
    Serial.println(F("")); //Add a line break in the serial monitor console.
    delay(500);  //wait this many milliseconds before looping
}
