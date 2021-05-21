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