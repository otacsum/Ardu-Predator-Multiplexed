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