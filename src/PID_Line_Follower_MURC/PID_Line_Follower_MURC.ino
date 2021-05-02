// begin initialisations
// IR sensors
int leftSensor = 13; // left-most sensor
int middleSensor = 11; // middle sensor
int rightSensor = 12; // right-most sensor

// initial values of sensors
int sensor[3] = { 0, 0, 0 };

// sensor states for comparison
int LINE_MIDDLE[] = { 0, 1, 0 };
int LINE_FULL[] = { 1, 1, 1 };
int LINE_LEFT_CLOSE[] = { 1, 1, 0 };
int LINE_LEFT_FAR[] = { 1, 0, 0 };
int LINE_RIGHT_CLOSE[] = { 0, 1, 1 };
int LINE_RIGHT_FAR[] = { 0, 0, 1 };
int LINE_NULL[] = { 0, 0, 0 };

// motor A and B variables
int motorL[] = { 4, 2, 3 };
int motorR[] = { 7, 8, 6 }; 

// variables for usage in directional functions
int motorInput1 = motorL[0];
int motorInput2 = motorL[1];
int motorInput3 = motorR[0];
int motorInput4 = motorR[1];

// ENA/ENB
int ENA = motorL[2]; // 3
int ENB = motorR[2]; // 6

// initial motor speed
int initialMotorSpeed = 80;

// PID constants
float Kp = 10;
float Ki = 0;
float Kd = 40;
float error = 0, P = 0, I = 0, D = 0, PIDval = 0;
float previousError = 0, previousI = 0;
int flag = 0;

// string that contains status between line follower and the line
String status = "ON_LINE";
// end initialisations


void setup() {
  pinMode(leftSensor, INPUT);
  pinMode(middleSensor, INPUT);
  pinMode(rightSensor, INPUT);
  
  pinMode(motorInput1, OUTPUT);
  pinMode(motorInput2, OUTPUT);
  pinMode(motorInput3, OUTPUT);
  pinMode(motorInput4, OUTPUT);
  
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  Serial.begin(9600); // set serial monitor to default baud rate of 9600
  delay(500);
  Serial.println("starting...");
  delay(1000);
}


void loop() {
  readSensorValues();

  if(status == "OFF_LINE"){
    stopMoving();
  }
  else if(status == "FULL_LINE") {
    if(previousError > 0) {
      error = 5;
      calculatePID();
      motorControl("SHARP_LEFT"); // (left wheel reverse, right wheel forward)
    }
    else if(previousError < 0) {
      error = -5;
      calculatePID();
      motorControl("SHARP_RIGHT"); // (right wheel reverse, left wheel forward)
    }
  }
  else { // if(status == "ON_LINE")
    calculatePID();
    motorControl("FORWARD");
  }
}


void readSensorValues() {
  sensor[0] = !digitalRead(leftSensor);
  sensor[1] = !digitalRead(middleSensor);
  sensor[2] = !digitalRead(rightSensor);

  //-- OFF - OFF - OFF --
  if(areArraysEqual(sensor, LINE_NULL, 3)) {
    status = "OFF_LINE";
  }
  
  // -- |ON - ON - ON| --   stop the line follower
  else if(areArraysEqual(sensor, LINE_FULL, 3)) {
    status = "FULL_LINE";
  }
  
  else {
    status = "ON_LINE";
    
    // -- OFF |ON| OFF -- 
    if(areArraysEqual(sensor, LINE_MIDDLE, 3)) {
      error = 0;
    }
    // -- |ON - ON| OFF --
    else if(areArraysEqual(sensor, LINE_LEFT_CLOSE, 3)) {
      error = 2;
    }
    // -- |ON| OFF - OFF --
    else if(areArraysEqual(sensor, LINE_LEFT_FAR, 3)) {
      error = 4;
    }
    // -- OFF |ON - ON| --
    else if(areArraysEqual(sensor, LINE_RIGHT_CLOSE, 3)) {
      error = -2;
    }
    // -- OFF - OFF |ON| --
    else if(areArraysEqual(sensor, LINE_RIGHT_FAR, 3)) {
      error = -4;
    }
  }
}


void calculatePID() {
  P = error;
  I = I + previousI;
  D = error - previousError;
  PIDval = (Kp * P) + (Ki * I) + (Kd * D);
  previousI = I;
  previousError = error;
}


void motorControl(String DIRECTION) {
  // calculate effective motor speed
  int leftMotorSpeed = initialMotorSpeed - PIDval;
  int rightMotorSpeed = initialMotorSpeed + PIDval;
  
  // motor speed should not exceed max PWM val
  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

  analogWrite(ENA, leftMotorSpeed); // left motor speed
  analogWrite(ENB, rightMotorSpeed); // right motor speed

  if(DIRECTION == "FORWARD") {
    forward(); // to make the bot move forward
  }
  else if(DIRECTION == "SHARP_LEFT") {
    sharpLeft(); // to make the line follower sharp turn left
  }
  else if(DIRECTION == "SHARP_RIGHT") {
    sharpRight(); // to make the line follower sharp turn right
  }
}


// directional functions
// function defining direction of wheel rotation to be forward
void forward() {
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, HIGH);
}

// function defining direction of wheel rotation to be sharp left (left wheel reverse, right wheel forward)
void sharpLeft() {
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, HIGH);
}

// function defining direction of wheel rotation to be sharp left (right wheel reverse, left wheel forward)
void sharpRight() {
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
}

// function stopping bot
void stopMoving() {
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, LOW);
}

// function to compare arrays
boolean areArraysEqual(int arrayA[], int arrayB[], int numItems) {
  boolean same = true;

  for(int i = 1; i <= numItems; i++) {
    if(arrayA[i] != arrayB[i]) {
      same = false;
    }
  }

  return same;
}
