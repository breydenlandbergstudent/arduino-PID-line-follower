// where can we add useful Serial.println () statements...
// begin initialisations

// values below may need to be changed but structure is correct
// IR sensors
int leftSensor = 13, // left-most sensor
int middleSensor = 11, // middle sensor
int rightSensor = 12 // right-most sensor

// initial values of sensors
int sensor[3] = { 0, 0, 0 };

// motor A and B variables
int motorL[] = { 4, 2, 3 };
int motorR[] = { 7, 8, 9 };

// variables for usage in directional functions
int motorInput1 = motorL[0];
int motorInput2 = motorL[1];
int motorInput3 = motorR[0];
int motorInput4 = motorR[1];

// ENA/ENB
// may need to change
int ENA = 6;
int ENB = 11;

// initial motor speed
// may need to change
int initialMotorSpeed = 140;

// output pins for led
// may need to change
int ledPin1 = A3;
int ledPin2 = A4;

// PID constants
// may need to change
// what does each one of these do?
float Kp = 25;
float Ki = 0;
float Kd = 15;

float error = 0, P = 0, I = 0, D = 0, PIDval = 0;
float previousError = 0, previousI = 0;

int flag = 0;

// end initialisations


// may need to change
void setup () {
  pinMode (leftSensor, INPUT);
  pinMode (middleSensor, INPUT);
  pinMode (rightSensor, INPUT);

  pinMode (motorInput1, OUTPUT);
  pinMode (motorInput2, OUTPUT);
  pinMode (motorInput3, OUTPUT);
  pinMode (motorInput4, OUTPUT);
  // may need to change
  pinMode (ENA, OUTPUT);
  pinMode (ENB, OUTPUT);

  // may need to change
  pinMode (ledPin1, OUTPUT);
  pinMode (ledPin2, OUTPUT);
  digitalWrite (ledPin1, LOW);
  digitalWrite (ledPin2, LOW);

  Serial.begin (9600); // set serial monitor to default baud rate of 9600
  delay (500);
  Serial.println ("starting...");
  delay (1000);
}


// may need to change (a lot of it)
void loop () {
  readSensorValues ();
  Serial.println (error);

  // make left turn until straight path detected
  if (error == 100) {
    Serial.println ("left");
    do {
      readSensorValues ();
      analogWrite (ENA, 110);     // set left motor speed
      analogWrite (ENB, 90);      // set right motor speed
      sharpLeftTurn ();
    }
    while (error != 0);
  }

  // make right turn until straight path detected (only if right path detected - it will go forward in case of straight and right)
  else if (error = 101) {
    Serial.println ("right");
    analogWrite (ENA, 110);     // set left motor speed
    analogWrite (ENB, 90);      // set right motor speed
    forward ();
    delay (200);
    stopMoving ();
    readSensorValues ();

    if (error = 102) {
      do {
        analogWrite (ENA, 110);     // set left motor speed
        analogWrite (ENB, 90);      // set right motor speed
        sharpRightTurn ();
        readSensorValues ();
    }
    while (error != 0);
    }
  }

  // make left turn until straight path detected
  else if (error == 102) {
    // TODO
  }
}


// may need to change
void readSensorValues () {
  sensor[0] = !digitalRead (leftSensor);
  sensor[1] = !digitalRead (middleSensor);
  sensor[2] = !digitalRead (rightSensor);

  // may need to change
  // what does each one of these do?
  if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0))
    error = 3;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0))
    error = 2;
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 0))
    error = 1;
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1))
    error = 0;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1))
    error = -1;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1))
    error = -2;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0))
    error = -3;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1)) // turn left
    error = 100;
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1)) // turn right
    error = 101;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0)) // make U turn
    error = 102;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1)) // turn left or stop
    error = 103;
}


// may need to change
// how does this work?
void calculatePID () {
  P = error;
  I = I + previousI;
  D = error - previousError;

  PIDval = (Kp * P) + (Ki * I) + (Kd * D);

  previousI = I;
  previousError = error;
}


// may need to change
void motorControl () {
  // calculate effective motor speed
  int leftMotorSpeed = initialMotorSpeed - PIDval;
  int rightMotorSpeed = initialMotorSpeed + PIDval;

  // motor speed should not exceed max PWM val
  leftMotorSpeed = constrain (leftMotorSpeed, 0, 255);
  rightMotorSpeed = constrain (leftMotorSpeed, 0, 255);

  // may need to change
  analogWrite (ENA, left_motor_speed); // left motor speed
  analogWrite (ENB, right_motor_speed - 30); // right motor speed

  // to make the bot move forward
  forward ();
}


// directional functions
// function defining direction of wheel rotation to be forward
void forward () {
  digitalWrite (motorInput1, LOW);
  digitalWrite (motorInput2, HIGH);
  digitalWrite (motorInput3, LOW);
  digitalWrite (motorInput4, HIGH);
}

// function defining direction of wheel rotation to be reverse
void reverse () {
  digitalWrite (motorInput1, HIGH);
  digitalWrite (motorInput2, LOW);
  digitalWrite (motorInput3, HIGH);
  digitalWrite (motorInput4, LOW);
}

// function defining direction of wheel rotation to be right
void right () {
  digitalWrite (motorInput1, LOW);
  digitalWrite (motorInput2, HIGH);
  digitalWrite (motorInput3, LOW);
  digitalWrite (motorInput4, LOW);
}

// function defining direction of wheel rotation to be left
void left () {
  digitalWrite (motorInput1, LOW);
  digitalWrite (motorInput2, LOW);
  digitalWrite (motorInput3, LOW);
  digitalWrite (motorInput4, HIGH);
}

// function defining direction of wheel rotation to be sharp right
void sharpRight () {
  digitalWrite (motorInput1, LOW);
  digitalWrite (motorInput2, HIGH);
  digitalWrite (motorInput3, HIGH);
  digitalWrite (motorInput4, LOW);
}

// function defining direction of wheel rotation to be sharp left
void sharpLeft () {
  digitalWrite (motorInput1, HIGH);
  digitalWrite (motorInput2, LOW);
  digitalWrite (motorInput3, LOW);
  digitalWrite (motorInput4, HIGH);
}

// function stopping bot
void stopMoving () {
  digitalWrite (motorInput1, LOW);
  digitalWrite (motorInput2, LOW);
  digitalWrite (motorInput3, LOW);
  digitalWrite (motorInput4, LOW);
}
