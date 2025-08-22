//#include <NewPing.h>
#include <Servo.h>
#include <AFMotor.h>

//Motor Pin
#define A_EN 3
#define B_EN 11
#define A_IN1 4
#define A_IN2 5
#define B_IN3 6
#define B_IN4 7


//hc-sr04 sensor
// #define TRIGGER_PIN 8
// #define ECHO_PIN 9
#define ECHO_PIN A3
#define TRIGGER_PIN A4
#define max_distance 50

//ir sensor
#define irRight A0
#define irLeft A1

//motor
#define SPEED 150

Servo servo;

//NewPing sonar(TRIGGER_PIN, ECHO_PIN, max_distance);


int distance = 0;
int leftDistance;
int rightDistance;
boolean object;

void setup() {
  Serial.begin(9600);
  pinMode(irLeft, INPUT);
  pinMode(irRight, INPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  servo.attach(10);
  servo.write(90);

  analogWrite(A_EN, SPEED);
  analogWrite(B_EN, SPEED);
}

void loop() {
  if (digitalRead(irLeft) == 0 && digitalRead(irRight) == 0 ) {
    objectAvoid();
    //forword
  }
  else if (digitalRead(irLeft) == 0 && digitalRead(irRight) == 1 ) {
    objectAvoid();
    //leftturn
    moveRight();
  }
  else if (digitalRead(irLeft) == 1 && digitalRead(irRight) == 0 ) {
    objectAvoid();
    //rightturn
    moveLeft();
  }
  else if (digitalRead(irLeft) == 1 && digitalRead(irRight) == 1 ) {
    //Stop
    Stop();
  }
}

void objectAvoid() {
  distance = getDistance();
  if (distance <= 15) {
    digitalWrite(12, 1);
    //stop
    Stop();

    lookLeft();
    lookRight();
    delay(100);
    if (rightDistance <= leftDistance) {
      //left
      object = true;
      turn();
    } else {
      //right
      object = false;
      turn();
    }
    delay(100);
  }
  else {
    //forword
    digitalWrite(12, 0);
    moveForward();
  }
}

// int getDistance() {
//   delay(50);
//   int cm = sonar.ping_cm();
//   if (cm == 0) {
//     cm = 100;
//   }
//   return cm;
// }
float getDistance()
  {
    float duration = 0.00;             // Float type variable declaration 
    float CM = 0.00;      
    digitalWrite(TRIGGER_PIN, LOW);        // Trig_pin output as OV (Logic Low-Level) 
    delayMicroseconds(2);              // Delay for 2 us    
    //Send 10us High Pulse to Ultra-Sonic Sonar Sensor "trigPin" 
    digitalWrite(TRIGGER_PIN, HIGH);       // Trig_pin output as 5V (Logic High-Level)
    delayMicroseconds(10);             // Delay for 10 us     
    digitalWrite(TRIGGER_PIN, LOW);        // Trig_pin output as OV (Logic Low-Level)    
    duration = pulseIn(ECHO_PIN, HIGH); // Start counting time, upto again "echoPin" back to Logical "High-Level" and puting the "time" into a variable called "duration" 
    CM = (duration / 58.82); //Convert distance into CM.     
    if(CM == 0) {
      CM = 100;
    }
    return CM;
  }

int lookLeft () {
  //lock left
  servo.write(150);
  delay(500);
  leftDistance = getDistance();
  delay(100);
  servo.write(90);
  return leftDistance;
  delay(100);
}

int lookRight() {
  //lock right
  servo.write(30);
  delay(500);
  rightDistance = getDistance();
  delay(100);
  servo.write(90);
  return rightDistance;
  delay(100);
}
void Stop() {
  digitalWrite(A_IN1, 0);
  digitalWrite(A_IN2, 0);
  digitalWrite(B_IN3, 0);
  digitalWrite(B_IN4, 0);
}
void moveForward() {
  digitalWrite(A_IN1, 1);
  digitalWrite(A_IN2, 0);
  digitalWrite(B_IN3, 1);
  digitalWrite(B_IN4, 0);
}
void moveBackward() {
  digitalWrite(A_IN1, 0);
  digitalWrite(A_IN2, 1);
  digitalWrite(B_IN3, 0);
  digitalWrite(B_IN4, 1);

}
void turn() {
  if (object == false) {
    moveLeft();
    delay(700);
    moveForward();
    delay(800);
    moveRight();
    delay(900);
    if (digitalRead(irRight) == 1) {
      loop();
    } else {
      moveForward();
    }
  }
  else {
    moveRight();
    delay(700);
    moveForward();
    delay(800);
    moveLeft();
    delay(900);
    if (digitalRead(irLeft) == 1) {
      loop();
    } else {
      moveForward();
    }
  }
}
void moveLeft() {
  digitalWrite(A_IN1, 0);
  digitalWrite(A_IN2, 1);
  digitalWrite(B_IN3, 1);
  digitalWrite(B_IN4, 0);
}
void moveRight() {
  digitalWrite(A_IN1, 1);
  digitalWrite(A_IN2, 0);
  digitalWrite(B_IN3, 0);
  digitalWrite(B_IN4, 1);
}