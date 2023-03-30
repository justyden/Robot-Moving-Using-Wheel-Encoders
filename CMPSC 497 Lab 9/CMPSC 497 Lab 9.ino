// Program that allows the robot to move a fixed distance using wheel encoders.

// Include the TimerOne Library from Paul Stoffregen
#include "TimerOne.h"
 
// Constants for Interrupt Pins
// Change values if not using Arduino Uno
 
const byte MOTOR1 = 2;  // Motor 1 Interrupt Pin - INT 0 (Encoder)
const byte MOTOR2 = 3;  // Motor 2 Interrupt Pin - INT 1 (Encoder)
 
// Integers for pulse counters
unsigned int counter1 = 0;
unsigned int counter2 = 0;
 
// Float for number of slots in encoder disk
float diskslots = 20;  // Change to match value of encoder disk

#define speedPinR 5   // RIGHT PWM pin connect MODEL-X ENA
#define RightDirectPin1  12    //  Right Motor direction pin 1 to MODEL-X IN1 
#define RightDirectPin2  11    // Right Motor direction pin 2 to MODEL-X IN2
#define speedPinL 6        //  Left PWM pin connect MODEL-X ENB
#define LeftDirectPin1  7    // Left Motor direction pin 1 to MODEL-X IN3
#define LeftDirectPin2  8   ///Left Motor direction pin 1 to MODEL-X IN4
#define LPT 4 // scan loop counter

#define SERVO_PIN     9  //servo connect to D9

#define Echo_PIN    2 // Ultrasonic Echo pin connect to D11
#define Trig_PIN    10  // Ultrasonic Trig pin connect to D12

#define BUZZ_PIN     13
#define FAST_SPEED  250     //both sides of the motor speed
#define SPEED  120     //both sides of the motor speed
#define TURN_SPEED  200     //both sides of the motor speed
#define BACK_SPEED1  255     //back speed
#define BACK_SPEED2  90     //back speed

/*motor control*/
void go_Advance(void)  //Forward
{
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,HIGH);
  digitalWrite(LeftDirectPin2,LOW);
  set_MotorSpeed(150, 150);
}
void go_Left()  //Turn left
{
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,HIGH);
  set_MotorSpeed(160, 160);
}
void go_Right()  //Turn right
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,HIGH);
  digitalWrite(LeftDirectPin1,HIGH);
  digitalWrite(LeftDirectPin2,LOW);
  set_MotorSpeed(150, 150);
}
void go_Back()  //Reverse
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,HIGH);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,HIGH);
  set_MotorSpeed(160, 160);
}
void stop_Stop()    //Stop
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,LOW);
  set_MotorSpeed(0,0);
}

/*set motor speed */
void set_MotorSpeed(int speed_L,int speed_R)
{
  analogWrite(speedPinL,speed_L); 
  analogWrite(speedPinR,speed_R);   
}

void buzz_ON()   //open buzzer
{
  
  for(int i=0;i<100;i++)
  {
   digitalWrite(BUZZ_PIN,LOW);
   delay(2);//wait for 1ms
   digitalWrite(BUZZ_PIN,HIGH);
   delay(2);//wait for 1ms
  }
}
void buzz_OFF()  //close buzzer
{
  digitalWrite(BUZZ_PIN, HIGH);
  
}
void alarm(){
   buzz_ON();
 
   buzz_OFF();
}


// Interrupt Service Routines
 
// Motor 1 pulse count ISR
void ISR_count1()  
{
  counter1++;  // increment Motor 1 counter value
} 
 
// Motor 2 pulse count ISR
void ISR_count2()  
{
  counter2++;  // increment Motor 2 counter value
} 
 
// TimerOne ISR
void ISR_timerone()
{
  Timer1.detachInterrupt();  // Stop the timer
  Serial.print("Motor Speed 1: "); 
  float rotation1 = (counter1 / diskslots) * 60.00; 
 // calculate RPM for Motor 1
  
  Serial.print(rotation1);  
  Serial.print(" RPM - "); 
  counter1 = 0;  //  reset counter to zero
  Serial.print("Motor Speed 2: "); 
  float rotation2 = (counter2 / diskslots) * 60.00;  
// calculate RPM for Motor 2
  
  Serial.print(rotation2);  
  Serial.println(" RPM"); 
  counter2 = 0;  //  reset counter to zero
  Timer1.attachInterrupt( ISR_timerone );  // Enable the timer
}

// This function checks how many times the wheel has rotated and compares it to the value passed.
// A boolean value is returned to determine if that many rotations happened.
boolean checkDistance(float wheelRotation) {
    // This checks to see if the rotations happens.
    if (float((counter1 / diskslots)) >= wheelRotation || float((counter2 / diskslots)) >= wheelRotation) {
      counter1 = 0; // Reset counter 1 so it can be called again.
      counter2 = 0; // Reset counter 2 so it can be called again.
      return true;
    }
    else {
      return false;
    }
}

// Turns the robot the amount of rotations.
void turnRobot(float turnRotation) {
  // Checks to see if that amount of rotations has happened.
  // If it has then the function call will stop.
  while (!checkDistance(turnRotation)) {
    go_Right();
  }  
}

// This is the function that allows the square to happen.
// It calls other functions to allow it to work properly.
void makeSquare(float wheelRotation) {
    int path = 0; // This value is needed to determine if the square is complete.
    while (path <= 3) { // Checks to see if the square is complete.
      if (checkDistance(wheelRotation)) { // Turn the robot.
        stop_Stop();
        delay(100);
        turnRobot(1);
        stop_Stop();
        delay(100);
        path++; // After a turn this is increased.
      }
      else {
        go_Advance(); // Makes the robot go forward.
      }
    }
}

void setup() 
{
  Serial.begin(9600);

  pinMode(BUZZ_PIN, OUTPUT);
  digitalWrite(BUZZ_PIN, HIGH);  
  
  Timer1.initialize(1000000); // set timer for 1sec
  attachInterrupt(digitalPinToInterrupt (MOTOR1), 
     ISR_count1, RISING);  
    // Increase counter 1 when speed sensor pin goes High
  
  attachInterrupt(digitalPinToInterrupt (MOTOR2),
     ISR_count2, RISING); 
   // Increase counter 2 when speed sensor pin goes High

  makeSquare(3); // Calls the function so the robot can complete the task.
  stop_Stop();
} 
void loop()
{
}