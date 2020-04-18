/************************************
Names: Garrett Genord, Josh Mcnabb, Ethan Bach, Connor Finkel
Class: EGR107-10
Professor KC
Final Robosockey Code
Description: This code will run our robot in the Final Robosockey Competition, there will be detailed comments throughout to describe what it does.
************************************/


unsigned long currentTime; // Both of these variables are to calculate how long we have been in each of our states
unsigned long startTime;

const int leftTrigPin = 26; // constant variables to hold the ultrasonic sensor pin numbers  
const int leftEchoPin = 28;
const int rightTrigPin = 25;
const int rightEchoPin = 23;
const int midTrigPin = 22;
const int midEchoPin = 24;


float leftUltrasonicDistance; // variables to hold the calculated distance from each of the ultrasonic sensors
float rightUltrasonicDistance;
float middleUltrasonicDistance;


bool needToBackupALIGN = true; /* this variable is used in the ALIGN state. it is used to determine if it is the first time in that state. 
                               if it is the first time, the robot will backup and then turning for the rest of the time in the state */
                               
bool needToSetDistanceWALLFOLLOW = true; /* this is used in the WALLFOLLOW state to tell if it is the first time in the state.
                                          if it is the first time, the robot will set its current distance from the right ultrasonic
                                           sensor as the distance to constantly compare against in order to follow the wall.*/

float wallUltrasonicDistance; // variable used to hold the right ultrasonic distance from the wall to compare against and wall follow


bool firstTimeShootingSHOOT = true; /* this variable is used in the SHOOT state. it is used to determine if it is the first time in that state. 
                                    if it is the first time, the robot will turning until the middle ultrasonic distance gets really big
                                    AKA, the robot is facing the goal*/ 


bool needToBackupSHOOT = true; /* this variable is used in the SHOOT state. it is used to determine if the robot should jerk back and forward while it shoots.*/
                               
char directionToTurnUNSTUCK = 'R'; /* this variable is used in the UNSTUCK state, but set in the SEARCH state. it is set right before entering the 
                                    UNSTUCK state by comparing the ultrasonic distances to tell which direction the robot should turn towards*/


bool upperRightLimitOn = false; // variable that are set to true if the respective limit switch is pushed
bool upperLeftLimitOn = false;
bool lowRightLimitOn = false;
bool lowLeftLimitOn = false;

const int upperRightLimitPin = 43; // constant variables that hold the digital pin that each limit switch is attached to 
const int upperLeftLimitPin = 37;
const int lowRightLimitPin = 31;
const int lowLeftLimitPin = 39;


const int leftPWM = 13; //variables that hold the pins that control the motors
const int leftIn1 = 42;
const int leftIn2 = 38;
const int rightPWM = 10;
const int rightIn1 = 41;
const int rightIn2 = 33;

const int normalLeftSpeed = 120; // variables that hold the speed that the motors should go in the SEARCH state
const int normalRightSpeed = 120;

int rightWallSpeed = 115; // variables that hold the speed the motors should go when wall following
int leftWallSpeed = 115; 

int rightShootSpeed = 150; // variables that hold the speed the motors should go when in the SHOOT state jerking back and forth
int leftShootSpeed = 150;

unsigned long conditiontToEnterSHOOTtime; // this variable holds the time that is reset every time that the conditions to enter the SHOOT state are false. used for an if condition over time
unsigned long conditiontoBeginShootingMotionTime; // this variable holds the time that is reset every time that the conditition to begin the shooting motion is false. used for an if condition over time
unsigned long conditionForWALLFOLLOWtimeout; // this variable holds the time thats reset every time the condition to exit the WALLFOLLOW state is false. used for an if condition over time

#include<Servo.h> // including servo library
Servo roller; // naming the servo "roller"

const int servoPin = 2; // setting the servo pin to 2

enum states { // these are the states for our state machine, they will be explained in the loop function
  
  SEARCH,
  UNSTUCK,
  WALLFOLLOW,
  SHOOT, 
  ALIGN
 
  };

enum states currentState; // variable used to hold the current state

void setup() {
   Serial.begin(9600); // starting serial communication for testing purposes

   pinMode(leftTrigPin,OUTPUT); // setting all of the trigger pins for the ultrasonics to output
   pinMode(rightTrigPin,OUTPUT); 
   pinMode(midTrigPin,OUTPUT); 
      
   digitalWrite(leftTrigPin,LOW);  // making sure all the ultrasonics do not start sending out a signal
   digitalWrite(rightTrigPin,LOW);  
   digitalWrite(midTrigPin,LOW);

   pinMode(upperRightLimitPin, INPUT_PULLUP);  // setting the pins for the limit switch to an input pull up
   pinMode(upperLeftLimitPin, INPUT_PULLUP); 
   pinMode(lowRightLimitPin, INPUT_PULLUP); 
   pinMode(lowLeftLimitPin, INPUT_PULLUP); 

   roller.attach(servoPin); // attaching the servo and making sure it starts out not moving
   roller.write(90);

   pinMode(leftPWM, OUTPUT);  // setting all the pins for motor control to output
   pinMode(leftIn1, OUTPUT); 
   pinMode(leftIn2, OUTPUT); 
   pinMode(rightPWM, OUTPUT); 
   pinMode(leftIn1, OUTPUT); 
   pinMode(leftIn2, OUTPUT);

   digitalWrite(leftIn1,LOW); // making sure the motors start not moving
   digitalWrite(leftIn2,LOW);
   digitalWrite(rightIn1,LOW);
   digitalWrite(rightIn2,LOW);
   analogWrite(rightPWM,0);
   analogWrite(leftPWM,0);
  
   currentState = SEARCH; // setting the current state to SEARCH and setting the start time
   startTime = millis();
}

void loop() {
  updateSensors(); // updating the sensors each time through the loop
  currentTime = millis();  // updating the current time each time through the loop
 
  switch(currentState)
  {
    //////////////////// beginning of switch statement
    
    case SEARCH:///////////////////////////////////////////////////////////////////////////////////////////////////////// start of SEARCH state
      /*this is the state the robot is set to to start. for the first 1:30, it will just keep moving forward at the same speed until any of the
      the limit switches go off, or it has been in this state for longer than 8 seconds.  before it exits this state, it compares
      the ultrasonic distances and whichever one is greater, it will set a char to 'L' or 'R' to signal which way to turn in the UNSTUCK 
      state. after the first 1:30, if one of the upper limit switches goes off, or aka it has hit a wall, it will enter the WALLFOLLOW state
      If it is in this state, and there are only 5 seconds left in the competition, it will enter the SHOOT state*/
      
      bothMotorsForward();                          // moving forward with the servo rotating inward to collect balls
      analogWrite(rightPWM,normalRightSpeed);
      analogWrite(leftPWM,normalLeftSpeed);
      roller.write(180);
       
      if( upperRightLimitOn or upperLeftLimitOn or lowLeftLimitOn or lowRightLimitOn or (currentTime - startTime > 8000) ){
                                                       /* if any of the limit switches go off or we have been in this state
                                                       longer than 8 secs, check the ultrasonic distances and enter UNSTUCK*/
          if(leftUltrasonicDistance > rightUltrasonicDistance and leftUltrasonicDistance < 160){
            directionToTurnUNSTUCK = 'L';
            } 
            
          if(rightUltrasonicDistance > leftUltrasonicDistance and rightUltrasonicDistance < 160){
            directionToTurnUNSTUCK = 'R';
           }
           
           st0p();
           currentState = UNSTUCK;
           startTime = millis();       
      }
     
     if(  (millis() > 90000) and (upperRightLimitOn or upperLeftLimitOn)  ){ // if a 1:30 has passed and one of the upper limit switches goes off, enter WALLFOLLOW
        currentState = WALLFOLLOW;
        startTime = millis();
        needToSetDistanceWALLFOLLOW = true;         
       }
        
      if(millis() > 175000){ // if there are 5 seconds left in the comp, enter the SHOOT state
          currentState = SHOOT;
          startTime = millis();
        }
        
        
    break;/////////////////////////////////////////////////////////////////////////////////////////////////////////////// end of SEARCH state
    
    case UNSTUCK://////////////////////////////////////////////////////////////////////////////////////////////////////// start of UNSTUCK state 
      /* this is the state the robot is set in after it gets stuck or it has been in its current state for too long. 
      it backs up for just less than one second and then turns to the designated direction from the
      ultrasonic distance readings for about half a second and then goes back to the search state. if we are in this 
      state and there are only 5 secs left, we will enter the shoot state*/
      
      reverseBothMotors();                     // moving backwards
      analogWrite(leftPWM,normalLeftSpeed + 15); 
      analogWrite(rightPWM,normalRightSpeed + 15);
     
      if(currentTime - startTime > 950 and directionToTurnUNSTUCK == 'R'){                                                               
          leftMotorForwardRightMotorBackward();                     // if we have been backing for longer than 950 ms and the right ultrasonic distance is bigger, turn right
          analogWrite(leftPWM,214 + 5);
          analogWrite(rightPWM,214 + 5);
        } else if (currentTime - startTime > 950 and directionToTurnUNSTUCK == 'L'){
            rightMotorForwardLeftMotorBackward();                // if we have been backing for longer than 950 ms and the left ultrasonic distance is bigger, turn left
            analogWrite(leftPWM,214 + 5);
            analogWrite(rightPWM,214 + 5);
            }

            
      if(millis() > 175000){ // if in this state wtih only 5 seconds left, enter the SHOOT state
          currentState = SHOOT;
          startTime = millis();  
        }
    
      if(currentTime-startTime > 1350){ // if we have been turning for longer than a half second, go back to SEARCH
          st0p();
          currentState = SEARCH;
          startTime = millis();
        }
    break; //////////////////////////////////////////////////////////////////////////////////////////////////////////// end of UNSTUCK state
    
    case ALIGN://///////////////////////////////////////////////////////////////////////////////////////////////////// start of ALIGN state
    /* this state is entered from the WALLFOLLOW state, it is used to back up a little, and then turn away from the wall a little. 
    it is meant to be called repeatedly in WALLFOLLOW in a hope that eventually the robot will be driving parallel with the wall to the 
    right of it (so that it can follow it). after it backs up and turns, it will enter the WALLFOLLOW state again. if we are in this state
    with only 5 seconds left in the comp, we will enter the SHOOT state*/
    
      if(needToBackupALIGN){ // if it is the first time in this state, which is designated by this boolean variable whenever this state is entered, backup for less than .5 seconds
        reverseBothMotors();
        analogWrite(rightPWM,normalRightSpeed + 44);
        analogWrite(leftPWM,normalLeftSpeed  + 44);
        
          if(currentTime - startTime >= 330){ //  if we have been backing up for longer than 330 ms, we dont want to backup anymore, so set the boolean variable to false
            needToBackupALIGN = false;
            }
     
      }
    
     
      if(needToBackupALIGN == false){ // if we no longer need to be backing up, turn slightly away from the wall we are following
          rightMotorForwardLeftMotorBackward();
          analogWrite(rightPWM,normalRightSpeed + 44);
          analogWrite(leftPWM,normalLeftSpeed + 44 );
        }

      if(currentTime - startTime > 500){ // if we have been in this state for longer than .5 seconds, go back to following the wall
        currentState = WALLFOLLOW;
        startTime = millis();
        needToSetDistanceWALLFOLLOW = true; 
       }
    
      if(millis() > 175000){ // if we are in this state and there are 5 seconds left in the comp, enter the SHOOT state
        currentState = SHOOT;
        startTime = millis();  
        }
         
    
    break;/////////////////////////////////////////////////////////////////////////////////////////////////////// end of ALIGN state
    
    case WALLFOLLOW: /////////////////////////////////////////////////////////////////////////////////////////// start of WALLFOLLOW state
    /* this state is entered after 1:30 has passed and an upper limit switch has gone off. 
    if it is the first time this state has been entered from a different one, it will set the wall distance from the right ultrasonic sensor
    and constantly compare against it to see which motor speed it should be increasing. the robot will just keep following the righthand wall
    until one of the upper limit switches goes off (aka we hit a wall), and when one of them goes off, it enteres the ALIGN state to back up 
    and turn a little until it is driving parallel to the wall. if we have been in this state and no limit switches have gone off for 
    8 seconds, we wil enter the unstuck state and go somewhere else. if we are following the wall and the right Ultrasonic distance gets
    realy big (and the other distances are in a range that is consistent with being in the relative location of the goal) for longer than
    700 ms, we will enter the SHOOT state and start to turn to face the goal if we are in this state with 5 seconds left, we will enter the
    SHOOT State*/
    
        if(needToSetDistanceWALLFOLLOW){ // if it is the first time in this state set the wall distance to compare against
          wallUltrasonicDistance = rightUltrasonicDistance;
          needToSetDistanceWALLFOLLOW = false; 
        }
    
      if(upperRightLimitOn == false and upperLeftLimitOn == false){ // if neither of the limit switches are on follow the wall
         
        if(rightUltrasonicDistance > wallUltrasonicDistance){ 
            bothMotorsForward();               /* if the right ultrasonic distance is bigger than the set distance, the left motor needs to be increased to come back towards the wall*/
            analogWrite(rightPWM,rightWallSpeed - 25);
            analogWrite(leftPWM,leftWallSpeed + 25);
          } else if (rightUltrasonicDistance < wallUltrasonicDistance){
                 bothMotorsForward();        /* if the right ultrasonic distance is less than the set distance, the right motor needs to be increased to move away from the wall*/
                 analogWrite(rightPWM,rightWallSpeed + 15);
                 analogWrite(leftPWM,leftWallSpeed - 15);
            }
      } else if ((upperRightLimitOn or upperLeftLimitOn)){ // if one of the upper limit switches goes off, we have hit a wall and need to enter the ALIGN state to turn away from it
          currentState = ALIGN;
          startTime = millis();
          needToBackupALIGN = true;
        
        }


      if(upperRightLimitOn or upperLeftLimitOn){ // if the upper limit switches go off, reset the time condition for exiting the state after a limit switch not going off for 8 secs
        conditionForWALLFOLLOWtimeout = millis();  
      }
  
      if(millis() - conditionForWALLFOLLOWtimeout > 8000){ // if the difference between the current time and the set time is bigger than 8 secs, then a limit switch hasnt been pressed in 8 seconds
        currentState = UNSTUCK;
        startTime = millis(); 
       } 
  
      if(   (rightUltrasonicDistance < 130 or rightUltrasonicDistance > 250) or (leftUltrasonicDistance < 40 or leftUltrasonicDistance > 70) or (middleUltrasonicDistance < 50 or middleUltrasonicDistance > 80)  )
          conditiontToEnterSHOOTtime = millis();            /* this if statement is checking if the robot is in the relative area of the goal with our specific arena dimensions
                                                             and it will keep resetting the time if it does not meet the conditions*/
      if(millis() - conditiontToEnterSHOOTtime >= 700){ // if the time has not been reset for 700ms, then we are near the goal and must turn and shoot
          delay(250);
          currentState = SHOOT;
          startTime = millis();
        }
    
      if(millis() > 175000){ // if we are in this state and there are 5 seconds left in the competition, enter the shoot state.
        currentState = SHOOT;
        startTime = millis();  
      }
      
      if(lowRightLimitOn or lowLeftLimitOn){ // if one of the lower limit switches goes off, aka we hit an obstacle, enter the UNSTUCK state 
        currentState = UNSTUCK;
        startTime = millis();  
      }
      
    break;////////////////////////////////////////////////////////////////////////////////////////////////////// end of WALLFOLLOW state
    
    case SHOOT: /////////////////////////////////////////////////////////////////////////////////////////////// start of the SHOOT state
      /*this state is for turning and facing the goal, and then shooting. if it is the first time in this state from another one, 
        it will turn until it is facing the goal and then jerk back and forth a few times while rotating the servo outward*/
  
       if(firstTimeShootingSHOOT){  // if it is the first time in this state, turn towards the goal
          leftMotorForwardRightMotorBackward();
          analogWrite(rightPWM,normalRightSpeed + 15);
          analogWrite(leftPWM,normalLeftSpeed + 15);
        
        }
         
        if(middleUltrasonicDistance < 130) // if the middle ultrasonic distance is not really big, aka not facing the goal, reset the time condition.
           conditiontoBeginShootingMotionTime = millis();

      if(millis() - conditiontoBeginShootingMotionTime >= 100 or (currentTime - startTime > 5000)){ 
          firstTimeShootingSHOOT = false; /* if the difference between the current time and time it has not met the condition is bigger than 100ms, we are facing the goal and should shoot
                                            or if the robot gets stuck on something and is turning for longer than 5 seconds, it should shoot*/
        }

        if(firstTimeShootingSHOOT == false and needToBackupSHOOT){ // after we are done turning to face the goal, start jerking back and forth while rotating the servo outward 
            reverseBothMotors();
            analogWrite(rightPWM,rightShootSpeed);
            analogWrite(leftPWM,leftShootSpeed + 10);
            roller.write(0);
            delay(500);
            bothMotorsForward();
            analogWrite(rightPWM,rightShootSpeed + 10);
            analogWrite(leftPWM,leftShootSpeed);
            delay(500);
            reverseBothMotors();
            analogWrite(rightPWM,rightShootSpeed);
            analogWrite(leftPWM,leftShootSpeed + 10);
            roller.write(0);
            delay(500);
            bothMotorsForward();
            analogWrite(rightPWM,rightShootSpeed + 10);
            analogWrite(leftPWM,leftShootSpeed);
            delay(500);
            reverseBothMotors();
            analogWrite(rightPWM,rightShootSpeed);
            analogWrite(leftPWM,leftShootSpeed + 10);
            roller.write(0);
            delay(500);
            bothMotorsForward();
            analogWrite(rightPWM,rightShootSpeed + 10);
            analogWrite(leftPWM,leftShootSpeed);
            delay(500);
            st0p();
            needToBackupSHOOT = false;  
    }
    
   
    
    break; /////////////////////////////////////////////////////////////////////////////////////// end of the SHOOT state
    

  } // end of switch statement

} // end of loop function



int updateSensors(){ // this function updates all of the sensors each time it is called
  
    unsigned long t1; // variables used to hold time values
    unsigned long t2;
    unsigned long pulse; // the pulse calculated 
    float cm; // variable to hold distance 
    
  
///////////////////////////////////////////////////////////////////// getting the left ultrasonic distance
    digitalWrite(leftTrigPin,HIGH); 
    delayMicroseconds(10);
    digitalWrite(leftTrigPin,LOW); 

    t1 = millis();
    while ( digitalRead(leftEchoPin) == LOW ){
        t2 = millis();
        if((t2 - t1) > 2000)
            return (-1);
      }

    t1 = micros();
    while( (digitalRead(leftEchoPin) == HIGH)  );
    t2 = micros();

     pulse = t2 - t1; // finding pulse by differencing the times
     cm = pulse / 58.0; // dividing by 58 which is a conversion factor to cm
     if(cm < 400){
        leftUltrasonicDistance = cm;
      } else 
        leftUltrasonicDistance = -2;
////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////// getting the left ultrasonic distance
    digitalWrite(rightTrigPin,HIGH); // chirping the sensor for 10 micro seconds
    delayMicroseconds(10);
    digitalWrite(rightTrigPin,LOW); 

    t1 = millis();
    while ( digitalRead(rightEchoPin) == LOW ){
        t2 = millis();
        if((t2 - t1) > 2000)
            return (-1);
      }
      
    t1 = micros();
    while( (digitalRead(rightEchoPin) == HIGH)  );
    t2 = micros();

     pulse = t2 - t1; // finding pulse by differencing the times
     cm = pulse / 58.0; // dividing by 58 which is a conversion factor to cm
     if(cm < 400){
        rightUltrasonicDistance = cm;
      } else 
        rightUltrasonicDistance = -2;
//////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////// getting the middle ultrasonic distance
  digitalWrite(midTrigPin,HIGH); // chirping the sensor for 10 micro seconds
    delayMicroseconds(10);
    digitalWrite(midTrigPin,LOW); 

    t1 = millis();
    while ( digitalRead(midEchoPin) == LOW ){
        t2 = millis();
        if((t2 - t1) > 2000)
            return (-1);
      }
      
    t1 = micros();
    while( (digitalRead(midEchoPin) == HIGH)  );
    t2 = micros();

     pulse = t2 - t1; // finding pulse by differencing the times
     cm = pulse / 58.0; // dividing by 58 which is a conversion factor to cm
     if(cm < 400){
        middleUltrasonicDistance = cm;
      } else 
        middleUltrasonicDistance= -2;

///////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////// seeing if the limit switches are on, and if they are setting the correct boolean variable
      if(digitalRead(upperRightLimitPin) == LOW){
          upperRightLimitOn = true;
        } else if (digitalRead(upperRightLimitPin) == HIGH){
            upperRightLimitOn = false;
         }

       if(digitalRead(upperLeftLimitPin) == LOW){
          upperLeftLimitOn = true;
        } else if (digitalRead(upperLeftLimitPin) == HIGH){
            upperLeftLimitOn = false;
          }

          
        if(digitalRead(lowLeftLimitPin) == LOW){
          lowLeftLimitOn = true;
          } else if (digitalRead(lowLeftLimitPin) == HIGH){
              lowLeftLimitOn = false;
            }

        if(digitalRead(lowRightLimitPin) == LOW){
            lowRightLimitOn = true;
          } else if (digitalRead(lowRightLimitPin) == HIGH){
              lowRightLimitOn = false;
            }
         
}//// end of update sensors function



void bothMotorsForward(){ // this function sets the correct pins on the h bridge for have both motors move forward
  
    digitalWrite(leftIn1,HIGH);
    digitalWrite(leftIn2,LOW);
    digitalWrite(rightIn1,LOW);
    digitalWrite(rightIn2,HIGH);
  
  }

void reverseBothMotors() { // this function sets the correct pins on the h bridge for have both motors move backward
  
    digitalWrite(leftIn1,LOW);
    digitalWrite(leftIn2,HIGH);
    digitalWrite(rightIn1,HIGH);
    digitalWrite(rightIn2,LOW);
  
  }

void rightMotorForwardLeftMotorBackward (){ // this function sets the correct pins on the h bridge to have the right motor move forward and the left motor move backward
  
    digitalWrite(rightIn1,LOW);
    digitalWrite(rightIn2,HIGH);
    digitalWrite(leftIn1,LOW);
    digitalWrite(leftIn2,HIGH);
    
}


void leftMotorForwardRightMotorBackward (){ // this function sets the correct pins on the h bridge to have the left motor move forward and the right motor move backward
  
  digitalWrite(leftIn1,HIGH);
  digitalWrite(leftIn2,LOW);
  digitalWrite(rightIn1,HIGH);
  digitalWrite(rightIn2,LOW);
  
}

void st0p(){ // this function stops both the motors
  
   analogWrite(rightPWM,0);
   analogWrite(leftPWM,0);
   
}
