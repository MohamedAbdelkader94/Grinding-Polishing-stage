


#include <Stepper.h>


const int speedPBInc  =2;//define input pin 
const int stopPB=3;//define input pin for Stop push button
const int speedPBDec =4;//define input pin 
const int motorPin[] ={8, 9, 10, 11};
const int direction =1;//0 for CW, 1 for CCW;
const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
int speedStep = 20;
int stepMinimum = 5;
int stepMaximum = 2000;
int stopType =0;//0=fully stopped , 1=hold (consumes energy) 
int incomingByte=0;
int currentSpeed=100;
int currentSPR=stepsPerRevolution;
int motorSpeed=300;
#define START 1 //
#define STOP 0
#define CW 1
#define CCW -1
int motorStopState=STOP;//change if you need to
// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, motorPin[0], motorPin[1], motorPin[2], motorPin[3]);

void setup() {
  //Robojax.com L298N Stepper Speed STLPB-01
  
  // initialize the serial port:
  Serial.begin(9600);
  pinMode(speedPBInc, INPUT_PULLUP);
  pinMode(stopPB,INPUT_PULLUP);
  pinMode(speedPBDec,INPUT_PULLUP); 
  //attachInterrupt(digitalPinToInterrupt(stopPB), stopMotor, FALLING);  
}

void loop() {
  
  updateState();

if(!motorStopState)
{
  currentSPR =0;
}else{
  currentSPR =stepsPerRevolution;  
}
  
  myStepper.setSpeed(currentSpeed);
  if(direction ==1)
  {
    myStepper.step(-currentSPR);
  }else{
    myStepper.step(currentSPR);    
  }
 

}//loop



void updateState()
{

 if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();}

 //Robojax.com L298N Stepper Speed STLPB-01
  if( incomingByte == 'b')
  {
   motorStopState =1-motorStopState;// stop the motor
    if(motorStopState  ==STOP)
    {
      stopMotor();

    }
  
 
  }
  
  if(digitalRead(stopPB) ==LOW )
  {
   motorStopState =1-motorStopState;// stop the motor
    if(motorStopState  ==STOP)
    {
      stopMotor();

      delay(500);
    }
  
 
  }
  if(digitalRead(speedPBInc) ==LOW ||incomingByte == 'u' )
  {
  motorStopState = START;
  currentSpeed += speedStep;
   if( currentSpeed >=stepMaximum )  currentSpeed =stepMaximum ;
  }

  if(digitalRead(speedPBDec) ==LOW ||incomingByte == 'd')
  {
  motorStopState = START;    
   currentSpeed -= speedStep;
   if( currentSpeed <stepMinimum )  currentSpeed =stepMinimum ;
  }  

  if( incomingByte == 'm' )
  {
  //delay(500);
  for(int i=0; i<4; i++)
    {
      digitalWrite(motorPin[i], LOW);
    }
  }
}//updateState end

 

void stopMotor()
{

  if(stopType ==0 )
  {
    for(int i=0; i<4; i++)
    {
      digitalWrite(motorPin[i], LOW);
    }
  }
  
}//stopMotor() end
