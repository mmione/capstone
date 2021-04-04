
#define ENCODER_INPUT_1 3
#define ENCODER_INPUT_2 4
#define PWM_PIN 6
//#define BUTTON_INPUT 2
//#define POTENTIOMETER A0

// Defining output pins for controlling motor direction. 
#define INA 8
#define INB 9 

#define GEAR_RATIO 20
#define ENCODER_FACTOR 12


// Need to use volatile variables as their values may be messed with by the interrupts

volatile bool motordir = false; 
volatile uint32_t lastA = 0; // Unsigned int, 32bit long
volatile float RPM = 0; 

int buttonState = 1;
float potentiometerReading;

// BELOW is MPU stuff, merging in. 

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h" 
#include "Wire.h"
#include <PID_v1.h>
MPU6050 mpu;

// MPU control/status vars
bool MPUInitialized = false;  // set true if MPU initialized correctly
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

//Quaternion and gravity vectors are used to calculate ypr with more accuracy
Quaternion q;     //In this case it becomes a three dimensional vector which is used in calculations involving rotations      
VectorFloat gravity; //Such as the ypr calculations 
float ypr[3]; 

// Need to store the value of roll in deg.

float rollDeg;

//Needed everytime the FIFO buffer is filled with data and ready to be Dumped
volatile bool mpuInterrupt = false;     //When FIFO buffer its full, the interrupt becomes high
void dmpDataReady() {
    mpuInterrupt = true;
}

// PID stuff *********************************** // 

// SETPOINT: our GOAL angle
// INPUT: angle reading
// OUTPUT: PWM pin of motor. 
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void isr(){

  // Read motor's direction
  motordir = digitalRead(ENCODER_INPUT_2);

  uint32_t currA = micros(); // Take snapshot of time

  if(lastA < currA){

    float rev = currA - lastA;  // us
    rev = 1.0 / rev;            // rev per us
    rev *= 1000000;             // rev per sec
    rev *= 60;                  // rev per min
    rev /= GEAR_RATIO;             // account for gear ratio
    rev /= ENCODER_FACTOR;         // account for multiple ticks per rotation
    RPM = rev;
    
  }

  lastA = currA;
}


void setup() {

  // Setting up pins
  pinMode(ENCODER_INPUT_1, INPUT_PULLUP);
  pinMode(ENCODER_INPUT_2, INPUT_PULLUP);

  // Collection of pins BELOW are used for controlling motor driver.
  pinMode(PWM_PIN, OUTPUT); 
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  // Below is unneeded and was used for testing.
//  pinMode(BUTTON_INPUT, INPUT_PULLUP);
//  pinMode(POTENTIOMETER, INPUT);
 

  // One of the pins needs to be an interrupt pin, for the rotary encoder idea to work.
  attachInterrupt(digitalPinToInterrupt(ENCODER_INPUT_1), isr, RISING);

  // Serial for debugging 
  //Serial.begin(9600);

  // BELOW is MPU stuff

  //Joins the I2C bus
    Wire.begin();
    Serial.begin(115200);

    bool connectionTest = false;
    
    // initialize device
    Serial.println(F("Initializing I2C devices and testing connections..."));
    mpu.initialize();
    connectionTest = mpu.testConnection();

    if (connectionTest == true){
      Serial.println(F("Connection with MPU6050 SUCCESSFULL!!"));
    }
    else if (connectionTest == false){
      Serial.println(F("Connection with MPU6050 FAILED. FATAL ERROR!!"));
    }
    
    //Configure the digital motion processing
    devStatus = mpu.dmpInitialize();
    mpu.setDMPEnabled(true);

    //Sets up interrupt attatching it to pin 0 and enabling interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    
    MPUInitialized = true; //Flag so we know we can move on to main loop and start filling FIFO buffer
    packetSize = mpu.dmpGetFIFOPacketSize(); //Need to get packetSize so we know when the FIFO buffer is full 
    //and is ready to be read

    //Setup for your Gyroscope - Juan: Mine is very stable with these offsets for the pitch. Use whichever angle is more stable at a neutral position
//    mpu.setXGyroOffset(220);
//    mpu.setYGyroOffset(76);
//    mpu.setZGyroOffset(-85);
//    mpu.setZAccelOffset(1788); 

    //mpu.calibrateGyro();

    // PID SETUP ***************************************** //
    //initialize the variables we're linked to
    Input = 999;
    Setpoint = 0; // This is the angle we are always trying to target.
  
    //turn the PID on
    myPID.SetMode(AUTOMATIC);

    
}


void loop() {
    
//   potentiometerReading = (float)analogRead(POTENTIOMETER)/1023;
   
//   Serial.print("Duty Cycle:\t");
//   Serial.print(potentiometerReading);
//   Serial.print(" RPM:\t");
//   Serial.println(avgRPM(10));   
//   analogWrite(PWM_PIN,255*potentiometerReading); // Need around 120-130 to start. 
   
   
   //Serial.println(avgRPM(20));

    
    //analogWrite(PWM_PIN, 255*(!(rollDeg>2 && rollDeg<-2)));

    // INA = 1, INB = 0,  CLOCKWISE SPIN (facing the bike head on)
    // INA = 0, INB = 1,  COUNTERCLOCKWISE SPIN
    // if rollDeg > 0, tilted ccw, else tilted cw

//    // Branchless system to respond to 
//    digitalWrite(INA, 1*(rollDeg>2));
//    digitalWrite(INB, 1*(rollDeg<-2)); 

    if (!MPUInitialized){
      return; //Will only run if the initialization was unsuccessful or there was an error in the setup
    }

    //Need to wait till either the interrupt or the packet is available. Should be very short
    while (!mpuInterrupt && fifoCount < packetSize) {
    }

    // Resets interrupt status and waits until next interrupt is triggerred
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    //Gets the current First in First Out count and when it becomes as large as the packet size, we are ready to read it and use it for the quaternion
    fifoCount = mpu.getFIFOCount();

    //Checks when interrupt is triggered
     if (mpuIntStatus & 0x02) {
   
        while (fifoCount < packetSize){ //Wait until data length ready to be read. That would be when a whole packet is stored in the FIFO buffer
          fifoCount = mpu.getFIFOCount();
        }

        //Here, a packet from the FIFO buffer is taken which is then used for quaternion calculations
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        //If there is more than one packet available, the FIFO count reduces itself to stay in the loop and keep getting more bytes
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer); //Gets quaternion vector using whats stored in the FIFO Buffer

        // Possibly get rid of this -- may not be super necessary could approx to 9.8m/s^2? 
        mpu.dmpGetGravity(&gravity, &q); //Gets the gravity vector which is used for more accurate calculations even though its not necessary
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //Calculates the angles in radians for yaw pitch roll

        // We may just need the ROLL component to get things working. 
//        Serial.print("Yaw -> ");
//        Serial.print(ypr[0] * 180/M_PI); //Convert to degrees from rads
//        Serial.print("     Pitch -> ");
//        Serial.print(ypr[1] * 180/M_PI); //Convert to degrees from rads
        Serial.print("     Roll -> ");

        //rollDeg = ypr[2] * 180/M_PI;
        Input = ypr[2] * 180/M_PI;
        
        
        Serial.println(Input); //Convert to degrees from rads

        myPID.Compute();
        // Compute() function simply calculates and "puts" what it thinks
        // is the "right" value to the motor. 

        digitalWrite(INA, 1*(rollDeg>2));
        digitalWrite(INB, 1*(rollDeg<-2)); 
        analogWrite(PWM_PIN, 255);
      
//        analogWrite(PWM_PIN, 255);
//
//        // INA = 1, INB = 0,  CLOCKWISE SPIN (facing the bike head on)
//        // INA = 0, INB = 1,  COUNTERCLOCKWISE SPIN
//        
//        digitalWrite(INA, 1);
//        digitalWrite(INB, 0); 


  }

  
}

float avgRPM(int pollingInterval){ // Float has the same amt of precision as double on Arduino Uno

  // Take average of the RPM values to x amount of milliseconds
  
  int sum = RPM;
  int time = millis();
  int counter = 0;
  
  while(counter<=4){
    int currentTime = millis();
    if ((currentTime - time)>=pollingInterval){ // Essentially delays by 5*pollingInterval [ms]
      counter++;
      time = currentTime;
      sum += RPM;
    }
    
  }

  return sum/5;
}
