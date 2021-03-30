
//Self-Stabilizing Motorcycle Capstone
//molinacj

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h" 
#include "Wire.h"

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

//Needed everytime the FIFO buffer is filled with data and ready to be Dumped
volatile bool mpuInterrupt = false;     //When FIFO buffer its full, the interrupt becomes high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
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
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); 
}

void loop() {
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
        mpu.dmpGetGravity(&gravity, &q); //Gets the gravity vector which is used for more accurate calculations even though its not necessary
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //Calculates the angles in radians for yaw pitch roll
        Serial.print("Yaw -> ");
        Serial.print(ypr[0] * 180/M_PI); //Convert to degrees from rads
        Serial.print("     Pitch -> ");
        Serial.print(ypr[1] * 180/M_PI); //Convert to degrees from rads
        Serial.print("     Roll -> ");
        Serial.println(ypr[2] * 180/M_PI); //Convert to degrees from rads
    }
}
