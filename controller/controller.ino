
#define ENCODER_INPUT_1 3
#define ENCODER_INPUT_2 4
#define PWM_PIN 5 

#define GEAR_RATIO 20
#define ENCODER_FACTOR 12


// Need to use volatile variables as their values may be messed with by the interrupts

volatile bool motordir = false; 
volatile uint32_t lastA = 0; // Unsigned int, 32bit long
volatile float RPM = 0; 

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

  pinMode(PWM_PIN, OUTPUT);

  // One of the pins needs to be an interrupt pin, for the rotary encoder idea to work.
  attachInterrupt(digitalPinToInterrupt(ENCODER_INPUT_1), isr, RISING);

  // Serial for debugging 
  Serial.begin(9600);
  
}


void loop() {

  //Serial.println(millis());
  PWM(1000,1, PWM_PIN);
  
  
  
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

int PWM(int period, float dutyCycle, int pinNumber){
  int startTime = millis();
  int currentTime = 0;
 
  digitalWrite(pinNumber,HIGH);

  while(currentTime - startTime < period){

    if(currentTime >= startTime + period*dutyCycle){
      digitalWrite(pinNumber,LOW);
    }
    currentTime = millis();
  } 
  return 0;

}
