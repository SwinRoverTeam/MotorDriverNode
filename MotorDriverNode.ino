// simple constant speed driving

#include <mcp_can_dfs.h>
#include <mcp_can.h>
#include <SPI.h>
#include <swinCan.h>
#define MIN_DELAY_MICRO_SEC (100)//Try to keep above 2*pulse width - minimum 50.
#define LEFT_MOTOR_SPEED (0)
#define LEFT_STEPS (1)
#define RIGHT_MOTOR_SPEED (2)
#define RIGHT_STEPS (3)
#define TORQUE (4)


unsigned long currenttime;
unsigned long laststeptime;
long stepinterval = 1280;


long counter;

// init motor values
int DirFrontLeftValue = 0;
int DirFrontRightValue = 0;
int DirRearLeftValue = 0;
int DirRearRightValue = 0;
int StepFrontLeftValue = 0;
int StepFrontRightValue = 0;
int StepRearLeftValue = 0;
int StepRearRightValue = 0;


//
int delayScalar = 1;

// Left Side Pins
const uint8_t DirFrontLeftPin = 4; // Green
const uint8_t DirRearLeftPin = 5; // Yellow
const uint8_t StepFrontLeftPin = 9; // Green
const uint8_t StepRearLeftPin = 10; // Yellow
static long int counterLeft, delayLeft;
int stepsLeft = 0;

// driver left

//Right Side Pins
const uint8_t DirFrontRightPin = 6; // Purple
const uint8_t DirRearRightPin = 8; // Orange
const uint8_t StepFrontRightPin = 11; // Purple
const uint8_t StepRearRightPin = 12; // Orange
static long int counterRight, delayRight;
int stepsRight = 0;

bool l_state = false;
bool r_state = false;

// SPI
#define PULSE_WIDTH 75 //Don't go lower than 25

// CAN
const uint8_t CSCan = 17; // trace on PCB
MCP_CAN CAN(CSCan);

bool drivingRight = false;
bool drivingLeft = false;

void setup_timer()
{
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1 = 0;  // initialize counter value to 0
  // set compare match register for 2khz increments
  OCR1A = 924; // = (16*10^6) / (2 * interruptFreq * PreScaler) - 1, must be <= 65,535 (unsigned 16 bits). Timer 0 and 2 can only store upto 255 (unsigned 8 bits)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 and CS10 for prescaler of 64, see register chart for other prescalers, they are set by manipulating CS10, CS11 and CS12.
  TCCR1B |= (1 << CS11) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
}

void flip_left()
{
  l_state = !l_state;
  digitalWrite(StepFrontLeftPin, l_state);
  digitalWrite(StepRearLeftPin, l_state);
}

void flip_right()
{
  r_state = !r_state;
  digitalWrite(StepFrontRightPin, r_state);
  digitalWrite(StepRearRightPin, r_state);
}

void setDirection(int pin, bool dir)
{
  delayMicroseconds(PULSE_WIDTH);
  digitalWrite(pin, dir);
  delayMicroseconds(PULSE_WIDTH);
}

void setup()
{
  Serial.begin(115200);
  
  while (CAN_OK != CAN.begin(CAN_1000KBPS))    // init can bus : baudrate = 500k
  {
      Serial.println("CAN BUS FAIL!");
      delay(100);
  }
  Serial.println("CAN BUS OK!");

  SPI.begin();

  // Drive the STEP and DIR pins low initially.
  pinMode(StepFrontLeftPin, OUTPUT);
  digitalWrite(StepFrontLeftPin, LOW);
  pinMode(StepRearLeftPin, OUTPUT);
  digitalWrite(StepRearLeftPin, LOW);
  pinMode(DirFrontLeftPin, OUTPUT);
  digitalWrite(DirFrontLeftPin, LOW);
  pinMode(DirRearLeftPin, OUTPUT);
  digitalWrite(DirRearLeftPin, LOW);

  pinMode(StepFrontRightPin, OUTPUT);
  digitalWrite(StepFrontRightPin, LOW);
  pinMode(StepRearRightPin, OUTPUT);
  digitalWrite(StepRearRightPin, LOW);
  pinMode(DirFrontRightPin, OUTPUT);
  digitalWrite(DirFrontRightPin, LOW);
  pinMode(DirRearRightPin, OUTPUT);
  digitalWrite(DirRearRightPin, LOW);
  
  // initialise CAN connection
  CAN.begin(CAN_1000KBPS); // init can bus : baudrate = 1000k

  counter = counterLeft = counterRight = delayLeft = delayRight = 0;

  // setup timer configuration registers
  setup_timer();
  laststeptime = micros(); 

}

// read a frame of CAN into buf[8]
unsigned char buf[8]; bool read_CAN_flag = false;
void read_CAN()
{
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    unsigned char len = 0;
    CAN.readMsgBuf(&len, buf);
    unsigned long canId = CAN.getCanId();

    if (canId == cube + drive_motor) {
      setMotorValues();
    }
  }
}

ISR(TIMER1_COMPA_vect) { //This functions runs when timer1 counter is equal to OCR1A
  read_CAN_flag = true;
}

// driver motors from command in buf[8]
//Can frame MotorLeft LeftSteps MotorRight RightSteps microStepping Blank Blank Blank
void setMotorValues() { 
  //Serial.println("Revieved CAN message");
  delayScalar = 1;//(buf[TORQUE] - 4); //Remove -3 when microstepping is fixed
  int max = 400 + 50 * delayScalar;
  int min = 200 + 50 * delayScalar;
  //Checking Motor Left value
  int left_speed = buf[LEFT_MOTOR_SPEED];
  int right_speed = buf[RIGHT_MOTOR_SPEED];
  //Serial.println(right_speed + leftspeed);
  if (left_speed > 135 || right_speed > 135 ) {
    //Serial.println("Out of Range");
    return;
  }
  if (left_speed >= 89 && left_speed <= 92) {
    drivingLeft = false;
  }
  else if (left_speed < 89) {
    setDirection(DirFrontLeftPin, HIGH);
    setDirection(DirRearLeftPin, HIGH);
    //Serial.println("Left Back");
    delayLeft = map(left_speed, 88, 50, max, min);
    drivingLeft = true;
  } else if (left_speed > 92) {
    setDirection(DirFrontLeftPin, LOW);
    setDirection(DirRearLeftPin, LOW);
    //Serial.println("Left Forward");
    delayLeft = map(left_speed, 93, 132, max, min);
    drivingLeft = true;
  }
  //
  stepsLeft = buf[LEFT_STEPS];
  //
  if (right_speed >= 89 && right_speed <= 92) {
    drivingRight = false;
  }
  else if (right_speed < 89) {
    setDirection(DirFrontRightPin, LOW);
    setDirection(DirRearRightPin, HIGH);
    delayRight = map(right_speed, 88, 50, max, min);
    drivingRight = true;
  } else if (right_speed > 92) {
    setDirection(DirFrontRightPin, HIGH);
    setDirection(DirRearRightPin, LOW);
    delayRight = map(right_speed, 93, 132, max, min);
    drivingRight = true;
  }
  stepsRight = buf[RIGHT_STEPS];
  //Serial.println(delayScalar);
}


void loop()
{
  
   currenttime = micros(); 

  
  // check if data coming
  if (read_CAN_flag) {
    read_CAN();
    read_CAN_flag = false;
  }




if (currenttime-laststeptime > stepinterval){
    if (drivingLeft){
    StepFrontLeftValue = 1-StepFrontLeftValue;
    StepRearLeftValue = 1-StepRearLeftValue;
  }
  
  if (drivingRight){
    StepFrontRightValue = 1-StepFrontRightValue;
    StepRearRightValue = 1-StepRearRightValue;
  }

  

  laststeptime = currenttime;
}


// set values
  digitalWrite(StepFrontLeftPin, StepFrontLeftValue);
  digitalWrite(StepRearLeftPin, StepRearLeftValue);
  digitalWrite(StepFrontRightPin, StepFrontRightValue);
  digitalWrite(StepRearRightPin, StepRearRightValue);
  
}
