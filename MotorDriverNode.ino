#include <mcp_can_dfs.h>
#include <mcp_can.h>
#include <SPI.h>
#include <HighPowerStepperDriver.h>
#include "SwinCAN.h"

#define MAX_CURRENT (4000)
#define MIN_DELAY_MICRO_SEC (30) 
#define LEFT_MOTOR_SPEED (0)
#define LEFT_STEPS (1)
#define RIGHT_MOTOR_SPEED (2)
#define RIGHT_STEPS (3)
#define MICRO_STEPS (4)
//
uint16_t microSteps = 256;

// Left Side Pins
const uint8_t DirLeftPin = 5; // needs connection
const uint8_t StepLeftPin = 11; // needs connection
static long int counterLeft, delayLeft;
int stepsLeft = 0;

// driver left
HighPowerStepperDriver frntLeft;
#define FrntLeftCS 8 // needs connection
HighPowerStepperDriver rearLeft;
#define RearLeftCS 9 // needs connection maybe 6

//Right Side Pins
const uint8_t DirRightPin = 4; // needs connection
const uint8_t StepRightPin = 10; // needs connection
static long int counterRight, delayRight;
int stepsRight = 0;

//driver right
HighPowerStepperDriver frntRight;
#define FrntRightCS A0 // needs connection A0
HighPowerStepperDriver rearRight;
#define RearRightCS 6 // needs connection

// SPI
#define PULSE_WIDTH 3

// CAN
const uint8_t CSCan = 17; // trace on PCB
MCP_CAN CAN(CSCan);

bool drivingRight;
bool drivingLeft;

void setup_timer()
{
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1 = 0;  // initialize counter value to 0
  // set compare match register for 2khz increments
  OCR1A = 824; // = (16*10^6) / (2 * interruptFreq * PreScaler) - 1, must be <= 65,535 (unsigned 16 bits). Timer 0 and 2 can only store upto 255 (unsigned 8 bits)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 and CS10 for prescaler of 64, see register chart for other prescalers, they are set by manipulating CS10, CS11 and CS12.
  TCCR1B |= (1 << CS11) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
}

void setMicroSteps() {
  frntLeft.setStepMode(microSteps);
  frntRight.setStepMode(microSteps);
  rearLeft.setStepMode(microSteps);
  rearRight.setStepMode(microSteps);
}

void step(int pin)
{
  digitalWrite(pin, HIGH);
  delayMicroseconds(PULSE_WIDTH);
  digitalWrite(pin, LOW);
  delayMicroseconds(PULSE_WIDTH);
}

void setDirection(int pin, bool dir)
{
  delayMicroseconds(PULSE_WIDTH);
  digitalWrite(pin, dir);
  delayMicroseconds(PULSE_WIDTH);
}

void setUpDriver(HighPowerStepperDriver sd, int cs) {
  sd.setChipSelectPin(cs);
  sd.resetSettings();
  sd.clearStatus();
  sd.setDecayMode(HPSDDecayMode::AutoMixed);
  sd.setCurrentMilliamps36v4(MAX_CURRENT);
  sd.setStepMode(HPSDStepMode::MicroStep16);
  sd.enableDriver();
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
  pinMode(StepLeftPin, OUTPUT);
  digitalWrite(StepLeftPin, LOW);
  pinMode(DirLeftPin, OUTPUT);
  digitalWrite(DirLeftPin, LOW);
  pinMode(StepRightPin, OUTPUT);
  digitalWrite(StepRightPin, LOW);
  pinMode(DirRightPin, OUTPUT);
  digitalWrite(DirRightPin, LOW);

  delay(3);

  //Set up drivers
  setUpDriver(frntLeft, FrntLeftCS);
  setUpDriver(rearLeft, RearLeftCS);
  setUpDriver(frntRight, FrntRightCS);
  setUpDriver(rearRight, RearRightCS);


  // initialise CAN connection
  CAN.begin(CAN_1000KBPS); // init can bus : baudrate = 1000k

  delay(10);

  counterLeft = counterRight = delayLeft = delayRight = 0;

  // setup timer configuration registers
  setup_timer();
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
      /*
      for (int i = 0; i < 2; i++) {Serial.print(buf[i], HEX); Serial.print('\t');}
      Serial.println(' ');
      */
      setMotorValues();
    }
    else if (canId == power_supply + heart_beat) {
      //heartbeat();
    }
  }
}

ISR(TIMER1_COMPA_vect) { //This functions runs when timer1 counter is equal to OCR1A
  read_CAN_flag = true;
}

// driver motors from command in buf[8]
//Can frame MotorLeft LeftSteps MotorRight RightSteps microStepping Blank Blank Blank
void setMotorValues() { 
  //Checking Motor Left value
  if (buf[LEFT_MOTOR_SPEED] == 151) {
    drivingLeft = false;
  }
  else if (buf[LEFT_MOTOR_SPEED] < 151) {
    setDirection(DirLeftPin, HIGH);
    delayLeft = map(buf[LEFT_MOTOR_SPEED], 151, 110, 255, 0);
    drivingLeft = true;
  } else if (buf[LEFT_MOTOR_SPEED] > 151) {
    setDirection(DirLeftPin, LOW);
    delayLeft = map(buf[LEFT_MOTOR_SPEED], 151, 192, 255, 0);
    drivingLeft = true;
  }
  //
  stepsLeft = buf[LEFT_STEPS];
  //
  if (buf[RIGHT_MOTOR_SPEED] == 151) {
    drivingRight = false;
  }
  else if (buf[RIGHT_MOTOR_SPEED] < 151) {
    setDirection(DirRightPin, HIGH);
    delayRight = map(buf[RIGHT_MOTOR_SPEED], 151, 110, 255, 0);
    drivingRight = true;
  } else if (buf[RIGHT_MOTOR_SPEED] > 151) {
    setDirection(DirRightPin, LOW);
    delayRight = map(buf[RIGHT_MOTOR_SPEED], 151, 192, 255, 0);
    drivingRight = true;
  }
  stepsRight = buf[RIGHT_STEPS];
  //
  if (pow(2, buf[MICRO_STEPS]) != microSteps) {
    microSteps = pow(2, buf[MICRO_STEPS]);
    setMicroSteps();
  }
}

void loop()
{
  // check if data coming
  if (read_CAN_flag) {
    read_CAN();
    read_CAN_flag = false;
  }

  // maybe perform stepping
  if (drivingLeft && (counterLeft > delayLeft + MIN_DELAY_MICRO_SEC)) {
    step(StepLeftPin);
    counterLeft = 0;
    stepsLeft--;
  }
  
  if (drivingRight && (counterRight > delayRight + MIN_DELAY_MICRO_SEC)) {
    step(StepRightPin);
    counterRight = 0;
    stepsRight--;
  }

  counterLeft++;
  counterRight++;
}
