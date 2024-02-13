#include <mcp_can_dfs.h>
#include <mcp_can.h>
#include <SPI.h>
#include <HighPowerStepperDriver.h>

#define MAX_CURRENT (4000)
#define CUBE_CAN_ID (0x01)
// Left Side Pins
const uint8_t DirLeftPin = 11; // needs connection
const uint8_t StepLeftPin = 5; // needs connection
static long int counterLeft, delayLeft;

// driver front left
HighPowerStepperDriver frntLeft;
#define FrntLeftCS 6 // needs connection
HighPowerStepperDriver rearLeft;
#define RearLeftCS A0 // needs connection

//Right Side Pins
const uint8_t DirRightPin = 10; // needs connection
const uint8_t StepRightPin = 4; // needs connection
static long int counterRight, delayRight;

//driver front right
HighPowerStepperDriver frntRight;
#define FrntRightCS 8 // needs connection
HighPowerStepperDriver rearRight;
#define RearRightCS 9 // needs connection

// SPI
const uint8_t HighPulseWidth = 3;

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

void step(int pin)
{
  digitalWrite(pin, HIGH);
  delayMicroseconds(HighPulseWidth);
  digitalWrite(pin, LOW);
  delayMicroseconds(HighPulseWidth);
}

void setDirection(int pin, bool dir)
{
  delayMicroseconds(3);
  digitalWrite(pin, dir);
  delayMicroseconds(3);
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
unsigned char buf[8]; unsigned long canId = CUBE_CAN_ID; bool read_CAN_flag = false;
void read_CAN()
{
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    unsigned char len = 0;
    CAN.readMsgBuf(&len, buf);
    canId = CAN.getCanId();

    if (canId == 1) {
      /*
      for (int i = 0; i < 2; i++) {Serial.print(buf[i], HEX); Serial.print('\t');}
      Serial.println(' ');
      */

      handle_CAN();
    }
  }
  /* fake reading CAN
  buf[0] = 190; // full forward
  buf[1] = 170; //half ish
  */
}

ISR(TIMER1_COMPA_vect) { //This functions runs when timer1 counter is equal to OCR1A
  read_CAN_flag = true;
}

// driver motors from command in buf[8]
void handle_CAN() { 
  if (canId == CUBE_CAN_ID) {
    if (buf[0] == 151) {
      drivingLeft = false;
    }
    if (buf[1] == 151) {
      drivingRight = false;
    }
    
    if (buf[0] < 151) {
      setDirection(DirLeftPin, HIGH);
      delayLeft = map(buf[0], 151, 110, 255, 0);
      drivingLeft = true;
    } else if (buf[0] > 151) {
      setDirection(DirLeftPin, LOW);
      delayLeft = map(buf[0], 151, 192, 255, 0);
      drivingLeft = true;
    }
    if (buf[1] < 151) {
      setDirection(DirRightPin, HIGH);
      delayRight = map(buf[1], 151, 110, 255, 0);
      drivingRight = true;
    } else if (buf[1] > 151) {
      setDirection(DirRightPin, LOW);
      delayRight = map(buf[1], 151, 192, 255, 0);
      drivingRight = true;
    }
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
  if (drivingLeft && (counterLeft > delayLeft + 30)) {
    step(StepLeftPin);
    counterLeft = 0;
  }
  
  if (drivingRight && (counterRight > delayRight + 30)) {
    step(StepRightPin);
    counterRight = 0;
  }

  counterLeft++;
  counterRight++;
}
