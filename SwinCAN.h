#ifndef SWIN_CAN_H_
#define SWIN_CAN_H_

enum NodeClass {
	cube = 0x000,
	power_supply = 0x100,
	motor_driver = 0x200,
	arm_controller = 0x300
};

enum cmd_type {
	heart_beat = 0x01,
	drive_motor = 0x02,
	set_relay = 0x03,
	set_arm = 0x04
};


class swin_stepper : public HighPowerStepperDriver
{
public:
  swin_stepper() : 
  HighPowerStepperDriver()
  {
  }
};

class swin_side
{
public:
  bool dir;
  swin_stepper front;
  swin_stepper back;

  // cycles until next step
  int delayInitial = 0;
  int delayRemain = 0;

  //Can step
  bool canStep = true;

  swin_side()
  {
    bool dir = false;
    delayInitial = 0;
    delayRemain = 0;
  }
};

#endif // SWINCAN_H_
