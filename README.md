# MotorDriverNode
This code is for the CanBed that recieves CAN message from the CubePilot and then drives the Polulu Stepper drivers that control each wheel motor.

## Current limit
Each stepper motor has a set current limit that it can handle, the motors being used for Rock Muncher are 4000mA max so the MAX_CURRENT has been set to 4000, change as necessary before uploading to the CanBed.

## Pin settout
The CanBed will need to be connected to all four of the stepper drivers, some connections are shared between all four, some are shared between two and others are only connected to one board. The list of each is as follows:
### Shared with all
CanBed **SO** -> Driver **SDATAO** -- Controller In Peripheral Out (CIPO)\
CanBed **SI** -> Driver **SDATAI** -- Controller Out Peripheral In (COPI)\
CanBed **SCK** -> Driver **SCLK** -- Serial Clock (SCK)\
CanBed **5V** -> Driver **S̅L̅P̅** -- Sleep pin will disable driver when left low, when tied to VCC, driver is always active\
CanBed **GND** -> Driver **GND** -- Ground connection for drivers

### Shared with side
This catagory has pins that the left side drivers share with eachother and ones the right side shares together
#### Left
CanBed **D11** -> Driver **STEP** -- Step pin shared for both the front and rear left drivers\
CanBed **D5** -> Driver **DIR** -- Direction pin shared for both the front and rear left drivers
#### Right
CanBed **D10** -> Driver **STEP** -- Step pin shared for both the front and rear right drivers\
CanBed **D4** -> Driver **DIR** -- Direction pin shared for both the front and rear right drivers

### Direct Conection
CanBed **D6** -> Front Left Driver **SCS** -- Chip Select for Front Left Driver\
CanBed **A0** -> Rear Left Driver **SCS** -- Chip Select for Read Left Driver\
CanBed **D8** -> Front Right Driver **SCS** -- Chip Select for Front Right Driver\
CanBed **D9** -> Rear Right Driver **SCS** -- Chip Select for Rear Right Driver
