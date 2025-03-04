#pragma region -ee- BEGIN:  <doc> File Changes

/*             Documentation of -ee- Changes In THIS File:  Updated 2025-02-03a

      CHANGES:

  - Added:  
      - #pragma region -ee- BEGIN:  Defines
      - #pragma region -ee- BEGIN:  -ee- Tokens And Supporting Variables
            #define T_IMU_READ_DATA '@'      // -ee- Added
            #define T_DISPLAY_MORE_INFO '+'  // -ee- Added
            #define T_SUPER_TOKEN '`'        // -ee- Added
            bool dummyTokenQ = true;         // -ee- We will use dummyTokenQ when the _tokenQ parameter is not used in tokenExtenderQ().
      - #pragma region -ee- BEGIN:  Global Variable Definitions
      -  #pragma region -ee- BEGIN:  Supporting variables for print6Axis_ee()
            int printGyroSkipNum = 101;                      // -ee- Number of read_IMU() cycles to skip before calling print6Axis_ee().  Current value gives about 1 sec between calls.
            #define print6AxisCounterStart 0                 // -ee- Added
            int print6AxisCounter = print6AxisCounterStart;  // -ee- Helper counter of call read_IMU() cycles.
            int numLoops = 15;                               // -ee- Number of loops to use in imSetup() when mpu.CalibrateAccel() and mpu.CalibrateGyro() are called.  Default is 20.
  - Edited:  #define ESTE_DATE "250202a"     // -ee-  version tracking for my work.
  - Added:  /*  -ee- List of all possible token characters and list of token characters officially (not -ee-) in use
  - Added:  /*  -ee- The variable "gyroBalanceQ" is confusingly named because it has Balance in the name but it does more than affect robot balancing.
  - Added:  bool imuDataReadQ = true;  // -ee-  Added here and implemented in io.h readEnvironment.h
  - In moduleList[] initialization
      - Added:  comment:  // -ee- The tokens are shown in the following comments.
  - In moduleActivatedQ[] initialization
      - Added:  comment:  // -ee- Mapping tokens to modules:T  L  D  I  U  G  M  A      //1 = enabled, 0 = disabled.  Currently, only one module is enabled at a time.
  - In #ifdef BITTLE, int angleLimit[][2]
      - Added:  various -ee- notes
  - In initRobot()
      - Added:  #pragma region -ee- BEGIN:  -ee- Customizations

      ENABLED / DISABLED:

  Enabled
    - 

  Disabled
    - 

*/
#pragma endregion   END:  <doc> File Changes


/* BiBoard
    PWM:
                        |--------------------------------
                        |    PWM[0]           PWM[6]    |
                        |    PWM[1]           PWM[7]    |
                        |    PWM[2]           PWM[8]    |
                        |-----------                    |
                        |           |                   |
                        |   ESP32   |  IMU         USB-C|~~~~Tail~~~~
                        |           |                   |
                        |-----------                    |
                        |    PWM[3]           PWM[9]    |
                        |    PWM[4]           PWM[10]   |
                        |    PWM[5]           PWM[11]   |
                        |-------------------------------|

    Pin Name    |   ESP32 Pin   |   Arduino Pin Name    |   Alternative Function
    PWM[0]          GPIO4               4                   GPIO / Ain / Touch
    PWM[1]          GPIO5               5                   GPIO / VSPI SS
    PWM[2]          GPIO18              18                  GPIO / VSPI SCK
    -----------------------------------------------------------------------------
    PWM[3]          GPIO32              32                  GPIO / Ain / Touch
    PWM[4]          GPIO33              33                  GPIO / Ain / Touch
    PWM[5]          GPIO19              19                  GPIO / VSPI MISO
    -----------------------------------------------------------------------------
    PWM[6]          GPIO2               2                   boot pin, DO NOT PUT HIGH WHEN BOOT!
    PWM[7]          GPIO15              15                  GPIO / HSPI SS / Ain Touch
    PWM[8]          GPIO13              13                  built-in LED / GPIO / HSPI MOSI / Ain / Touch
    -----------------------------------------------------------------------------
    PWM[9]          GPIO12              12                  GPIO / HSPI MISO / Ain / Touch
    PWM[10]         GPIO14              14                  GPIO / HSPI SCK / Ain / Touch
    PWM[11]         GPIO27              27                  GPIO / Ain / Touch

    I2C:

    Pin Name    |   ESP32 Pin   |   Arduino Pin Name    |   Alternative Function
    I2C-SCL         GPIO22              22                  Fixed - ICM20600 - Pulled
    I2C-SDA         GPIO21              21                  Fixed = ICM20600 - Pulled

    System default, nothing to declaration!

    Other Peripherals:

    Pin Name    |   ESP32 Pin   |   Arduino Pin Name    |   Alternative Function
    IR_Remote       GPIO23              23                  Fixed - VS1838B IR
    DAC_Out         GPIO25              25                  Fixed - PAM8302
    IMU_Int         GPIO26              26                  Fixed - MPU6050 Interrupt

    System default, nothing to declare!
*/

/* BiBoard2
  IMU_Int     27
  BUZZER      14
  VOLTAGE     4
  RGB LED     15
  GREEN-LED   5
*/

/*  Total DOF            Walking DOF
                   Nybble    Bittle    Cub
   BiBoard  (12)  skip 0~4  skip 0~4    12
   BiBoard2 (16)  skip 0~8  skip 0~8  skip0~4
*/



#pragma region -ee- BEGIN:  <doc> Defines

#define SERIAL_TIMEOUT 10  // 5 may cut off the message
#define SERIAL_TIMEOUT_LONG 150
#ifdef BiBoard_V0_1
#define BOARD "B01"
#elif defined BiBoard_V0_2
#define BOARD "B02"
#else
#define BOARD "B"
#endif
#define DATE "240422"  // YYMMDD
#define ESTE_DATE "250202a"     // -ee-  version tracking for my work.
String SoftwareVersion = "";

#define BIRTHMARK 'x'  // Send '!' token to reset the birthmark in the EEPROM so that the robot will know to restart and reset

#define BT_BLE    // toggle Bluetooth Low Energy (BLE）
#define BT_SSP    // toggle Bluetooth Secure Simple Pairing (BT_SSP)
#define GYRO_PIN  // toggle the Inertia Measurement Unit (IMU), i.e. the gyroscope

#if defined BiBoard_V0_1 || defined BiBoard_V0_2
#define ESP_PWM
#define PWM_NUM 12
#define INTERRUPT_PIN 26  // use pin 2 on Arduino Uno & most boards
#define BUZZER 25
#define IR_PIN 23
#define ANALOG1 34
#define ANALOG2 35
#define ANALOG3 36
#define ANALOG4 39
#define UART_RX 16
#define UART_TX 17

// L:Left-R:Right-F:Front-B:Back---LF, RF, RB, LB
const uint8_t PWM_pin[PWM_NUM] = {
  19, 4, 2, 27,   // head or shoulder roll
  33, 5, 15, 14,  // shoulder pitch
  32, 18, 13, 12  // knee
};
#if defined NYBBLE || defined BITTLE
#define SERVO_FREQ 250
#else  // CUB
#define SERVO_FREQ 250
#endif

#else  // BiBoard2
#define PWM_NUM 16
#define INTERRUPT_PIN 27  // use pin 2 on Arduino Uno & most boards
#define BUZZER 14
#define VOLTAGE 4
#define LOW_VOLTAGE 6.8
#define NEOPIXEL_PIN 15
#define PWM_LED_PIN 5
#define IR_PIN 23
#define TOUCH0 12
#define TOUCH1 13
#define TOUCH2 32
#define TOUCH3 33

//                                headPan, tilt, tailPan, NA
const uint8_t PWM_pin[PWM_NUM] = {
  12, 11, 4, 3,
  13, 10, 5, 2,  // shoulder roll
  14, 9, 6, 1,   // shoulder pitch
  //                                  13,       10,     6,    2,     //shoulder roll
  //                                  14,        9,     5,    1,     //shoulder pitch
  15, 8, 7, 0  // knee
};
// L:Left R:Right F:Front B:Back   LF,        RF,    RB,   LB

#define SERVO_FREQ 240
#endif

#define MAX_READING 4096.0  //to compensate the different voltage level of boards
#define BASE_RANGE 1024.0
double rate = 1.0 * MAX_READING / BASE_RANGE;

#define DOF 16
#if defined NYBBLE || defined BITTLE
#define WALKING_DOF 8
#define GAIT_ARRAY_DOF 8
#else  // CUB
#define WALKING_DOF 12
#define GAIT_ARRAY_DOF 8
#endif

enum ServoModel_t {
  G41 = 0,
  P1S,
  P2K
};

// Tutorial: https://bittle.petoi.com/11-tutorial-on-creating-new-skills
#ifdef NYBBLE
#define MODEL "Nybble"
#define HEAD
#define TAIL
#define X_LEG
#define REGULAR P1S  //G41
#define KNEE P1S     //G41
#include "InstinctNybbleESP.h"

#elif defined BITTLE
#define MODEL "Bittle"
#define HEAD
#define LL_LEG
#define REGULAR P1S
#define KNEE P1S
#include "InstinctBittleESP.h"

#elif defined CUB
#define MODEL "DoF16"
#ifdef BiBoard2
#define HEAD
#define TAIL
#endif
#define LL_LEG
#define REGULAR P1S
#define KNEE P2K
#include "InstinctCubESP.h"
// #define MPU_YAW180
#endif

ServoModel_t servoModelList[] = {
  REGULAR, REGULAR, REGULAR, REGULAR,
  REGULAR, REGULAR, REGULAR, REGULAR,
  REGULAR, REGULAR, REGULAR, REGULAR,
  KNEE, KNEE, KNEE, KNEE
};

bool newBoard = false;

#include <math.h>

/*  -ee- List of all possible token characters and list of token characters officially (not -ee-) in use

    There are 94 standard (using only the Shift key as modifier) visibly typeable ASCII characters on the typical PC keyboard.
      10 digits: 0123456789
      26 lower case letters: abcdefghijklmnopqrstuvwxyz
      26 upper case letters: ABCDEFGHIJKLMNOPQRSTUVWXYZ
      32 special characters: `~!@#$%^&*()-_=+[]\{}|;':",./<>?

    As of the above "File Changes" date:

      There are 40 characters are officially used as tokens:
      (note that currently, tokens ';' and ':' are NOT defined in OpenCat.h but are hard coded in the switch within reaction() [reaction.h] )
        abBcCdfFgGiIjkKlLmMnopqRstTuvVwWxz!?.,;:

      There are 3 characters used by -ee-
        @+`

      There are 51 characters that remain available for use as tokens:
       0123456789ADeEhHJNOPQrSUXyYZ~#$%^&*()-_=[]\{}|'"/<>
*/

// token list
#define T_ABORT 'a'      //abort the calibration values
#define T_BEEP 'b'       //b note1 duration1 note2 duration2 ... e.g. b12 8 14 8 16 8 17 8 19 4 \
                         //bVolume will change the volume of the sound, in scale of 0~10. 0 will mute all sound effect. e.g. b3. \
                         //a single 'b' will toggle all sound on/off
#define T_BEEP_BIN 'B'   //B note1 duration1 note2 duration2 ... e.g. B12 8 14 8 16 8 17 8 19 4 \
                         //a single 'B' will toggle all sound on/off
#define T_CALIBRATE 'c'  //send the robot to calibration posture for attaching legs and fine-tuning the joint offsets. \
                         //c jointIndex1 offset1 jointIndex2 offset2 ... e.g. c0 7 1 -4 2 3 8 5
#define T_COLOR 'C'      //change the eye colors of the RGB ultrasonic sensor \
                         //a single 'C' will cancel the manual eye colors
#define T_REST 'd'

#define T_SERVO_FEEDBACK 'f'            //return the servo's position info if the chip supports feedback. \
                                        //e.g. f8 returns the 8th joint's position. A single 'f' returns all the joints' position
#define T_SERVO_FOLLOW 'F'              //make the other legs follow the moved legs
#define T_GYRO_FINENESS 'g'             //adjust the finess of gyroscope adjustment to accelerate motion
#define T_GYRO_BALANCE 'G'              //toggle on/off the gyro adjustment
#define T_INDEXED_SIMULTANEOUS_ASC 'i'  //i jointIndex1 jointAngle1 jointIndex2 jointAngle2 ... e.g. i0 70 8 -20 9 -20 \
                                        //a single 'i' will free the head joints if it were previously manually controlled.
#define T_INDEXED_SIMULTANEOUS_BIN 'I'  // I jointIndex1 jointAngle1 jointIndex2 jointAngle2 ... e.g. I0 70 8 -20 9 -20
#define T_JOINTS 'j'                    // A single "j" returns all angles. "j Index" prints the joint's angle. e.g. "j 8" or "j11".
#define T_SKILL 'k'
#define T_SKILL_DATA 'K'
#define T_SLOPE 'l'                   // inverse the slope of the adjustment function
#define T_LISTED_BIN 'L'              // a list of the DOFx joint angles: angle0 angle1 angle2 ... angle15
#define T_INDEXED_SEQUENTIAL_ASC 'm'  // m jointIndex1 jointAngle1 jointIndex2 jointAngle2 ... e.g. m0 70 0 -70 8 -20 9 -20
#define T_INDEXED_SEQUENTIAL_BIN 'M'  // M jointIndex1 jointAngle1 jointIndex2 jointAngle2 ... e.g. M0 70 0 -70 8 -20 9 -20
#define T_NAME 'n'                    // customize the Bluetooth device's broadcast name. e.g. nMyDog will name the device as "MyDog" \
                                      // it takes effect the next time the board boosup. it won't interrupt the current connecton.
#define T_MELODY 'o'
#define T_PAUSE 'p'
#define T_TASK_QUEUE 'q'
#define T_SAVE 's'
#define T_TILT 't'
#define T_TEMP 'T'  // call the last skill data received from the serial port
#define T_MEOW 'u'
#define T_PRINT_GYRO 'v'            // print Gyro data once
#define T_VERBOSELY_PRINT_GYRO 'V'  // toggle verbosely print Gyro data
#define T_SERVO_MICROSECOND 'w'     // PWM width modulation
#define T_XLEG 'x'
#define T_RANDOM_MIND 'z'  // toggle random behaviors

#define T_READ 'R'        // read pin     R
#define T_WRITE 'W'       // write pin                      W
// -ee- next two items are not tokens
#define TYPE_ANALOG 'a'   //            Ra(analog read)   Wa(analog write)
#define TYPE_DIGITAL 'd'  //            Rd(digital read)  Wd(digital write)

#define T_RESET '!'
#define T_QUERY '?'
#define T_ACCELERATE '.'
#define T_DECELERATE ','

#define EXTENSION 'X'
#define EXTENSION_DOUBLE_TOUCH 'T'
#define EXTENSION_DOUBLE_LIGHT 'L'
#define EXTENSION_DOUBLE_IR_DISTANCE 'D'
#define EXTENSION_PIR 'I'
#define EXTENSION_ULTRASONIC 'U'
#define EXTENSION_GESTURE 'G'
#define EXTENSION_CAMERA_MU3 'M'
#define EXTENSION_VOICE 'A'

#pragma endregion   END:  <doc> Defines

#pragma region -ee- BEGIN:  <merge> -ee- Tokens And Supporting Variables
#define T_IMU_READ_DATA '@'      // -ee- Added
#define T_DISPLAY_MORE_INFO '+'  // -ee- Added
#define T_SUPER_TOKEN '`'        // -ee- Added
bool dummyTokenQ = true;         // -ee- We will use dummyTokenQ when the _tokenQ parameter is not used in tokenExtenderQ().
#pragma endregion   END:  <merge> -ee- Tokens And Supporting Variables

#pragma region -ee- BEGIN:  <doc> Global Variable Definitions

// bool updated[10];
float degPerRad = 180 / M_PI;
float radPerDeg = M_PI / 180;

// control related variables
#define IDLE_TIME 3000
long idleTimer = 0;
#define CHECK_BATTERY_PERIOD 10000  // every 10 seconds. 60 mins -> 3600 seconds
int uptime = -1;
int frame = 0;
int tStep = 1;
long loopTimer;
byte fps = 0;

char token;
char lastToken;
char lowerToken;
#define CMD_LEN 10
char *lastCmd = new char[CMD_LEN + 1];  // the last char must be '\0' for safe so CMD_LEN+1 elements are required
int cmdLen = 0;
byte newCmdIdx = 0;
int8_t periodGlobal = 0;
#define BUFF_LEN 2507  // 1524 =125*20+7=2507
char *newCmd = new char[BUFF_LEN + 1];
int spaceAfterStoringData = BUFF_LEN;
int serialTimeout;
int lastVoltage;
char terminator;
// int serialTimeout;
long lastSerialTime = 0;

bool interruptedDuringBehavior = false;
bool fineAdjust = true;

/*  -ee- The variable "gyroBalanceQ" is confusingly named because it has Balance in the name but it does more than affect robot balancing.
    Setting it to false causes [in io.h, readEnvironment() ] the IMU data not to be read.
    With gyroBalanceQ = false, you also cannot get any IMU data for navigation!
    IMO, this should just turn on/off the robot balancing but should not stop getting IMU data for other purposes.
    There should be a separate bool for enabling/disabling reading IMU data.
*/
bool gyroBalanceQ = true;  
bool imuDataReadQ = true;  // -ee-  Added here and implemented in io.h readEnvironment.h

bool printGyro = false;
bool autoSwitch = false;
bool walkingQ = false;
bool manualHeadQ = false;
bool nonHeadJointQ = false;
bool hardServoQ = true;
bool manualEyeColorQ = false;

// bool keepDirectionQ = true;
#define HEAD_GROUP_LEN 4  // used for controlling head pan, tilt, tail, and other joints independent from walking
int targetHead[HEAD_GROUP_LEN];

bool imuUpdated;
int exceptions = 0;
byte transformSpeed = 2;
float protectiveShift;  // reduce the wearing of the potentiometer

int8_t moduleList[] = {
                                 // -ee- The tokens are shown in the following comments.
  EXTENSION_DOUBLE_LIGHT,        // L
  EXTENSION_DOUBLE_IR_DISTANCE,  // D
  EXTENSION_PIR,                 // I
  EXTENSION_ULTRASONIC,          // U
  EXTENSION_GESTURE,             // G
  EXTENSION_CAMERA_MU3,          // M
  EXTENSION_VOICE                // A
};
// Mapping tokens to modules:T  L  D  I  U  G  M  A      // -ee- 1 = enabled, 0 = disabled.  Currently, only one module is enabled at a time.
bool moduleActivatedQ[] = { 0, 0, 0, 0, 0, 0, 0, 0 };   //default is { 0, 0, 0, 0, 0, 0, 0, 1 }
bool initialBoot = true;
bool safeRest = true;
bool soundState;
byte buzzerVolume;
float amplifierFactor = 100.0;  //to fit the actual amplifier range of BiBoard

int delayLong = 20;
int delayMid = 8;
int delayException = 5;
int delayShort = 3;
int delayStep = 1;
int delayPrevious;
int runDelay = delayMid;

  #pragma region -ee- BEGIN:  <merge> Supporting variables for print6Axis_ee()

/*  -ee- 
*   Measured the # of millisec between print6Axis_ee() calls for different printGyroSkipNum values
*     @ printGyroSkipNum = 500  >>>  4990.0 millisec between calls.
*     @ printGyroSkipNum = 100  >>>  998.0 millisec between calls.
*     @ printGyroSkipNum = 50   >>>  499.0 millisec between calls.
*     @ printGyroSkipNum = 25   >>>  249.5 millisec between calls.
*/
int printGyroSkipNum = 101;                      // -ee- Number of read_IMU() cycles to skip before calling print6Axis_ee().  Current value gives about 1 sec between calls.
#define print6AxisCounterStart 0                 // -ee- Added
int print6AxisCounter = print6AxisCounterStart;  // -ee- Helper counter of call read_IMU() cycles.
int numLoops = 15;                               // -ee- Number of loops to use in imSetup() when mpu.CalibrateAccel() and mpu.CalibrateGyro() are called.  Default is 20.
  #pragma endregion   END:  <merge> Supporting variables for print6Axis_ee()

#pragma endregion   END:  <doc> Global Variable Definitions


#ifdef NYBBLE
int8_t middleShift[] = { 0, 15, 0, 0,
                         -45, -45, -45, -45,
                         10, 10, -10, -10,
                         -30, -30, 30, 30 };
#elif defined BITTLE
int8_t middleShift[] = { 0, 15, 0, 0,
                         -45, -45, -45, -45,
                         55, 55, -55, -55,
                         -55, -55, -55, -55 };

#else  // CUB
int8_t middleShift[] = { 0, 15, 0, 0,
                         -45, -45, -45, -45,
                         55, 55, -55, -55,
                         -45, -45, -45, -45 };
#endif

// #define INVERSE_SERVO_DIRECTION
#ifdef CUB
int8_t rotationDirection[] = { 1, -1, 1, 1,
                               1, -1, 1, -1,
                               1, -1, -1, 1,
                               1, -1, -1, 1 };
int angleLimit[][2] = {
  { -120, 120 },
  { -30, 80 },
  { -120, 120 },
  { -120, 120 },
  { -90, 60 },
  { -90, 60 },
  { -90, 90 },
  { -90, 90 },
  { -180, 120 },
  { -180, 120 },
  { -80, 200 },
  { -80, 200 },
  { -66, 100 },
  { -66, 100 },
  { -66, 100 },
  { -66, 100 },
};
#else
int8_t rotationDirection[] = { 1, -1, 1, 1,
                               1, -1, 1, -1,
                               1, -1, -1, 1,
                               -1, 1, 1, -1 };
#ifdef BITTLE
int angleLimit[][2] = {
  { -120, 120 },     // -ee- Joint Index = 0
  { -85, 85 },
  { -120, 120 },
  { -120, 120 },
  { -90, 60 },
  { -90, 60 },
  { -90, 90 },
  { -90, 90 },

  { -200, 80 },     // -ee- Joint Index = 8   LeftFront   Hip
  { -200, 80 },     // -ee- Joint Index = 9   RightFront  Hip
  { -80, 200 },     // -ee- Joint Index = 10  RightBack   Hip
  { -80, 200 },     // -ee- Joint Index = 11  LeftBack    Hip

  { -80, 200 },     // -ee- Joint Index = 12  LeftFront   Knee
  { -80, 200 },     // -ee- Joint Index = 13  RightFront  Knee
  { -70, 200 },     // -ee- Joint Index = 14  RightBack   Knee
  { -80, 200 },     // -ee- Joint Index = 15  LeftBack    Knee
};
#else
int angleLimit[][2] = {
  { -120, 120 },
  { -85, 50 },
  { -120, 120 },
  { -120, 120 },
  { -90, 60 },
  { -90, 60 },
  { -90, 90 },
  { -90, 90 },
  { -200, 80 },
  { -200, 80 },
  { -80, 200 },
  { -80, 200 },
  { -80, 80 },
  { -80, 80 },
  { -70, 80 },
  { -80, 80 },
};
#endif
#endif

#ifdef X_LEG
int currentAng[DOF] = { -30, -80, -45, 0,
                        0, 0, 0, 0,
                        75, 75, -75, -75,
                        -55, -55, 55, 55 };
int previousAng[DOF] = { -30, -80, -45, 0,
                         0, 0, 0, 0,
                         75, 75, -75, -75,
                         -55, -55, 55, 55 };
#else
int currentAng[DOF] = { -30, -80, -45, 0,
                        0, 0, 0, 0,
                        75, 75, 75, 75,
                        -55, -55, -55, -55 };
int previousAng[DOF] = { -30, -80, -45, 0,
                         0, 0, 0, 0,
                         75, 75, 75, 75,
                         -55, -55, -55, -55 };
#endif
int zeroPosition[DOF] = {};
int calibratedZeroPosition[DOF] = {};

int8_t servoCalib[DOF] = { 0, 0, 0, 0,
                           0, 0, 0, 0,
                           0, 0, 0, 0,
                           0, 0, 0, 0 };

int16_t imuOffset[9] = { 0, 0, 0,
                         0, 0, 0,
                         0, 0, 0 };

float expectedRollPitch[2];
float RollPitchDeviation[2];
float currentAdjust[DOF] = {};
int slope = 1;

#include "tools.h"
#include "QList/QList.h"
#include "taskQueue.h"

#include "sound.h"
#include "I2cEEPROM.h"
#ifdef BT_BLE
#include "bleUart.h"
#endif
#ifdef GYRO_PIN
#include "imu.h"
#endif
#ifdef IR_PIN
#include "infrared.h"
#endif
#include "espServo.h"
#include "motion.h"
#include "randomMind.h"
#include "io.h"
#include "moduleManager.h"

#include "skill.h"
#ifdef NEOPIXEL_PIN
#include "led.h"
#endif
#include "reaction.h"
#include "qualityAssurance.h"

void initRobot() {
  beep(20);
  Wire.begin();
  SoftwareVersion = SoftwareVersion + BOARD + "_" + DATE;
  PTL('k');
  PTLF("Flush the serial buffer...");
  PTL("\n* Start *");
  printToAllPorts(MODEL);
  PTF("Software version: ");
  printToAllPorts(SoftwareVersion);
  soundState = i2c_eeprom_read_byte(EEPROM_BOOTUP_SOUND_STATE);
  buzzerVolume = max(byte(0), min(byte(10), i2c_eeprom_read_byte(EEPROM_BUZZER_VOLUME)));
  PTF("Buzzer volume: ");
  PT(buzzerVolume);
  PTL("/10");
  i2cDetect();
  i2cEepromSetup();
#ifdef GYRO_PIN
  imuSetup();
#endif
#ifdef BT_BLE
  bleSetup();
#endif
#ifdef BT_SSP
  blueSspSetup();
#endif
  servoSetup();
  lastCmd[0] = '\0';
  newCmd[0] = '\0';
  skill = new Skill();
  skillList = new SkillList();
  for (byte i = 0; i < randomMindListLength; i++) {
    randomBase += choiceWeight[i];
  }

#ifdef NEOPIXEL_PIN
  ledSetup();
#endif
#ifdef PWM_LED_PIN
  pinMode(PWM_LED_PIN, OUTPUT);
#endif
#ifdef VOLTAGE
  while (lowBattery())
    ;
#endif

#ifdef IR_PIN
  irrecv.enableIRIn();
#endif

  QA();
  i2c_eeprom_write_byte(EEPROM_BIRTHMARK_ADDRESS, BIRTHMARK);  // finish the test and mark the board as initialized

  tQueue = new TaskQueue();

  initModuleManager();

  //  if (exceptions) {// Make the robot enter joint calibration state (different from initialization) if it is upside down.
  //    strcpy(newCmd, "calib");
  //    exceptions = 0;
  //  }
  //  else {// Otherwise start up normally
  //    strcpy(newCmd, "rest");
  //    token = 'd';
  //    newCmdIdx = 6;
  //  }
  //  loadBySkillName(newCmd);
  //
  allCalibratedPWM(currentAng);  // soft boot for servos
  delay(500);
#ifdef GYRO_PIN
  // read_IMU();  //ypr is slow when starting up. leave enough time between IMU initialization and this reading
  tQueue->addTask((exceptions) ? T_CALIBRATE : T_REST, "");
#endif
#if defined DOUBLE_LIGHT || defined DOUBLE_TOUCH || defined DOUBLE_INFRARED_DISTANCE || defined ULTRASONIC
  if (moduleActivatedQfunction(EXTENSION_DOUBLE_LIGHT) || moduleActivatedQfunction(EXTENSION_DOUBLE_LIGHT) || moduleActivatedQfunction(EXTENSION_DOUBLE_LIGHT)) {
    tQueue->addTask(T_SKILL, "sit", 500);
  } else {
    tQueue->addTask(T_REST, "");
  }
#endif

  PTL("Ready!");
  idleTimer = millis();
  beep(24, 50);

  #pragma region -ee- BEGIN:  -ee- Customizations

  PTL("idleTimer = " + String(idleTimer) );
//  tQueue->addTask('k', "up");  // -ee- Start robot in "stand-up" posture.
  imuCalibrate_ee();  // -ee- Added to force IMU calibration.
  delay(2000);
  Serial2.println("XAd");   // -ee- Turn off voice module recognition.  Doesn't work though...

  /* -ee- Here you can add any tasks to be run during the setup() */

  // These tasks should always be performed.
  tQueue->addTask('G', "0");   // Start with the IMU disabled
  tQueue->addTask('X', "-1");  // -ee- disable all modules.  Correctly changes the Extension array but the voice module still "listens" and tries to respond.
//  tQueue->addTask('XAd', "");  // Turn off voice module recognition  - Still the voice module still "listens" and tries to respond.
//  tQueue->addTask('X', "Ad");  // Turn off voice module recognition  - Still the voice module still "listens" and tries to respond.

  // These tasks should be enabled only when needed.
//  tQueue->addTask('n', "BittleA2");  // -ee- enable to automatically set the Bluetooth name.

//  tQueue->addTask('X', "Ad~");  // Need to use the '~' terminator.   but this does not work.  - Still the voice module still "listens" and tries to respond.
//  tQueue->addTask('k', "up");  // -ee- Start robot in "stand-up" posture.

  #pragma endregion   END:  -ee- Customizations
}
