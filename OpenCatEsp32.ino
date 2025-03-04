// This is Project https://github.com/este-este/OpenCatEsp32_este, a fork of https://github.com/PetoiCamp/OpenCatEsp32.
// See "/src/este.h" in the project folder for project documentation and a list of files that were modified from the original source code.

#pragma region -ee- BEGIN:  <doc> File Changes

/*             Documentation of -ee- Changes In THIS File:  Updated 2025-02-03a

      CHANGES:

  - Added:  // -ee- Exercise caution with AUTO_INIT...
  - Added:  // -ee- VOICE needs to be defined in order to send the "XAd" command to stop the voice module from listening.
  - Added:  #include "src/este.h"  // -ee- Added       < placed after #include "src/OpenCat.h" >

      ENABLED / DISABLED:

  Enabled
    - #define VOICE

  Disabled
    - 

*/
#pragma endregion   END:  <doc> File Changes


// modify the model and board definitions
//***********************
#define BITTLE  // Petoi 9 DOF robot dog: 1 on head + 8 on leg
// #define NYBBLE  //Petoi 11 DOF robot cat: 2 on head + 1 on tail + 8 on leg
// #define CUB

// #define BiBoard_V0_1  //ESP32 Board with 12 channels of built-in PWM for joints
#define BiBoard_V0_2
// #define BiBoard2  //ESP32 Board with 16 channels of PCA9685 PWM for joints
//***********************

// Send '!' token to reset the birthmark in the EEPROM so that the robot will restart to reset
//#define AUTO_INIT  //activate it to automatically reset joint and imu calibration without prompts    // -ee- Exercise caution with AUTO_INIT...

// you can also activate the following modes (they will diable the gyro to save programming space)
// allowed combinations: RANDOM_MIND + ULTRASONIC, RANDOM_MIND, ULTRASONIC, VOICE, CAMERA
#define VOICE  // Petoi Grove voice module    // -ee- VOICE needs to be defined in order to send the "XAd" command to stop the voice module from listening.
#define ULTRASONIC
#define PIR                       //for PIR (Passive Infrared) sensor
#define DOUBLE_TOUCH              //for double touch sensor
#define DOUBLE_LIGHT              //for double light sensor
#define DOUBLE_INFRARED_DISTANCE  //for double distance sensor
#define GESTURE                   //for Gesture module
#define CAMERA                    //for Mu Vision camera
// You need to install https://github.com/mu-opensource/MuVisionSensor3 as a zip library in Arduino IDE.
// Set the four dial switches on the camera as **v ^ v v** (the second switch dialed up to I2C) and connect the camera module to the I2C grove on NyBoard.
// The battery should be turned on to drive the servos.
//  
// You can use these 3D printed structures to attach the camera module.
// https://github.com/PetoiCamp/NonCodeFiles/blob/master/stl/MuIntelligentCamera_mount.stl
// https://github.com/PetoiCamp/NonCodeFiles/blob/master/stl/bone.stl
// After uploading the code, you may need to press the reset buttons on the module and then the NyBoard.
// The tracking demo works the best with a yellow tennis ball or some other round objects. Demo: https://www.youtube.com/watch?v=CxGI-MzCGWM
#include "src/OpenCat.h"
#include "src/este.h"  // -ee- Added

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);  // USB serial
  Serial.setTimeout(SERIAL_TIMEOUT);
  //  Serial1.begin(115200); //second serial port
  while (Serial.available() && Serial.read())
    ;  // empty buffer
  initRobot();
}

void loop() {
#ifdef VOLTAGE
  lowBattery();
#endif
  //  //—self-initiative
  //  if (autoSwitch) { //the switch can be toggled on/off by the 'z' token
  //    randomMind();//allow the robot to auto rest and do random stuff in randomMind.h
  //    powerSaver(POWER_SAVER);//make the robot rest after a certain period, the unit is seconds
  //
  //  }
  //  //— read environment sensors (low level)
  readEnvironment();
  //  //— special behaviors based on sensor events
  dealWithExceptions();  // low battery, fall over, lifted, etc.
  if (!tQueue->cleared()) {
    tQueue->popTask();
  } else {
    readSignal();
    //  readHuman();
  }
  //  //— generate behavior by fusing all sensors and instruction
  //  decision();

  //  //— action
  //  //playSound();
#ifdef NEOPIXEL_PIN
  playLight();
#endif
  reaction();
}
