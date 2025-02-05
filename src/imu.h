#pragma region -ee- BEGIN:  <doc> File Changes

/*             Documentation of -ee- Changes In THIS File:  Updated 2025-02-03a

      CHANGES:

  - In section // orientation/motion vars
      - Added:  // -ee- no, in degrees, not radians since #define OUTPUT_READABLE_YAWPITCHROLL is enabled.
  - In print6Axis()
      - Added:
      -   PT(millis() / 1000);        // -ee- Added.     //TODO add these two lines to print6Axis_ee, maybe in the #define PRINT_6_AXIS_TESTING section
      -   PT('\t');                   // -ee- Added.
  - Added:  // -ee- Forward declare printToAllPorts().
      Then added:  template<typename T> void printToAllPorts(T text);  // -ee- Added
  - Added:  #pragma region -ee- BEGIN:   print6Axis_ee()
  - In #ifdef OUTPUT_READABLE_YAWPITCHROLL
      - Added:  // -ee- degrees, not radians since #define OUTPUT_READABLE_YAWPITCHROLL is enabled.
  - In read_IMU()
      - Added:  /*  -ee- Comment out original code.
          Then added:  #pragma region -ee- BEGIN:  <merge> Using print6Axis_ee()
  - Added:  #pragma region -ee- BEGIN:  imuCalibrate_ee()

      ENABLED / DISABLED:

  Enabled
    - 

  Disabled
    - 

*/
#pragma endregion   END:  <doc> File Changes


// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v6.12)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2019-07-10 - Uses the new version of the DMP Firmware V6.12
//                 - Note: I believe the Teapot demo is broken with this versin as
//                 - the fifo buffer structure has changed
//      2016-04-18 - Eliminated a potential infinite loop
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2012 Jeff Rowberg

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "mpu6050/src/I2Cdev.h"

#include "mpu6050/src/MPU6050_6Axis_MotionApps_V6_12.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
   ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
   ========================================================================= */

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL
/* REALACCEL represents the acceleration related to the sensor.
   Even if the robot is not moving, it will be large if the sensor is tilted.
   Because gravity has a component in that tilted direction.
   It's useful to use aaReal.z to determine if the sensor is flipped up-side-down.
*/

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
#define OUTPUT_READABLE_WORLDACCEL
/* WORLDACCEL represents the acceleration related to the world reference.
   It will be close to zero as long as the robot is not moved.
   It's useful to detect the wobbling about the sensor's original position.
*/

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 gy;       // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector. unit is radian   // -ee- no, in degrees, not radians since #define OUTPUT_READABLE_YAWPITCHROLL is enabled.
int16_t *xyzReal[3] = { &aaReal.x, &aaReal.y, &aaReal.z };
int16_t previous_xyzReal[3];
float previous_ypr[3];
int8_t yprTilt[3];


#define ARX *xyzReal[0]
#define ARY *xyzReal[1]
#define ARZ *xyzReal[2]
#define AWX aaWorld.x
#define AWY aaWorld.y
#define AWZ aaWorld.z
int thresX, thresY, thresZ;
#define IMU_SKIP 1
#define IMU_SKIP_MORE 23  //use prime number to avoid repeatly skipping the same joint
byte imuSkip = IMU_SKIP;

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// The REALACCEL numbers are calculated with respect to the orientation of the sensor itself, so that if it is flat and you move it straight up, the "Z" accel will change, but if you flip it up on one side and move it in the new relative "up" direction (along the sensor's Z axis), it will still register acceleration on the Z axis. Essentially, it is sensor-oriented acceleration which removes the effects of gravity and any non-flat/level orientation.

// The WORLDACCEL numbers are calculated to ignore orientation. Moving it straight up while flat will look the same as the REALACCEL numbers, but if you then flip it upside-down and do the exact same movement ("up" with respect to you), you'll get exactly the same numbers as before, even though the sensor itself is upside-down.

#define READ_ACCELERATION
void print6Axis() {
  PT(millis() / 1000);        // -ee- Added.     //TODO add these two lines to print6Axis_ee, maybe in the #define PRINT_6_AXIS_TESTING section
  PT('\t');                   // -ee- Added.
  PT_FMT(ypr[0], 2);
  PT('\t');
  PT_FMT(ypr[1], 2);
  PT('\t');
  PT_FMT(ypr[2], 2);
#ifdef READ_ACCELERATION
  PT("\t");
  // PT(aaWorld.x);
  // PT("\t");
  // PT(aaWorld.y);
  PT(*xyzReal[0]);  //x is along the longer direction of the robot
  PT("\t");
  PT(*xyzReal[1]);
  PT('\t');
  PT(*xyzReal[2]);
  PT("\t");
  PT(aaWorld.z);
#endif
  PTL();
}

// -ee- Added Forward declare printToAllPorts() since it is used in print6Axis_ee() below.
template<typename T> void printToAllPorts(T text);  // -ee- Added

#pragma region -ee- BEGIN:   print6Axis_ee()

//#define PRINT_6_AXIS_TESTING                             // -ee- Default is off, unless testing.
//#define PRINT_6_AXIS___READ_ALL___YPR_PLUS_ACCELERATION  // -ee- Default is off.
//#define PRINT_6_AXIS___SHOW_COLUMN_TITLES                // -ee- Default is off.

void print6Axis_ee(bool createPacketQ)  // -ee- Added
{
  /*
      Purpose:  To create a packet of IMU data for consumption by a robot controller computer.
      Author:   este este
  */

  String timeStamp = String(millis() );
  String extraPadding = "";

  String startPacket = "";  //default is no packet
  String endPacket = "";    //default is no packet

  if (createPacketQ)
  {
    startPacket = "[";
    endPacket = "]";
  }

  // -ee- uncomment next line except for testing.
//  timeStamp = "1234567890";   //max value = "4294967295"

#ifdef  PRINT_6_AXIS_TESTING

  PT('\t');
  PT(ypr[0]);
  PT("\t");
  PT(ypr[1]);
  PT('\t');
  PT(ypr[2]);

  PT('\t');
  PT(*xyzReal[0]);
  PT("\t");
  PT(*xyzReal[1]);
  PT('\t');
  PT(*xyzReal[2]);
  PT("\t");
  PT(aaWorld.z);

  PTL();
  return;

#endif  //PRINT_6_AXIS_TESTING

  if (timeStamp.length() >= 9)
  {
    extraPadding = "\t";
  }

#ifdef PRINT_6_AXIS___SHOW_COLUMN_TITLES
  //print column titles
  printToAllPorts("\t\ttime" + 
    extraPadding +                  //Extra tab before yaw when timeStamp is >= 9 chars
    "\tyaw\tpitch\troll"
    #ifdef  PRINT_6_AXIS___READ_ALL___YPR_PLUS_ACCELERATION
    "\tXreal\tYreal\tZreal\tZworld"
    #endif  //PRINT_6_AXIS___READ_ALL___YPR_PLUS_ACCELERATION
  );
#endif  //PRINT_6_AXIS___SHOW_COLUMN_TITLES

//TODO:  -ee- Deprecated but KEEP.  It turns out that the following conversions are not actually needed.

    /*  -ee- 
        Convert ypr data from floating point values to decimal strings with fixed decimal places.
        Maximum width of each datum is 8 with a layout of [-XXX.XX\0]
    */

  //yaw:  rotation about z-axis (head left / right)
  char ypr0[8];                 // -ee- char array to hold formatted value.
  dtostrf(ypr[0], 3, 2, ypr0);  // -ee- format the value.

  //pitch:  rotation about y-axis (head up / down)
  char ypr1[8];
  dtostrf(ypr[1], 3, 2, ypr1);

  //roll:  rotation about x-axis (side up / down)
  char ypr2[8];
  dtostrf(ypr[2], 3, 2, ypr2);

  //TODO:  -ee- Deprecated but KEEP.  It turns out that the following conversions are not actually needed.

      /*  -ee- 
        Convert xyzReal and aaWorld data from floating point values to decimal strings with fixed decimal places.
        Maximum width of each datum is 11 with a layout of [-XXXXX.XX\0]
    */

        /*  -ee- 
            xyzReal is acceleration relative to the orientation of the BiBoard (well, the IMU sensor on the BiBoard actually).
            aaWorld is acceleration relative to the outside world.
                With the robot "right side up:
                      mMving the robot up is positive acceleration in both the "Real" frame and in the "World" frame.
                With the robot "upside up":
                      Moving the robot up is negative acceleration in the "Real" frame but positive acceleration in the "World" frame.
        */

//Real frame (gravity ignored) acceleration along the x-axis (along the longer direction of the robot).
  char xyzReal0[11];                 // -ee- char array to hold formatted value.
  dtostrf(*xyzReal[0], 5, 2, xyzReal0);  // -ee- format the value.

// -ee- Real frame (gravity ignored) acceleration along the y-axis (along the shorter direction of the robot).
  char xyzReal1[11];
  dtostrf(*xyzReal[1], 5, 2, xyzReal1);

// -ee- Real frame (gravity ignored) acceleration along the z-axis (up and perpendicular to the xy plane).
  char xyzReal2[11];
  dtostrf(*xyzReal[2], 5, 2, xyzReal2);

// -ee- World-frame (gravity corrected) acceleration along the z-axis.
  char aaWorldZ[11];
  dtostrf(aaWorld.z, 5, 2, aaWorldZ);


  /*  Create packet:
        @   Start the packet
              "\t" is used as a spacer but is not part of the packet.
              "[" starts the packet and "I-H" or "I-A" identifies the packet.  The character sequence is followed by a tab.
        @   The packet
              millis() followed by IMU data, all delimited by tabs
        @   End the packet
              "]" ends the packet.

        Example:	"	[I-H	4294967295	1.00	-3.51	2.13	]"
  */

//Using millis() next, which is the time since boot up in milliseconds.  
//Value is an unsigned long (4 bytes = 32 bits) so the max value = 4,294,967,295   (10 digits since there are no commas!)

#ifndef PRINT_6_AXIS___READ_ALL___YPR_PLUS_ACCELERATION
  char packet[45];       //1 + 4 + (10 + 1) + ( (1 + 8) * 3 ) + 1 + 1 = 45 characters, max, in the packet.
  sprintf(packet, 
    "\t%s"                     //Start the packet.
    "I-H"                       //Identify the packet.  I-H = "IMU, Heading" values only (ypr only).
    "\t%s\t%s\t%s\t%s"          //Contents of the packet with each datum delimited by tabs.
    "\t%s",                     //End the packet.
//    ypr0, ypr1, ypr2                                                    // -ee- This is deprecated.
startPacket,
    timeStamp,
        String(ypr[0]), String(ypr[1]), String(ypr[2]),      // -ee- This  works without the deprecated calculations.
endPacket
  );
#endif  // !PRINT_6_AXIS___READ_ALL___YPR_PLUS_ACCELERATION


#ifdef PRINT_6_AXIS___READ_ALL___YPR_PLUS_ACCELERATION
  char packet[81];       //1 + 4 + (10 + 1) + ( (1 + 8) * 7 ) + 1 + 1 = 81 characters, max, in the packet.
  sprintf(packet, 
    "\t%s"                              //Start the packet.
    "I-A"                               //Identify the packet.  I-A = "IMU, All" values (ypr plus acceleration).
    "\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s"  //Contents of the packet with each datum delimited by tabs.
    "\t%s",                             //End the packet.
//    ypr0, ypr1, ypr2, xyzReal0, xyzReal1, xyzReal2, aaWorldZ                      // -ee- This is deprecated.
startPacket,
    timeStamp,
        String(ypr[0]), String(ypr[1]), String(ypr[2]),              // -ee- This  works without the deprecated calculations.
        String(*xyzReal[0]), String(*xyzReal[1]), String(*xyzReal[2]), String(aaWorld.z),
endPacket
  );
#endif  // PRINT_6_AXIS___READ_ALL___YPR_PLUS_ACCELERATION

  printToAllPorts(packet);
//  printToAllPorts("");    // -ee- send blank line.

}
#pragma endregion   END:   print6Axis_ee()


void print6AxisMacro() {
#ifdef OUTPUT_READABLE_QUATERNION
  // display quaternion values in easy matrix form: w x y z
  PT("quat\t");
  PT(q.w);
  PT("\t");
  PT(q.x);
  PT("\t");
  PT(q.y);
  PT("\t");
  PT(q.z);
  PT("\t");
#endif

#ifdef OUTPUT_READABLE_EULER
  // display Euler angles in degrees
  PT("euler\t");
  PT(euler[0] * 180 / M_PI);
  PT("\t");
  PT(euler[1] * 180 / M_PI);
  PT("\t");
  PT(euler[2] * 180 / M_PI);
  PT("\t");
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
  // display angles in degrees
    // -ee- degrees, not radians since #define OUTPUT_READABLE_YAWPITCHROLL is enabled.
  PT("ypr\t");
  PT(ypr[0]);
  PT("\t");
  PT(ypr[1]);
  PT("\t");
  PT(ypr[2]);
  PT("\t");
  /*
    mpu.dmpGetAccel(&aa, fifoBuffer);
    PT("\tRaw Accl XYZ\t");
    PT(aa.x);
    PT("\t");
    PT(aa.y);
    PT("\t");
    PT(aa.z);
    mpu.dmpGetGyro(&gy, fifoBuffer);
    PT("\tRaw Gyro XYZ\t");
    PT(gy.x);
    PT("\t");
    PT(gy.y);
    PT("\t");
    PT(gy.z);
    PT("\t");
  */
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
  // display initial world-frame acceleration, adjusted to remove gravity
  // and rotated based on known orientation from quaternion
  PT("aworld\t");
  PT(aaWorld.x);
  PT("\t");
  PT(aaWorld.y);
  PT("\t");
  PT(aaWorld.z);
  PT("\t");
#endif

#ifdef OUTPUT_READABLE_REALACCEL
  // display real acceleration, adjusted to remove gravity
  PT("areal\t");
  PT(aaReal.x);
  PT("\t");
  PT(aaReal.y);
  PT("\t");
#endif
  PT("areal.z\t");
  PT(aaReal.z);  //becomes negative when flipped
  PT("\t");

  PTL();
}

bool read_IMU() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

    for (byte i = 0; i < 3; i++) {  //no need to flip yaw
      ypr[i] *= degPerRad;
#ifdef BiBoard_V0_1
      ypr[i] = -ypr[i];
      if (i != 2)
        *xyzReal[i] = -*xyzReal[i];
#endif
    }
    if (printGyro)
      /*  -ee- Comment out original code.
      print6Axis();
      */

      #pragma region -ee- BEGIN:  <merge> Using print6Axis_ee()

      {
        if (print6AxisCounter % printGyroSkipNum == 0)  // check the interval.
          {
            print6Axis_ee(true);  // print IMU data.
            print6AxisCounter = print6AxisCounterStart;  // reset the counter.
          }
        print6AxisCounter++;  // increment the counter.
      }
      #pragma endregion   END:  <merge> Using print6Axis_ee()

    // exceptions = aaReal.z < 0 && fabs(ypr[2]) > 85;  //the second condition is used to filter out some noise

    // Acceleration Real
    //      ^ head
    //        ^ x+
    //        |
    //  y+ <------ y-
    //        |
    //        | x-
    // if (AWZ < -8500 && AWZ > -8600)
    //   exceptions = -1;  //dropping
    // else
    if (ARZ < 0 && fabs(ypr[2]) > 85)  //  exceptions = aaReal.z < 0;
      exceptions = -2;                 // flipped
    // else if ((abs(ARX - previous_xyzReal[0]) > 6000 && abs(ARX) > thresX || abs(ARY - previous_xyzReal[1]) > 5000 && abs(ARY) > thresY))
    //   exceptions = -3;
    // else if (  //keepDirectionQ &&
    //   abs(previous_ypr[0] - ypr[0]) > 15 && abs(abs(ypr[0] - previous_ypr[0]) - 360) > 15)
    //   exceptions = -4;
    else exceptions = 0;
    //however, its change is very slow.
    for (byte m = 0; m < 3; m++) {
      previous_xyzReal[m] = *xyzReal[m];
      if (abs(ypr[0] - previous_ypr[0]) < 2 || abs(abs(ypr[0] - previous_ypr[0]) - 360) < 2) {
        previous_ypr[m] = ypr[m];
      }
    }
    return true;
  }
  return false;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void imuSetup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  // Serial.begin(115200);
  // while (!Serial)
  // ;  // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.
  int connectAttempt = 0;
  do {
    // initialize device
    PTLF("Initializing MPU...");
#if defined CONFIG_DISABLE_HAL_LOCKS && CONFIG_DISABLE_HAL_LOCKS == 1
    PTL("OK");
#else
    PTL("If the program stucks, modify the header file:\n  https://docs.petoi.com/arduino-ide/upload-sketch-for-biboard#sdkconfig.h");
#endif
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    // verify connection
    PTF("- Testing MPU connections...attempt ");
    PTL(connectAttempt++);
    delay(500);
  } while (!mpu.testConnection());
  PTLF("- MPU6050 connection successful");

  // load and configure the DMP
  PTLF("- Initializing DMP...");

  devStatus = mpu.dmpInitialize();

  for (byte m = 0; m < 6; m++)
    imuOffset[m] = i2c_eeprom_read_int16(EEPROM_IMU + m * 2);
  // supply the gyro offsets here, scaled for min sensitivity
  mpu.setXAccelOffset(imuOffset[0]);
  mpu.setYAccelOffset(imuOffset[1]);
  mpu.setZAccelOffset(imuOffset[2]);  //gravity
  mpu.setXGyroOffset(imuOffset[3]);   //yaw
  mpu.setYGyroOffset(imuOffset[4]);   //pitch
  mpu.setZGyroOffset(imuOffset[5]);   //roll

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    if (newBoard)
    {
      #ifndef AUTO_INIT
      PTL ("- Calibrate the Inertial Measurement Unit (IMU)? (Y/n): ");
      char choice = getUserInputChar ();
      PTL (choice);
      if (choice == 'Y' || choice == 'y')
      {
        #else
      PTL ("- Calibrate the Inertial Measurement Unit (IMU)...");
      #endif
      PTLF ("\nPut the robot FLAT on the table and don't touch it during calibration.");
      #ifndef AUTO_INIT
      beep (8, 500, 500, 5);
      #endif
      beep (15, 500, 500, 1);
      mpu.CalibrateAccel (20);
      mpu.CalibrateGyro (20);
      i2c_eeprom_write_int16 (EEPROM_IMU, mpu.getXAccelOffset ());
      i2c_eeprom_write_int16 (EEPROM_IMU + 2, mpu.getYAccelOffset ());
      i2c_eeprom_write_int16 (EEPROM_IMU + 4, mpu.getZAccelOffset ());
      i2c_eeprom_write_int16 (EEPROM_IMU + 6, mpu.getXGyroOffset ());
      i2c_eeprom_write_int16 (EEPROM_IMU + 8, mpu.getYGyroOffset ());
      i2c_eeprom_write_int16 (EEPROM_IMU + 10, mpu.getZGyroOffset ());
      beep (18, 50, 50, 6);
      #ifndef AUTO_INIT
      }
    #endif
    mpu.PrintActiveOffsets ();
    }
    // turn on the DMP, now that it's ready
    PTLF("- Enabling DMP...");
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    PTF("- Enabling interrupt detection (Arduino external interrupt ");
    PT(digitalPinToInterrupt(INTERRUPT_PIN));
    PTLF(")...");
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    PTLF("- DMP ready! Waiting for the first interrupt...");
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    PTF("- DMP Initialization failed (code ");
    PT(devStatus);
    PTLF(")");
  }

  delay(10);
  read_IMU();
  // for (byte t = 0; t < 100; t++) {
  //   read_IMU();
  //   print6Axis();
  //   delay(2);
  // }
  exceptions = aaReal.z < 0;
  previous_ypr[0] = ypr[0];
}


#pragma region -ee- BEGIN:  imuCalibrate_ee()

void imuCalibrate_ee()  // -ee- This is meant to be called in este.h, within tokenExtenderQ(), under switch case T_PRINT_GYRO: but we are  using the modified imSetup() for now.
{
  /*  -ee- 
    Summary:  
    This function is currently a subset of the code in original imuSetup() [imu.h].  It is used to periodically calibrate the MPU6050 Inertial Measurement Unit (IMU) to minimize the effects of drift, especially rotational yaw drift about the Z axis.  
    
    Details:
    This IMU uses a "6-axis" IMU which means it has a 3-axis gyroscope to measure rotation about the X, Y and Z axis and a 3-axis accelerometer to measure acceleration (and therefore displacement) in the X, Y and Z direction.  The accelerometers are used to compensate for pitch and roll rotational drift about the X and Y axes, respectively since they can use the gravity acceleration vector.  However, this approach is ineffective in compensating for yaw drift since it is rotation in the XY plane about the Z axis and has no gravity vector component.  In 9-axis IMUs, a 3-axis magnetometer is added which gives a magnetic vector in the XY plane that can be used to compensate for yaw drift.  

    However, we don't have such an IMU on the BiBoard, so dealing with yaw drift in a "6-axis" IMU is challenging.  The approach we will use is to calibrate (and therefore re-zero) the IMU before relying on it for inertial guidance.  

    Note:  This coded originates from imuSetup() [imu.h]

    Currently, this function is called in these locations:
      @ tokenExtenderQ() [este.h] under switch case T_PRINT_GYRO:
  */

  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();        // -ee- DMP = Digital Motion Processor.  See https://mjwhite8119.github.io/Robots/mpu6050

  for (byte m = 0; m < 6; m++)
    imuOffset[m] = i2c_eeprom_read_int16(EEPROM_IMU + m * 2);
  // supply the gyro offsets here, scaled for min sensitivity
  mpu.setXAccelOffset(imuOffset[0]);
  mpu.setYAccelOffset(imuOffset[1]);
  mpu.setZAccelOffset(imuOffset[2]);  //gravity
  mpu.setXGyroOffset(imuOffset[3]);   //yaw
  mpu.setYGyroOffset(imuOffset[4]);   //pitch
  mpu.setZGyroOffset(imuOffset[5]);   //roll
  mpu.PrintActiveOffsets();

  PTL("Calibrating IMU with " + String(numLoops) + " loops ....");       // -ee- numLoops is initialized in OpenCat.h
  mpu.CalibrateAccel(numLoops);                                          // -ee- Original value = 20
  mpu.CalibrateGyro(numLoops);                                           // -ee- Original value = 20
  i2c_eeprom_write_int16(EEPROM_IMU, mpu.getXAccelOffset());
  i2c_eeprom_write_int16(EEPROM_IMU + 2, mpu.getYAccelOffset());
  i2c_eeprom_write_int16(EEPROM_IMU + 4, mpu.getZAccelOffset());
  i2c_eeprom_write_int16(EEPROM_IMU + 6, mpu.getXGyroOffset());
  i2c_eeprom_write_int16(EEPROM_IMU + 8, mpu.getYGyroOffset());
  i2c_eeprom_write_int16(EEPROM_IMU + 10, mpu.getZGyroOffset());
  mpu.PrintActiveOffsets();

  mpu.setDMPEnabled(true);
  mpuIntStatus = mpu.getIntStatus();

  read_IMU();
}
#pragma endregion   END:  imuCalibrate_ee()


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void imuExample() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  read_IMU();
  print6Axis();
}
