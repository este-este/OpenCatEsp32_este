#pragma region -ee- BEGIN:  <doc> File Changes

/*             Documentation of -ee- Changes In THIS File:  Updated 2025-02-03a

      CHANGES:

  - In readEnvironment()
      - Added:  /* -ee- Comment out original code.
          Then added:  #pragma region -ee- BEGIN:  <merge> Replace use of gyroBalanceQ with imuDataReadQ.
  -  In template<typename T> void printToAllPorts(T text)
      - Added:  #pragma region -ee- BEGIN:  <merge> Added #ifdef PRINT_TO_ALL_PORTS___TESTING for testing.
      - Added:  /* -ee- Comment out original code.
          Then added:  #pragma region -ee- BEGIN:  <merge> Changed to use correct method in the guard condition.

      ENABLED / DISABLED:

  Enabled
    - 

  Disabled
    - 

*/
#pragma endregion   END:  <doc> File Changes


#include "soc/gpio_sig_map.h"

void read_sound() {
}

void read_GPS() {
}
#ifdef TOUCH0
void read_touch() {
  byte touchPin[] = {
    TOUCH0,
    TOUCH1,
    TOUCH2,
    TOUCH3,
  };
  for (byte t = 0; t < 4; t++) {
    int touchValue = touchRead(touchPin[t]);  //do something with the touch?
    //    PT(touchValue);
    //    PT('\t');
  }
  //  PTL();
}
#endif
void readEnvironment() {
#ifdef GYRO_PIN
/* -ee- Comment out original code.
  if (gyroBalanceQ && !(frame % imuSkip))
    imuUpdated = read_IMU();
*/
  #pragma region -ee- BEGIN:  <merge> Replace use of gyroBalanceQ with imuDataReadQ.
  if (imuDataReadQ && !(frame % imuSkip))  // -ee- Here, I replaced gyroBalanceQ with imuDataReadQ.
    imuUpdated = read_IMU();  // -ee- This is the key line that reads IMU data.
  #pragma endregion   END:  <merge> Replace use of gyroBalanceQ with imuDataReadQ.
#endif
  read_sound();
  read_GPS();
}

//— read master computer’s signals (middle level) —

//This example code is in the Public Domain (or CC0 licensed, at your option.)
//By Richard Li - 2020
//
//This example creates a bridge between Serial and Classical Bluetooth (SSP with authentication)
//and also demonstrate that SerialBT has the same functionalities as a normal Serial

#ifdef BT_SSP
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
boolean confirmRequestPending = true;
boolean BTconnected = false;

void BTConfirmRequestCallback(uint32_t numVal) {
  confirmRequestPending = true;
  Serial.println(numVal);
}

void BTAuthCompleteCallback(boolean success) {
  confirmRequestPending = false;
  if (success) {
    BTconnected = true;
    Serial.println("SSP Pairing success!!");
  } else {
    BTconnected = false;
    Serial.println("SSP Pairing failed, rejected by user!!");
  }
}

void blueSspSetup() {
  PTH("SSP: ", strcat(readLongByBytes(EEPROM_BLE_NAME), "_SSP"));
  SerialBT.enableSSP();
  SerialBT.onConfirmRequest(BTConfirmRequestCallback);
  SerialBT.onAuthComplete(BTAuthCompleteCallback);
  SerialBT.begin(strcat(readLongByBytes(EEPROM_BLE_NAME), "_SSP"));  //Bluetooth device name
  Serial.println("The SSP device is started, now you can pair it with Bluetooth!");
}

//void readBlueSSP() {
//  if (confirmRequestPending)
//  {
//    if (Serial.available())
//    {
//      int dat = Serial.read();
//      if (dat == 'Y' || dat == 'y')
//      {
//        SerialBT.confirmReply(true);
//      }
//      else
//      {
//        SerialBT.confirmReply(false);
//      }
//    }
//  }
//  else
//  {
//    if (Serial.available())
//    {
//      SerialBT.write(Serial.read());
//    }
//    if (SerialBT.available())
//    {
//      Serial.write(SerialBT.read());
//    }
//    delay(20);
//  }
//}

//end of Richard Li's code

#endif

//#define PRINT_TO_ALL_PORTS___TESTING
template<typename T> void printToAllPorts(T text) {

#ifdef BT_BLE
  if (deviceConnected)
    bleWrite(String(text));
#endif

#pragma region -ee- BEGIN:  <merge> Added #ifdef PRINT_TO_ALL_PORTS___TESTING for testing.

#ifdef PRINT_TO_ALL_PORTS___TESTING      // -ee- for testing.
  PTL("BTconnected = "                  + String(BTconnected                  ) );    //only true upon pairing attempt and then success
  PTL("SerialBT.isReady() = "           + String(SerialBT.isReady()           ) );    //only true when the SerialBT object is ready (the COM port need not exist!)
  PTL("SerialBT.available() = "         + String(SerialBT.available()         ) );    //always seems to be false (perhaps only used for the incoming port FROM BiBoard TO laptop?
//  PTL("SerialBT.availableForWrite() = " + String(SerialBT.availableForWrite() ) );    //Causes a reboot if issues on startup
//  PTL("SerialBT.connected() = "         + String(SerialBT.connected()         ) );    //Causes a reboot if issues on startup
//  PTL("SerialBT.isClosed() = "          + String(SerialBT.isClosed()          ) );    //Causes a reboot if issues on startup
#endif // PRINT_TO_ALL_PORTS___TESTING
#pragma endregion   END:  <merge> Added #ifdef PRINT_TO_ALL_PORTS___TESTING for testing.

#ifdef BT_SSP
/* -ee- Comment out original code.
  if (BTconnected)
*/

#pragma region -ee- BEGIN:  <merge> Changed to use correct method in the guard condition.
  if ( SerialBT.isReady() )
#pragma endregion   END:  <merge> Changed to use correct method in the guard condition.

    SerialBT.println(text);
#endif

  PTL(text);
}
