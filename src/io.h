#include "soc/gpio_sig_map.h"

// — read master computer’s signals (middle level) —

// This example code is in the Public Domain (or CC0 licensed, at your option.)
// By Richard Li - 2020
//
// This example creates a bridge between Serial and Classical Bluetooth (SSP with authentication)
// and also demonstrate that SerialBT has the same functionalities as a normal Serial

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
  SerialBT.enableSSP();
  SerialBT.onConfirmRequest(BTConfirmRequestCallback);
  SerialBT.onAuthComplete(BTAuthCompleteCallback);
#ifdef I2C_EEPROM_ADDRESS
  PTHL("SSP:\t", strcat(readLongByBytes(EEPROM_BLE_NAME), "_SSP"));
  SerialBT.begin(strcat(readLongByBytes(EEPROM_BLE_NAME), "_SSP"));  // Bluetooth device name
#else
  String blueID = "" + config.getString("ID", "P") + "_SSP";
  PTHL("SSP:\t", blueID);
  SerialBT.begin(blueID.c_str());  // Bluetooth device name
#endif
  Serial.println("The SSP device is started, now you can pair it with Bluetooth!");
}

// void readBlueSSP() {
//   if (confirmRequestPending)
//   {
//     if (Serial.available())
//     {
//       int dat = Serial.read();
//       if (dat == 'Y' || dat == 'y')
//       {
//         SerialBT.confirmReply(true);
//       }
//       else
//       {
//         SerialBT.confirmReply(false);
//       }
//     }
//   }
//   else
//   {
//     if (Serial.available())
//     {
//       SerialBT.write(Serial.read());
//     }
//     if (SerialBT.available())
//     {
//       Serial.write(SerialBT.read());
//     }
//     delay(20);
//   }
// }

// end of Richard Li's code

#endif

// Original version of printToAllPorts()
template<typename T>
void printToAllPorts(T text, bool newLine = true) {
#ifdef BT_BLE
  if (deviceConnected)
    bleWrite(String(text));
#endif
#ifdef BT_SSP
  if (BTconnected)
    SerialBT.println(text);
#endif
#ifdef WEB_SERVER
  if (cmdFromWeb) {
    webServer.send(200, "text/plain", String(text));
    cmdFromWeb = false;
  }
#endif
  if (moduleActivatedQ[0])  // serial2
    Serial2.println(text);
  PT(text);
  if (newLine)
    PTL();
}


// Replacement overload versions of printToAllPorts()

// Using XML Intellisense Documentation
/// <summary>
///   Overload 1 of 2
///   <para>
///     A String is printed to all ports, typically using print functions derived from the print() and println() base classes.
///   </para>
///   <para> ----------</para>
///   <para>
///     [ Replaces PT(s) / PTF(s) and PTL(s) / PTLF(s) ]...............where s is a String supplied as parameter "item"
///   </para>
///   <para>
///     [ Replaces PTT(s, delimiter) and PTTL(s, delimiter) ]......where s and delimiter are Strings that are concatenated and supplied as parameter "item"
///   </para>
///   <para>
///     [ Replaces PTH(head, value) and PTHL(head, value) ]...where head and value are Strings that are concatenated and supplied as parameter "item"
///   </para>
/// </summary>
/// <param name="item:  ">Name ITEM to print (type "T" is any type accepted by the String() function).</param>
/// <param name="printNewLine:  ">Append a newline after print?  enumPrintNewLine::Yes or enumPrintNewLine::No</param>
/// <returns>void</returns>
/// <remarks>
///   The forward declaration of this function in OpenCat.h requires that the default value be given there and not here in the definition.
/// </remarks>
template<typename T>
void printToAllPorts(T item, enumPrintNewLine printNewLine) {
  static_assert(   // At compile time, restrict "item" to only types that String() supports.
    std::is_same<T, int>::value || 
    std::is_same<T, unsigned int>::value || 
    std::is_same<T, long>::value || 
    std::is_same<T, unsigned long>::value || 
    std::is_same<T, float>::value || 
    std::is_same<T, double>::value || 
    std::is_same<T, char>::value || 
    std::is_same<T, const char*>::value || 
    std::is_same<T, String>::value || 
    std::is_same<T, bool>::value,
    "Unsupported Template type T for String() conversion"
  );

#ifdef BT_BLE
  if (deviceConnected)
    bleWrite(String(item));
#endif

#ifdef BT_SSP
//  if (BTconnected)          // This does not work with SPP communication.  (See https://tinyurl.com/bluetooth-spp-communication post in the Petoi.Camp.)
  if ( SerialBT.isReady() )   // This DOES work with Bluetooth SPP communication. (See the same post.)
    SerialBT.print(String(item) );

    if (printNewLine == enumPrintNewLine::Yes)
      SerialBT.println();
#endif

#ifdef WEB_SERVER
  if (cmdFromWeb) {
    webServer.send(200, "item/plain", String(item));
    cmdFromWeb = false;
  }
#endif

  if (moduleActivatedQ[0])  // Serial2
  {
    Serial2.print(String(item) );

    if (printNewLine == enumPrintNewLine::Yes)
      Serial2.println();
  }
  Serial.print(String(item) );

  if (printNewLine == enumPrintNewLine::Yes)
    Serial.println();
}


// Using XML Intellisense Documentation
/// <summary>
///   Overload 2 of 2
///   <para>
///     A Formatted Number is printed to all ports, typically using print functions derived from the print() and println() base classes.
///   </para>
///   <para> ----------</para>
///   <para>
///     [ Replaces PPTD(s, format) ].....where s (numeric) and format (unsigned int) are supplied as parameters "item" and "format"
///   </para>
/// </summary>
/// <param name="item:  ">ITEM number to print (type "N" must one of the following: unsigned char, int, long, long long, double).</param>
/// <param name="format:  ">Number FORMAT to apply to the ITEM (type "unsigned int" so must be 0 and positive integers including these special "format" values:  DEC = 10, HEX = 16, OCT = 8, BIN = 2).</param>
/// <param name="printNewLine:  ">Append a newline after print?  enumPrintNewLine::Yes or enumPrintNewLine::No</param>
/// <returns>void</returns>
/// <remarks>
///   The forward declaration of this function in OpenCat.h requires that the default value be given there and not here in the definition.
/// </remarks>
template<typename N>
void printToAllPorts(N item, unsigned int format, enumPrintNewLine printNewLine) {
  static_assert(std::is_arithmetic<N>::value, "Template type N must be numeric!");  // At compile time, restrict "item" to only integer or floating-point types.

#ifdef BT_BLE
  if (deviceConnected)
    bleWrite(String(item) );  // Nothing equivalent to Serial.print(val, format) so ignore the "format" parameter
#endif

#ifdef BT_SSP
//  if (BTconnected)          // This does not work with SPP communication.  (See https://tinyurl.com/bluetooth-spp-communication post in the Petoi.Camp.)
  if ( SerialBT.isReady() )   // This DOES work with Bluetooth SPP communication. (See the same post.)
    SerialBT.print(item, format);

    if (printNewLine == enumPrintNewLine::Yes)
      SerialBT.println();
#endif

#ifdef WEB_SERVER
  if (cmdFromWeb) {
    webServer.send(200, "item/plain", String(item));  // Nothing equivalent to Serial.print(val, format) so ignore the "format" parameter
    cmdFromWeb = false;
  }
#endif

  if (moduleActivatedQ[0])  // Serial2
  {
    Serial2.print(item, format);

    if (printNewLine == enumPrintNewLine::Yes)
      Serial2.println();
  }
  Serial.print(item, format);

  if (printNewLine == enumPrintNewLine::Yes)
    Serial.println();
}

