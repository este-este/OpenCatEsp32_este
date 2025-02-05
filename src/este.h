#pragma region -ee- BEGIN:  <doc> File Changes

/*             Documentation of -ee- Changes In THIS File:  Updated 2025-02-03a

      CHANGES:

  - Added:  #pragma region -ee- BEGIN:  <doc> Project Documentation
      - Contains all project documentation
  - Added:  #pragma region -ee- BEGIN:  <merge> Supporting Code for Function tokenExtenderQ()
      - (says exactly what it contains)
  - Added:  bool tokenExtenderQ()
  - Added:  pragma region -ee- BEGIN:  File Changes <TEMPLATE>

      ENABLED / DISABLED:

  Enabled
    - 

  Disabled
    - 

*/
#pragma endregion   END:  <doc> File Changes


#pragma region -ee- BEGIN:  <doc> Project Documentation

// Project Documentation Header
/*
  Project Title:          OpenCat32_este
  Project Author:         este este
  Project Type:           Fork from 
                          Upstream: https://github.com/PetoiCamp/OpenCatEsp32 (original OpenCatEsp32 source code)
  Project Repo:           Origin:   https://github.com/este-este/OpenCatEsp32_este
                            Last commit from Upstream:  commit #b7a6726f, 4/22/2024 9:52:29 AM
  OpenCat.h DATE:         240422
  Project Last Updated:   2025-02-02a
*/

// Project Documentation Introduction
/*
  The OpenCatEsp32 source code acts as the firmware in the Petoi(TM) BiBoard (https://www.petoi.com/products/biboard-esp32-development-board-for-quadruped-robot), which is an Arduino compatible, Espressif ESP32 System-On-Chip (SoC) based microcontroller.  The BiBoard acts as a type of "nervous system" in some Petoi Bittle quadruped robots (https://www.petoi.com/pages/bittle-open-source-bionic-robot-dog).  It provides access to functions such as limb movement, balance and inertial guidance and various types of sensing.

  This project is to make the Bittle robot easier to interface with a controller computer that will function as the "Bittle Brain" for the BiBoard & sensor "nervous system".

  This project makes the following types of changes to the original OpenCatEsp32 source code:
    @ Configuration Changes: These are within the scope of the original code (e.g. using pre-existing #define macro directives to enable / disable pre-existing functionality)
    @ Functionality Changes:  These are beyond the scope of the original code (e.g. add new functionality or change pre-existing functionality)
 */

// Project Documentation Annotations
/*
  Annotations in this project:
    @ The "at" character ('@') is used as a "bullet", as in "for bulleted lists", for Documentation.
    @ The "dash" character ('-') is used as a "bullet", as in "for bulleted lists", for File Changes
    @ The inwardly pointing chevron characters ('>' and '<') are placed in the left and/or right of certain to call attention to something unusual, especially if that something may be a software defect.
    @ The inwardly pointing brace characters ('}' and '{') are placed in the left and/or right of certain to call attention to incomplete information that will be added later.
    @ The square bracket characters "[]" are used to enclose a filename, typically after a function in that file is referenced.
*/

// Project Toolset
/*
  Visual Studio Community 2019 (VSC-2019) and the Visual Micro Extension (VME - see https://www.visualmicro.com/) were used for this project.  All code is compatible with the Arduino IDE v1.8 and later.  In particular, the toolchain used by Visual Micro is identical to that used by the Arduino IDE (see https://www.visualmicro.com/page/User-Guide.aspx?doc=How-The-Tools-Play-Together.html ) so the compile result is the same.  IMO, the coding experience is much better than with the Arduino IDE and even better than using VSCode with the appropriate extension.

  Note:  I try to keep code lines relatively short but the documentation comment can get long so I use Visual Studio's word-wrap mode.  Arduino IDE versions prior to v2.x lack this capability.
*/

// Project Style Guide and Special Techniques
/*
  This project uses a code formatting style, documented at https://github.com/este-este/A-Code-Formatting-Style, that covers how C++ code text and comment text appear in anything new that I have written as well as most changes I make to pre-existing OpenCatEsp32 source code.

  Visual Studio C++ pre-processor directives "#pragma region" and "pragma endregion", documented at https://learn.microsoft.com/en-us/cpp/preprocessor/region-endregion?view=msvc-170, are used for these special purposes:
    @ To act as a "folding comment" that can both wrap and document a long section of code.
        @ Use the format:  #pragma region -ee- BEGIN:  <doc>
    @ To delimit new code that must survive a git merge
        @ Use the format:  #pragma region -ee- BEGIN:  <merge>
  Note that Visual Studio and VSCode will code fold on these directives but the Arduino IDE does not currently have a code fold capability.

  Multi-line comments are used for this special purpose:
    @ To wrap and thereby disable pre-existing code in such a way as to avoid git merge conflicts.

  This project also uses these techniques:
    @ Single line comments are sometimes used above key opening curly braces, '{', to indicate -ee- modifications to pre-existing code.
        It is done this way so that when the curly braces "code fold", the comment can still be seen.
        Note:  The "//" comment delimiter must be indented at the same level as the opening curly brace.
    @ Lambda expressions (C++ anonymous functions) are used to emulate the local functions found in other languages (like C#).
    @ The unique identifier "-ee-" is used to tag comments I make.
        @ In particular, I sometimes add an in-line comment to explain a pre-existing code statement.  Such tagging will help the comments survive a git merge.
*/

// Project File Changes:  Currently >>>> 11 <<<< Files have modifications of pre-existing code or have new "ee" code.
/*
  OpenCatEsp32.ino
  OpenCat.h
  reaction.h
  InstinctBittleESP.h
  este.h
  skill.h
  io.h
  imu.h
  doubleLight.h
  I2cEEPROM.h
  MPU6050.cpp
*/

// Project Configuration Changes (within the scope of the original code)
/*
  @ Enabled the voice extension (Uncommented "#define VOICE" in OpenCatEsp32.ino)
*/

// Project Functionality Changes (beyond the scope of the original code)

  // @b General Read-back Functionality:  Modified template printToAllPorts() [io.h]
/*
  The template "template<typename T> void printToAllPorts(T text)" [io.h] was modified to use the correct method in the "#ifdef BT_SSP" guard condition.  This provides reliable bidirectional Bluetooth Serial Port Profile (SPP) communication between the robot controller computer and the BiBoard. See https://www.petoi.camp/forum/clinic/solved-in-search-of-reliable-bidirectional-bluetooth-serial-port-profile-spp-communication  

  I also added support for the preprocessor macro "#define PRINT_TO_ALL_PORTS___TESTING" [io.h].

  This function is called in:
    @ tokenExtenderQ()  [este.h]
    @ print6Axis_ee()   [imu.h]
    @ read_serial()     [moduleManager.h]
    @ initRobot()       [OpenCat.h]
    @ reaction()        [reaction.h]
    @ lookUp()          [skill.h]
    @ perform()         [skill.h]
    @ set_voice()       [voice.h]

  This function calls:
    @ bleWrite()
    @ String()
    @ SerialBT.println()
    @ PTL()
*/

  // @b IMU Read-back Functionality 1:  Created function print6Axis_ee() [imu.h]
/*
  The robot controller computer expects packets of telemetry data from sensors in the robot.  In order to use the IMU sensor for inertial guidance, I made the 
  function "void print6Axis_ee(bool createPacketQ)" [imu.h] to generate the desired data packets.  The original print6Axis() is retained and used (via the new T_IMU_READ_DATA token) when accessing the new IMU calibration functionality (see below).

  Note:  A forward declare for "template<typename T> void printToAllPorts(T text)" was added since  printToAllPorts()is used within print6Axis_ee().

  This function is called in:
    @ read_IMU()        [imu.h]      This call uses print6Axis_ee(true) to produce the IMU data packet.  That packet will be detected and used by the robot controller computer.  The IMU data will be visible to the GUI of the robot controller computer but the controller will not "act" upon that data.
    @ tokenExtenderQ()  [este.h]     This call uses print6Axis_ee(false) so it provides IMU data but not in a packet format.

  This function calls:
    @ millis()
    @ dtostrf()
    @ sprintf()
    @ String()
    @ printToAllPorts()  [io.h]
*/

  // @b IMU Read-back Functionality 2:  Modified function read_IMU() [imu.h]
/*
  The function "bool read_IMU()" [imu.h] was modified to use the above created "print6Axis_ee()" [imh.h], instead of the original "print6Axis()" [imh.h].

  Added these supporting global variables and directive macros.
    @ int printGyroSkipNum            [OpenCat.h]
    @ #define print6AxisCounterStart  [OpenCat.h]
    @ int print6AxisCounter           [OpenCat.h]
    @ int numLoops                    [OpenCat.h]

  This function is called in:
    @ imuSetup()            [imu.h]
    @ imuCalibrate_ee()     [imu.h]
    @ imuExample            [imu.h]
    @ readEnvironment()     [io.h]
    @ testMPU()             [qualityAssurance.h]
    @ dealWithExceptions()  [reaction]
    @ perform()             [skill.h]

  This function calls:
    @ print6Axis_ee()             [imu.h]     to generate an IMU data packet for read-back to the controller computer at intervals set by printGyroSkipNum.
    @ MPU6050 mpu object methods  [imu.h]
    @ abs()
*/

  // @b IMU Calibration Functionality:  Created function imuCalibrate_ee() [imu.h]
/*
  This project uses inertial guidance to help Bittle navigate.  However, the MPU6050 Inertial Measurement Unit (IMU) on the BiBoard has significant yaw drift and requires frequent calibration (see the documentation in this function for more info).  The function "void imuCalibrate_ee()" [imu.h] was therefore created to perform IMU calibration on command.  Note that forward declarations were made before each call.  The IMU calibration functionality is accessed via the tokenExtenderQ() function which extends token T_PRINT_GYRO = 'v' to add "v c", "v cv1", and 'v cv0" as behavior modifiers.
    
  This function is called in:
    @ initRobot()       [OpenCat.h]
    @ tokenExtenderQ()  [este.h]

  This function calls:
    @ Wire.setClock()               [I2Cdev.cpp]
    @ MPU6050 mpu object methods    [imu.h]
    @ i2c_eeprom_write_int16()      [I2cEEPROM.h]
    @ read_IMU()                    [imu.h]
*/

  // @ Serial Token & Command Functionality:  Created function tokenExtenderQ() [este.h] and modified pre-existing and created new tokens in reaction() [reaction.h]
/*
  <-Background->

  Control of the robot occurs via serial commands sent to the BiBoard and received in one of the "read" functions within readSignal() [moduleManager.h].  Simple commands consist of a single character which the code stores in a global char variable called "token".  More complex commands add a stream of characters after the token which the code stores in a global char pointer variable (C-style string aka a null-terminated array of characters) called "newcmd".  

  The contents of the token and newcmd variables is processed within the switch control statement of reaction() [reaction.h], where the token character is compared to a matching case statement and appropriate action is taken.

  <-Project->

  The project requires expanded functionality from the OpenCatEsp32 token command set.  
    @ In some cases, I used the pre-existing token but added capabilities via the newcmd variable.  That is the primary purpose of the new tokenExtender() [este.h].  Its secondary purpose is to place such functionality in the este.h file, outside of the original OpenCatEsp32 source code to minimize git merge conflicts.
    @ In other cases, I added new tokens and sometimes passed information via the newcmd variable.  This required adding a new case statement for each new token and also involved use of the tokenExtender() function.
        @ Note that the token landscape is getting a bit crowded so I am using non-alphabetical members of the ASCII character set as tokens.

  The following changes were therefore made:

    @ Created
        @ function "bool tokenExtenderQ(char _token, char* _newCmd, bool verboseQ, String tokenHelp, bool& _tokenQ)"   [este.h]
            @ Provides flexibility to the token system whereby you can:
                @ set/get the value of certain global runtime variables or select different behaviors
                @ provide contextual help
                @ toggle the value of certain boolean variables
            @ Required placement of a forward declaration for this function placed before reaction() [reaction.h]
            @ See the documentation provide with this function

    @ Specifics
            @ Added "sght" = "set/select, get, help, toggle" functionality to extend these pre-existing simple bool tokens:
                @ T_GYRO_BALANCE            currently 'G'
                @ T_VERBOSELY_PRINT_GYRO    currently 'V'
            @ Added the ability to calibrate the IMU with or without starting/stopping verbose gyro (IMU) printing with this pre-existing bool token:
                @ T_PRINT_GYRO              currently 'v'
            @ Added new token T_IMU_READ_DATA       = '@' to turn on or off the new "read IMU data" capability (see below).
            @ Added new token T_DISPLAY_MORE_INFO   = '+' to provide more info that this project needed.
            @ Added new token T_SUPER_TOKEN         = '`' (aka the "backtick" character) as a way to change selected global runtime variables.
                @ Currently, the global runtime variables that can be changed via sending this token are:
                    @ runDelay        now have direct control over gait speed
                    @ soundState      now have direct control over the buzzer/melody speaker mute and unmute capability (note, this is not the speaker on the voice module)
                    @ buzzerVolume    now have direct control over the buzzer/melody speaker volume

  This function is called in:
    @ reaction()  [reaction.h]

  This function calls:
    @ strlenUntil()
    @ strlen()
    @ String()
    @ arrayNCPY()
    @ strtok()
    @ printToAllPorts()
    @ .toInt()
    @ print6Axis()
    @ imuCalibrate_ee()
    @ print6Axis_ee()
    @ tQueue->addTask()
*/
#pragma endregion   END:  <doc> Project Documentation

// -ee- This header file should be included after all other includes in this project.
/*  -ee-
  This is typically done in the .ino (sketch) file.
  Furthermore, you should Forward Declare each function just before it is first used.
*/

#pragma region -ee- BEGIN:  <merge> Supporting Code for Function tokenExtenderQ()

// Define enums that token T_SUPER_TOKEN will use
enum StringCode
{
  enumNoRuntimeVariableName = -2,
  enumUnknownRuntimeVariableName = -1,
  enumRunDelay = 0,
  enumSoundState = 1,
  enumBuzzerVolume = 2
};

// Define the blank runtime variable name
String _noRuntimeVariableName  = "";

// Define names of global runtime variables that token T_SUPER_TOKEN can access as an array.
String runtimeNamesToAccess[] = 
  {
    "runDelay", 
    "soundState", 
    "buzzerVolume"
  };

// Define simple hash function to accept a string and return a matching integer that can then be used in a switch control statement
StringCode hashString(String str)
{
  if (str == runtimeNamesToAccess[StringCode::enumRunDelay])
    {
      return StringCode::enumRunDelay;
    }
  else if (str == runtimeNamesToAccess[StringCode::enumSoundState])
    {
      return StringCode::enumSoundState;
    }
  else if (str == runtimeNamesToAccess[StringCode::enumBuzzerVolume])
    {
      return StringCode::enumBuzzerVolume;
    }
  else if (str == _noRuntimeVariableName)
    {
      return StringCode::enumNoRuntimeVariableName;
    }
  else
    {
      return StringCode::enumUnknownRuntimeVariableName;
    }
}

bool canConvertStrToIntQ(String str)
{                                             // Use strtol() instead of stoi() since it has error handling capability.
  char *endptr;                               // A char pointer that will be set to first character in the string str that can not converted to a long.  Will be set to the null character, '\0' if successful.
  long result;                                // The "result" is the value of the long after the string to long conversion (or partial conversion)
  result = strtol(str.c_str(), &endptr, 10);  // The ".c_str()" string member function returns a pointer to null-terminated C-string (char array ending in the null character, '\0').  
                                                  // The function strtol() converts string str to a long that, here, is in base 10.
  if (str == endptr)  // Happens when string is empty or consists only of spaces.
    {
      return false;
    }
  else
    {
      return (*endptr == '\0') &&                        // To return true (str can be converted to an integer):  *endptr must be equal to the end of C-string character   AND
        (result >= std::numeric_limits<int>::min ()) &&     //                                    the converted result must not be less than the minimum int value      AND
        (result <= std::numeric_limits<int>::max ());       //                                    the converted result must not be greater than the maximum int value   AND
    }
}
#pragma endregion   END:  <merge> Supporting Code for Function tokenExtenderQ()


bool tokenExtenderQ(char _token, char* _newCmd, bool verboseQ, String tokenHelp, bool& _tokenQ)
{
  // BEGIN:  Function tokenExtenderQ() Documentation
/*
      Author: este este
    
      Definitions:
        @ tokenExtender = Extends token functionality which can include the ability to set/select, get, help, toggle capabilities.
        @ TokenQ = The token and any boolean variable (represented by 'Q') it is bound to.
   
      What It Does Now:
        @ Called in reaction() [reaction.h].
        @ Provides flexibility to the token system whereby you can
            @ set the value of certain internal runtime variables or select different behaviors
            @ get the value of certain internal runtime variables
            @ provide contextual help
            @ toggle the value of certain boolean variables
        @ Uses the passed in token and newCmd to implement contextual behaviors.
          newCmd can be:
            @ Just a command appropriate to the token.
            @ A command followed by data appropriate to the token and required by the command.
                @   Currently, the command in newCmd is lower case and not space delimited.  This could be changed in the future...
        @ Uses parameter verboseQ when additional information should be displayed.
        @ Uses parameter tokenHelp to provide a token dependent help message.
        @ Uses parameter _tokenQ when the token is dependent on the state of a token-specific boolean variable.

      What It Might Do In The Future:
        @ Use a future tokenState parameter to capture the current state of the token (allows for boolean values with casting).
    
      Details:
        @ This function is used with tokens that can have different behaviors or states, including those bound to a boolean variable.
          In the later case, the boolean variable normally just acts as a toggle and has a variable name 
          typically ending in the letter 'Q', normally just acts as a toggle.
        @ This function therefore extends the capability of the following tokens:            
            @ For token T_GYRO_BALANCE, currently 'G'
                @ _newCmd must be one of the following:
                        @ "0"     Sets _tokenQ to false which is then returned to set the calling boolean "Q" variable.
                        @ "1"     Sets _tokenQ to true which is then returned to set the calling boolean "Q" variable.
                        @ "="     Gets the value of _tokenQ and does read-back.
                        @ "?"     Provides help as described in tokenHelp.
                        @ ""      Performs a toggle of _tokenQ.
            
            @ For token T_PRINT_GYRO, currently 'v'
                @ _tokenQ:   Is not used.
                @ _newCmd must be one of the following:
                        @ ""              Gives one read-back in the original (non-packet) style.
                        @ "c"             Performs the IMU calibration (with before and after read-back in the original (non-packet) style).
                        @ "cv1" or "cv0   Same as item "c", except after that behavior, it uses "V1" or "V0" to start or stop verbose gyro printing.
                        @ "?"             Provides help as described in tokenHelp.
            
            @ For token T_VERBOSELY_PRINT_GYRO, currently 'V'
                @ _newCmd must be one of the following:
                        @ "0"     Sets _tokenQ to false which is then returned to set the calling boolean "Q" variable.
                        @ "1"     Sets _tokenQ to true which is then returned to set the calling boolean "Q" variable.
                        @ "="     Gets the value of _tokenQ and does read-back.
                        @ "?"     Provides help as described in tokenHelp.
                        @ ""      Performs a toggle of _tokenQ.
            
            @ For token T_IMU_READ_DATA, currently, '@'
                @ _newCmd must be one of the following:
                        @ "0"     Sets _tokenQ to false which is then returned to set the calling boolean "Q" variable.
                        @ "1"     Sets _tokenQ to true which is then returned to set the calling boolean "Q" variable.
                        @ "="     Gets the value of _tokenQ and does read-back.
                        @ "?"     Provides help as described in tokenHelp.
                        @ ""      Performs a toggle of _tokenQ.
   
      How It Works:
        @ Receive
            @ token _token             Value of the token to operate on.
            @ newCmd _newCmd           Pointer to the characters after the token.
            @ boolean verboseQ         Determines if just the result is given in the read-back (when verboseQ = true) or if additional text is also used in the read-back (when verboseQ = false).
            @ token help tokenHelp     Value of the help to display for this token.
            @ token boolean _tokenQ    Reference to any bool variable bound to that token.
        @ Convert _newCmd to _newCmd_WithNullTerminator
        @ Split _newCmd_WithNullTerminator (on " ", i.e. a space) and look for string values to apply to the token.
            @ Use newCmd_FirstValueStr values "1" or "0" to set _tokenQ to true or false
            @ Use newCmd_FirstValueStr value "=" to display value of _tokenQ
            @ Use newCmd_FirstValueStr value "?" to display help on the token
            @ Use newCmd_FirstValueStr value "" to toggle _tokenQ, but only if the token is bound to a boolean variable referenced by _tokenQ
        @ Take appropriate actions, including modifying _tokenQ and setting functionSucceededQ to true or false.
        @ Return functionSucceededQ
   
      Dependencies:
        @ This function is used by reaction() [reaction.h] and itself uses printToAllPorts [io.h]
   
      Other Notes:
        @ Must add a forward declaration with the function's signature 
            immediately before the definition of the function reaction() [reaction.h].
            For example:
            bool tokenExtenderQ(char _token, char* _newCmd, bool verboseQ, String tokenHelp, bool& _tokenQ);
   
        @ This function could be extended to the following tokens that have bool variable bindings:
            @ token T_GYRO_FINENESS 'g' bound to  fineAdjust    probably should be named with a "Q".
            @ token T_RANDOM_MIND 'z'   bound to  autoSwitch    probably should be named with a "Q".
            @ token T_BEEP 'b'          bound to  soundState    probably should be named with a "Q".
            @ token T_BEEP_BIN 'B'      bound to  soundState    probably should be named with a "Q".
              @ Note:  token T_BEEP 'b' and token T_BEEP_BIN 'B' are bound to bool soundState but are also associated with
                volume via byte buzzerVolume.
                    @  Would need to get 3 characters, not 1 character from newCmd.
*/
  // END:  Function tokenExtenderQ() Documentation

  // BEGIN:  Process function parameters:  _token, _newCmd
  bool functionSucceededQ = false;
  int _cmdLen = 0;
  char _cmdTerminator = '\0';  // Initialized to default value for lowercase plus special characters.

  // Define C-string variables to hold the first and second string (" " delimited) in _newCmd.
  char *newCmd_FirstValueAsCstring;
  char *newCmd_SecondValueAsCstring;

 // Define Cpp-string variables to hold the first and second string (" " delimited) in _newCmd.
  String newCmd_FirstValueStr = "";   // Initialized to default value for token alone (_newCmd has no values other than '~' or '\0')
  String newCmd_SecondValueStr = "";  // Initialized to default value for token alone (_newCmd has no values other than '~' or '\0')

  // token & terminator comments
  /* For token = A to Z, unless excluded, newCmd will have a "~" terminator.  If such tokens are presented alone,
     newCmd will have a strlen() of 1, since the single character it contains is "~" which IS counted by that function.
   
     For token <> A to Z, or such those as were excluded, newCmd will have a "\0" terminator.  If such tokens are presented alone,
     newCmd will have a strlen() of 0, since the single character it contains is "\0" which is NOT counted by that function.
   
     So, need to check the token and then set the correct terminator.
  */;

  // Set terminator, depending on token, then get _cmdLen for that terminator.
    if (_token >= 'A' && _token <= 'Z')  // Case for capitalized characters (A-Z)
    {
      _cmdTerminator = '~';  //Set terminator for this case.
    }
  _cmdLen = strlenUntil(_newCmd, _cmdTerminator);

  // Make _newCmd_WithNullTerminator
  char* _newCmd_WithNullTerminator = new char[_cmdLen];     // new string to copy into
  arrayNCPY(_newCmd_WithNullTerminator, _newCmd, _cmdLen);  // copy from parameter _newCmd into new string nCWithNullTerminator)
  _newCmd_WithNullTerminator[_cmdLen] = '\0';               // add '\0' as terminator.

  // Process function parameters
  String _tokenAsStr = String(_token);
  if (strlen(_newCmd_WithNullTerminator) == 0)  // First handle case where the token is alone (_newCmd has no values other than '~' or '\0')
    {
      ;  // Don't need to do anything.
    }
  else  // Then handle all other cases
    {
      newCmd_FirstValueAsCstring = strtok(_newCmd_WithNullTerminator, " ");  // This holds the first string found in _newCmd as a C-string
      newCmd_FirstValueStr = newCmd_FirstValueAsCstring;                     // Convert to a Cpp-string

      newCmd_SecondValueAsCstring = strtok(nullptr, " ");                    // This holds the second string found in (if any) _newCmd  as a C-string
      if (newCmd_SecondValueAsCstring != NULL)
        {
          newCmd_SecondValueStr = newCmd_SecondValueAsCstring;               // Convert to a Cpp-string
        }
    }
  // END:  Process function parameters:  _token, _newCmd

  // BEGIN:  Set up String message variables
  String instructions;
  String setTo;
  String equalTo;
  String toggledTo;

  String toggleInstruction            = "\"" + _tokenAsStr + "\" to toggle";
  String turnOnInstruction            = "\"" + _tokenAsStr + " 1\" to turn on";
  String turnOffInstruction           = "\"" + _tokenAsStr + " 0\" to turn off";
  String getCurrentValueInstruction   = "\"" + _tokenAsStr + " =\" to get current value";
  String showHelpInstruction          = "\"" + _tokenAsStr + " ?\" to show help";
  String spaceAfterTokenInstruction   = "Preferred syntax is to send a space after the token.";
  String specialInstruction           = "";

  String newCmd_FirstValueStr_commandNotRecognizedMsg     = "\"" + newCmd_FirstValueStr + "\" command not recognized";
  String newCmd_SecondValueStr_commandNotRecognizedMsg    = "\"" + newCmd_SecondValueStr + "\" command not recognized";

  // Initialize default instructions given with _newCmd = "?" for boolean tokens
  instructions =
    "(Send " +
    toggleInstruction           + ", " +
    turnOnInstruction           + ", " +
    turnOffInstruction          + ", " +
    getCurrentValueInstruction  + ", " +
    showHelpInstruction         + ".)" +
    "\n"                               + 
    spaceAfterTokenInstruction
    ;

  // Initialize these String elements
  if (verboseQ)
    {
      setTo     = "token " + _tokenAsStr + " set to ";
      equalTo   = "token " + _tokenAsStr + " = ";
      toggledTo = "token " + _tokenAsStr + " toggled to ";
    }
  else
    {
      setTo     = "";
      equalTo   = "";
      toggledTo = "";
    }

  // Give tokenExtender() message
  printToAllPorts("This token has extended functionality.  Get more info by sending the token followed by \" ?\" (with a space).");
  printToAllPorts("Also, try the Display More Info token:  Send \"+\" (or \"+ ?\" to get help on THAT token) ");
  printToAllPorts("");  // Send blank line
  // END:  Set up String message variables

  //Take action
  switch (_token)
    {
      case T_GYRO_BALANCE:          // 'G'
        {
          // Set     "0" or "1"
          if (newCmd_FirstValueStr == "1" || newCmd_FirstValueStr == "0")  // Sent token plus "1" or "0" (prefer with intervening space).
            {
              _tokenQ = newCmd_FirstValueStr.toInt();
              printToAllPorts(setTo + String(_tokenQ));  // Do read-back.
              functionSucceededQ = true;
            }

          // Get     "="
          else if (newCmd_FirstValueStr == "=")  // Sent token plus "=" (prefer with intervening space).
            {
              printToAllPorts(equalTo + String(_tokenQ));  // Do read-back.
              functionSucceededQ = true;
            }

          // Help on this token    "?"
          else if (newCmd_FirstValueStr == "?")  // Sent token plus "?" (prefer with intervening space).
            {
              printToAllPorts(tokenHelp);     // Show token specific help.
              printToAllPorts(instructions);  // Show instructions.
              functionSucceededQ = true;
            }

          // Toggle  ""
          else if (newCmd_FirstValueStr == "")  // Sent token alone.
            {
              _tokenQ = !_tokenQ;                            // Toggle true/false
              printToAllPorts(toggledTo + String(_tokenQ));  // Do read-back.
              functionSucceededQ = true;
            }

          // Command not recognized
          else
            {
            printToAllPorts(newCmd_FirstValueStr_commandNotRecognizedMsg);     // Show "command not recognized" message
            functionSucceededQ = false;
            }

          break;
        }

      case T_PRINT_GYRO:            // 'v'
        {
          // Set up special instructions for THIS token
          instructions =
            "(Send "
            "\"" + _tokenAsStr  + "\" to get a single gyro read-back (original token ability), "
            "\"" + _tokenAsStr  + " c\" to perform IMU calibration with before and after gyro read backs, "
            "\n"
            "\"" + _tokenAsStr  + " cv1\" to calibrate but also turn on verbose gyro printing, "
            "\"" + _tokenAsStr  + " cv0\" to calibrate but also turn off verbose gyro printing, " +
            showHelpInstruction + ".)" +
            "\n" +
            spaceAfterTokenInstruction
            ;

          // Do Original Behavior        ""      One read-back with print6Axis()
          if (newCmd_FirstValueStr == "")
            {
              print6Axis();
            }

          // Calibrate IMU            "c", "cv1", "cv0"        Calibrate IMU with before and after read backs in the original style
          else if (newCmd_FirstValueStr == "c" || newCmd_FirstValueStr == "cv1" || newCmd_FirstValueStr == "cv0")
            {
              print6Axis_ee(false);  // -ee- Show the "before" values.

              printToAllPorts("Running imuCalibrate_ee().....");  // -ee- Tell what we are doing.
              imuCalibrate_ee();                                  // -ee- Do calibration and offsets [was using imuSetup() but now using imuCalibrate_ee() ].
              printToAllPorts("");                                // -ee- Send blank line.
              print6Axis_ee(false);                               // -ee- Show the "after" values.

              // Start Gyro read-backs
              if (newCmd_FirstValueStr == "cv1")
              {
                tQueue->addTask ('V', "1");  // -ee- Start Gyro read-backs.
              }
              // Stop Gyro read-backs
              else if (newCmd_FirstValueStr == "cv0")
              {
                tQueue->addTask ('V', "0");  // -ee- Stop Gyro read-backs.
              }
            }

          // Help on this token                       "?"
          else if (newCmd_FirstValueStr == "?")  // Sent token plus "?" (prefer with intervening space).
            {
              printToAllPorts(tokenHelp);     // Show token specific help.
              printToAllPorts(instructions);  // Show instructions.
            }

          // Command not recognized
          else
            {
            printToAllPorts(newCmd_FirstValueStr_commandNotRecognizedMsg);     // Show "command not recognized" message
            functionSucceededQ = false;
            }

          break;
        }

      case T_VERBOSELY_PRINT_GYRO:  // 'V'
        {
          // Set    "0" or "1"
          if (newCmd_FirstValueStr == "1" || newCmd_FirstValueStr == "0")  // Sent token plus "1" or "0" (prefer with intervening space).
            {
              _tokenQ = newCmd_FirstValueStr.toInt();
              printToAllPorts(setTo + String(_tokenQ));  // Do read-back.
              functionSucceededQ = true;
            }

          // Get    "="
          else if (newCmd_FirstValueStr == "=")  // Sent token plus "=" (prefer with intervening space).
            {
              printToAllPorts(equalTo + String(_tokenQ));  // Do read-back.
              functionSucceededQ = true;
            }

          // Help on this token   "?"
          else if (newCmd_FirstValueStr == "?")  // Sent token plus "?" (prefer with intervening space).
            {
              printToAllPorts(tokenHelp);     // Show token specific help.
              printToAllPorts(instructions);  // Show instructions.
              functionSucceededQ = true;
            }

          // Toggle ""
          else  if (newCmd_FirstValueStr == "")  // Sent token alone.
            {
              _tokenQ = !_tokenQ;                            // Toggle true/false
              printToAllPorts(toggledTo + String(_tokenQ));  // Do read-back.
              functionSucceededQ = true;
            }

          // Command not recognized
          else
          {
            printToAllPorts(newCmd_FirstValueStr_commandNotRecognizedMsg);     // Show "command not recognized" message
            functionSucceededQ = false;
          }

          break;
        }

      case T_IMU_READ_DATA:         // '@'
        {
          // Set    "0" or "1"
          if (newCmd_FirstValueStr == "1" || newCmd_FirstValueStr == "0")  // Sent token plus "1" or "0" (prefer with intervening space).
            {
              _tokenQ = newCmd_FirstValueStr.toInt();
              printToAllPorts(setTo + String(_tokenQ));  // Do read-back.
              functionSucceededQ = true;
            }

          // Get    "="
          else if (newCmd_FirstValueStr == "=")  // Sent token plus "=" (prefer with intervening space).
            {
              printToAllPorts(equalTo + String(_tokenQ));  // Do read-back.
              functionSucceededQ = true;
            }

          // Help on this token   "?"
          else if (newCmd_FirstValueStr == "?")  // Sent token plus "?" (prefer with intervening space).
            {
              printToAllPorts(tokenHelp);     // Show token specific help.
              printToAllPorts(instructions);  // Show instructions.
              functionSucceededQ = true;
            }

          // Toggle ""
          else if (newCmd_FirstValueStr == "")  // Sent token alone.
            {
              _tokenQ = !_tokenQ;                            // Toggle true/false
              printToAllPorts(toggledTo + String(_tokenQ));  // Do read-back.
              functionSucceededQ = true;
            }

          // Command not recognized
          else
            {
            printToAllPorts(newCmd_FirstValueStr_commandNotRecognizedMsg);     // Show "command not recognized" message
            functionSucceededQ = false;
            }

          break;
        }

      case T_DISPLAY_MORE_INFO:     // '+'
        {
          // Set up special instructions for THIS token
          instructions =
            "(Send " +
            showHelpInstruction + ".)" +
            "\n" +
            spaceAfterTokenInstruction
            ;

          // Default behavior       ""
          if (newCmd_FirstValueStr == "")
            {              
            printToAllPorts("Model = " + String(MODEL));
            printToAllPorts("SoftwareVersion = " + String(SoftwareVersion));
            printToAllPorts("Git Commit Date = " + String(DATE));
            printToAllPorts("");  // Send blank line

            printToAllPorts("Bluetooth Name = " + String(readLongByBytes(EEPROM_BLE_NAME)));
            printToAllPorts("Este Date = " + String(ESTE_DATE));
            printToAllPorts(""); // Send blank line

            printToAllPorts("runDelay = " + String(runDelay));   // -ee- Convert the int runDelay to a string
            printToAllPorts("imuDataReadQ = " + String(imuDataReadQ));
            printToAllPorts("gyroBalanceQ = " + String(gyroBalanceQ));
            printToAllPorts("Modules Status: ");
            showModuleStatus();
            printToAllPorts(""); // Send blank line

            printToAllPorts("Tokens using the tokenExtender() function = G v V @ + ` (the last one is the \"backtick\" character)");
            printToAllPorts("Get help on these tokens by sending the token followed by \" ?\". ");

              functionSucceededQ = true;
            }

          // Help on this token   "?"
          else if (newCmd_FirstValueStr == "?")  // Sent token plus "?" (prefer with intervening space).
          {
            printToAllPorts(tokenHelp);     // Show token specific help.
            printToAllPorts(instructions);  // Show instructions.
            functionSucceededQ = true;
          }

          // Command not recognized
          else
          {
            printToAllPorts(newCmd_FirstValueStr_commandNotRecognizedMsg);     // Show "command not recognized" message
            functionSucceededQ = false;
          }

          break;
        }

      case T_SUPER_TOKEN:           // '`'  the backtick character        IN PROGRESS   Change Variable to RuntimeVariable
        {

          // BEGIN:  Set up for this token
          // Set up special instructions for THIS token
          getCurrentValueInstruction = "\"" + _tokenAsStr + " <Runtime_Variable_Name> =\" to get current value of the runtime variable";

          // Help instruction variable for the runtime variable name (NOT used for help on the token itself!)
          String runtimeVariableNameInstruction;

          instructions =
            "(Send " +
            showHelpInstruction + ", " +
            getCurrentValueInstruction + ".)"  +
            "\n"                               +
            spaceAfterTokenInstruction         +
            "\n"                               +
            "The following runtime variables are accessible to the Super Token:  \n" +
            "     runDelay      (sets gait speed in the range of 3 to 20, where a lower value is a faster gait)\n" +
            "     soundState    (sets the robot speaker to 0 = mute, 1 = unmute)\n" +
            "     buzzerVolume  (sets robot speaker volume from low = 1 to high = 10)"
            ;

          // Help on this token   "?"
          if (newCmd_FirstValueStr == "?")  // Sent token plus "?" (prefer with intervening space).
            {
              printToAllPorts(tokenHelp);        // Show token specific help.
              printToAllPorts(instructions);     // Show instructions.
              functionSucceededQ = true;
              break;                             // Nothing more to do.
            }

          // Initialize needed variables.  Use variable names appropriate for this token.

          String runtimeVariableNameStr = newCmd_FirstValueStr;
          String runtimeVariableValueStr = newCmd_SecondValueStr;

          int aliasToActual_RuntimeVariableValue_AsInt = 0;  //Used as an argument to the lambda expressions which can only accept int type 
                                                                // and therefore can accept types like bool and byte that can be cast to int.

          int runtimeVariableValueAsInt = 0;
          int runtimeVariableValueAsInt_Min = 0;
          int runtimeVariableValueAsInt_Max = 999;
          bool runtimeVariableValueIsIntQ = false;
          showHelpInstruction = "\"" + _tokenAsStr + " " + runtimeVariableNameStr + " ?\" to show help";

          // END:  Set up for this token

          // BEGIN:  Lambdas used as local functions

          // Create local_RuntimeVariableValueIsNotAnInteger_Lambda to use as a local function.
          auto local_RuntimeVariableValueIsNotAnInteger_Lambda = 
            [  // Lambda captured variable list
              &runtimeVariableValueStr,
              &runtimeVariableNameStr,
              &showHelpInstruction,
              &runtimeVariableNameInstruction
            ]
            (  // Lambda parameter list
              int &aliasToActual_RuntimeVariableValue_AsInt
            )
          {
            bool functionSucceededQ = false;

            // Help     "?"     (for this specific runtime variable name)
            if (runtimeVariableValueStr == "?")
              {
                printToAllPorts (runtimeVariableNameInstruction);
                functionSucceededQ = true;
              }
            // Get     "="
            else if (runtimeVariableValueStr == "=")
              {
                printToAllPorts (runtimeVariableNameStr + " = " + String(aliasToActual_RuntimeVariableValue_AsInt) );
                functionSucceededQ = true;
              }
            else if (runtimeVariableValueStr == "")
              {
                printToAllPorts("Runtime variable value not supplied.  Send " + showHelpInstruction);
              }
            else
              {
                printToAllPorts ("Invalid runtime variable value.  Must be an integer.  Send " + showHelpInstruction);
              }
            return functionSucceededQ;
          };

          // Create local_RuntimeVariableValueIsAnInteger_Lambda to use as a local function.
          auto local_RuntimeVariableValueIsAnInteger_Lambda = 
            [  // Lambda captured variable list
              &runtimeVariableNameStr,
              &runtimeVariableValueAsInt,
              &runtimeVariableNameInstruction,
              &runtimeVariableValueAsInt_Min,
              &runtimeVariableValueAsInt_Max
            ]
            (  // Lambda parameter list
              int &aliasToActual_RuntimeVariableValue_AsInt
            )
          {
            int runtimeVariableValueAsInt_BeforeChange = 0;
            bool functionSucceededQ = false;
            if (runtimeVariableValueAsInt >= runtimeVariableValueAsInt_Min && runtimeVariableValueAsInt <= runtimeVariableValueAsInt_Max)  // Requested runtime variable value must be in correct range.
              {
                if (runtimeVariableValueAsInt == aliasToActual_RuntimeVariableValue_AsInt)  // Different message if the requested runtime variable value will not actually change runtime variable value.
                  {
                    printToAllPorts(runtimeVariableNameStr + " is already = " + aliasToActual_RuntimeVariableValue_AsInt);
                  }
                else
                  {
                    runtimeVariableValueAsInt_BeforeChange = aliasToActual_RuntimeVariableValue_AsInt;  // Store value before change
                    aliasToActual_RuntimeVariableValue_AsInt = runtimeVariableValueAsInt;  // Make the change then give the changed message.
                    printToAllPorts("Runtime variable " + runtimeVariableNameStr + ":  " + runtimeVariableValueAsInt_BeforeChange + "  changed to  " + aliasToActual_RuntimeVariableValue_AsInt);
                  }
                functionSucceededQ = true;  
              }
            else  // Get here when the requested runtime variable value is not in the correct range.
              {
                printToAllPorts(runtimeVariableNameInstruction);
                printToAllPorts("Runtime variable " + runtimeVariableNameStr + " = " + aliasToActual_RuntimeVariableValue_AsInt + " is unchanged.");
                functionSucceededQ = false;
              }
            return functionSucceededQ;
          };
          
          // END:  Lambdas used as local functions

          // Check that runtimeVariableValueStr is an integer (need to know this later)
          if ( canConvertStrToIntQ(runtimeVariableValueStr) )
            {
              runtimeVariableValueAsInt = runtimeVariableValueStr.toInt();
              runtimeVariableValueIsIntQ = true;
            }
          else
            {
              runtimeVariableValueIsIntQ = false;
            }

          // Process runtime variable name
          switch (hashString(runtimeVariableNameStr) )  // We first look for a runtime variable name that the Super Token can access.
            {
              case StringCode::enumRunDelay:  // 1st runtime variable name that Super Token can access
                {
                  // Set up min and max values for this runtimeVariableName
                  runtimeVariableValueAsInt_Min = delayShort;
                  runtimeVariableValueAsInt_Max = delayLong;

                  // Set up the help for this specific runtime variable name
                  runtimeVariableNameInstruction = 
                    "Runtime variable \"" + runtimeVariableNameStr + "\" must have an integer value >= " + runtimeVariableValueAsInt_Min + " (fast gait) but <= " + runtimeVariableValueAsInt_Max + " (slow gait).\n" +
                    "Send \"" + _tokenAsStr + " " + runtimeVariableNameStr + " =\" to get current value of the runtime variable"
                    ;

                  int aliasToActual_RuntimeVariableValue_AsInt = (int)runDelay;  // Always cast to int (needed for the lambdas).

                  if (!runtimeVariableValueIsIntQ)  // Got a valid runtime variable name so next do actions required when runtime variable value is not an integer.
                    {
                      functionSucceededQ = local_RuntimeVariableValueIsNotAnInteger_Lambda(aliasToActual_RuntimeVariableValue_AsInt);  // This is an informational lambda
                      break;
                    }
                  functionSucceededQ = local_RuntimeVariableValueIsAnInteger_Lambda(aliasToActual_RuntimeVariableValue_AsInt);  // This lambda can make changes to the actual runtime variable value

                  runDelay = (int)aliasToActual_RuntimeVariableValue_AsInt;  // Cast back to proper type needed to change state
                  break;
                }

              case StringCode::enumSoundState:  // 2nd runtime variable name that Super Token can access
                {
                  // Set up min and max values for this runtimeVariableName
                  runtimeVariableValueAsInt_Min = 0;
                  runtimeVariableValueAsInt_Max = 1;

                  // Set up the help for this specific runtime variable name
                  runtimeVariableNameInstruction = 
                    "Runtime variable \"" + runtimeVariableNameStr + "\" can be either " + 
                    String(runtimeVariableValueAsInt_Min) + " (muted) or " + String(runtimeVariableValueAsInt_Max) + " (unmuted).\n" +
                    "Send \"" + _tokenAsStr + " " + runtimeVariableNameStr + " =\" to get current value of the runtime variable"
                    ;

                  int aliasToActual_RuntimeVariableValue_AsInt = (int)soundState;  // Always cast to int (needed for the lambdas).

                  if (!runtimeVariableValueIsIntQ)  // Got a valid runtime variable name so next do actions required when runtime variable value is not an integer.
                    {
                      functionSucceededQ = local_RuntimeVariableValueIsNotAnInteger_Lambda(aliasToActual_RuntimeVariableValue_AsInt);  // This is an informational lambda
                      break;
                    }
                  functionSucceededQ = local_RuntimeVariableValueIsAnInteger_Lambda(aliasToActual_RuntimeVariableValue_AsInt);  // This lambda can make changes to the actual runtime variable value

                  soundState = (bool)aliasToActual_RuntimeVariableValue_AsInt;  // Cast back to proper type needed to change state
                  break;
                }

              case StringCode::enumBuzzerVolume:  // 3rd runtime variable name that Super Token can access
                {
                  // Set up min and max values for this runtimeVariableName
                  runtimeVariableValueAsInt_Min = 1;
                  runtimeVariableValueAsInt_Max = 10;

                  // Set up the help for this specific runtime variable name
                  runtimeVariableNameInstruction = 
                    "Runtime variable \"" + runtimeVariableNameStr + "\" must have an integer value >= " + runtimeVariableValueAsInt_Min + " (soft) but <= " + runtimeVariableValueAsInt_Max + " (loud).\n" +
                    "Send \"" + _tokenAsStr + " " + runtimeVariableNameStr + " =\" to get current value of the runtime variable"
                    ;

                  int aliasToActual_RuntimeVariableValue_AsInt = (int)buzzerVolume;  // Always cast to int (needed for the lambdas).

                  if (!runtimeVariableValueIsIntQ)  // Got a valid runtime variable name so next do actions required when runtime variable value is not an integer.
                    {
                      functionSucceededQ = local_RuntimeVariableValueIsNotAnInteger_Lambda(aliasToActual_RuntimeVariableValue_AsInt);  // This is an informational lambda
                      break;
                    }
                  functionSucceededQ = local_RuntimeVariableValueIsAnInteger_Lambda(aliasToActual_RuntimeVariableValue_AsInt);  // This lambda can make changes to the actual runtime variable value

                  buzzerVolume = (byte)aliasToActual_RuntimeVariableValue_AsInt;  // Cast back to proper type needed to change state
                  break;
                }



              /*--------------- New Runtime Variable Name Cases Go Here ---------------*/

              case StringCode::enumNoRuntimeVariableName: // Runtime variable name error case 1:  No runtime variable name given.
                {
                  printToAllPorts("Runtime variable name must not be blank.");
                  functionSucceededQ = false;
                  break;
                }

              default:                                    // Runtime variable name error case 2:  Runtime variable name given is invalid or is not one that Super Token can access.
                {
                  printToAllPorts("Runtime variable name \"" + runtimeVariableNameStr + "\" is invalid or is not one that Super Token can access.");
                  functionSucceededQ = false;
                  break;
                }
            }
        }

      default:
        {
          functionSucceededQ = false;
          break;
        }
    }
  return functionSucceededQ;
}


#pragma region -ee- BEGIN:  <doc> File Changes <TEMPLATE>

/*             Documentation of -ee- Changes In THIS File:  Updated 2025-02-01a

      CHANGES:

  - 

      ENABLED / DISABLED:

  Enabled
    - 

  Disabled
    - 

*/
#pragma endregion   END:  <doc> File Changes