/*******************************************************************************
 * Title: Reflow Oven Controller
 * Version: 1.20-cg-1.0
 * Date: 22 Feb, 2014
 * Modified By: Mark Chester
 * Company: Chester's Garage
 * Web site: http://www.chestersgarage.com
 * 
 * Updates:
 * - Added LiquidTWI library for Adafruit 12c LCD Backpack
 * - Servo-controlled door
 * - Cooling fan
 * - PID for the cooling stage
 * - Setpoint step ramp for all PIDs to handle different ovens and ambient conditions
 * - Variety of buzzer patterns for different events
 * - Variety of LED patterns for differennt events
 * - 2nd button selects between Pb/Sn and Pb-Free reflow temperature profiles
 * - Almost a total rewrite for modularity and easy changes
 * 
 * Additional Required Libraries
 * =============================
 * - LiquidTWI Library for high-performance 12c serial LCD
 *   >> http://forums.adafruit.com/viewtopic.php?f=19&t=21586&p=113177
 * 
 *******************************************************************************/

/*******************************************************************************
 * Title: Reflow Oven Controller
 * Version: 1.20
 * Date: 26-11-2012
 * Company: Rocket Scream Electronics
 * Author: Lim Phang Moh
 * Website: www.rocketscream.com
 * 
 * Brief
 * =====
 * This is an example firmware for our Arduino compatible reflow oven controller. 
 * The reflow curve used in this firmware is meant for lead-free profile 
 * (it's even easier for leaded process!). You'll need to use the MAX31855 
 * library for Arduino if you are having a shield of v1.60 & above which can be 
 * downloaded from our GitHub repository. Please check our wiki 
 * (www.rocketscream.com/wiki) for more information on using this piece of code 
 * together with the reflow oven controller shield. 
 *
 * Temperature (Degree Celcius)                 Magic Happens Here!
 * 245-|                                               x  x  
 *     |                                            x        x
 *     |                                         x              x
 *     |                                      x                    x
 * 200-|                                   x                          x
 *     |                              x    |                          |   x   
 *     |                         x         |                          |       x
 *     |                    x              |                          |
 * 150-|               x                   |                          |
 *     |             x |                   |                          |
 *     |           x   |                   |                          | 
 *     |         x     |                   |                          | 
 *     |       x       |                   |                          | 
 *     |     x         |                   |                          |
 *     |   x           |                   |                          |
 * 30 -| x             |                   |                          |
 *     |<  60 - 90 s  >|<    90 - 120 s   >|<       90 - 120 s       >|
 *     | Preheat Stage |   Soaking Stage   |       Reflow Stage       | Cool
 *  0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
 *                                                                Time (Seconds)
 *
 * This firmware owed very much on the works of other talented individuals as
 * follows:
 * ==========================================
 * Brett Beauregard (www.brettbeauregard.com)
 * ==========================================
 * Author of Arduino PID library. On top of providing industry standard PID 
 * implementation, he gave a lot of help in making this reflow oven controller 
 * possible using his awesome library.
 *
 * ==========================================
 * Limor Fried of Adafruit (www.adafruit.com)
 * ==========================================
 * Author of Arduino MAX6675 library. Adafruit has been the source of tonnes of
 * tutorials, examples, and libraries for everyone to learn.
 *
 * Disclaimer
 * ==========
 * Dealing with high voltage is a very dangerous act! Please make sure you know
 * what you are dealing with and have proper knowledge before hand. Your use of 
 * any information or materials on this reflow oven controller is entirely at 
 * your own risk, for which we shall not be liable. 
 *
 * Licences
 * ========
 * This reflow oven controller hardware and firmware are released under the 
 * Creative Commons Share Alike v3.0 license
 * http://creativecommons.org/licenses/by-sa/3.0/ 
 * You are free to take this piece of code, use it and modify it. 
 * All we ask is attribution including the supporting libraries used in this 
 * firmware. 
 *
 * Required Libraries
 * ==================
 * - Arduino PID Library: 
 *   >> https://github.com/br3ttb/Arduino-PID-Library
 * - MAX31855 Library (for board v1.60 & above): 
 *   >> https://github.com/rocketscream/MAX31855
 * - MAX6675 Library (for board v1.50 & below):  **** NO LONGER SUPPORTED IN THIS SKETCH ****
 *   >> https://github.com/adafruit/MAX6675-library
 *
 * 
 * Revision  Description
 * ========  ===========
 * 1.20      Adds supports for v1.60 (and above) of Reflow Oven Controller 
 *           Shield:
 *           - Uses MAX31855KASA+ chip and pin reassign (allowing A4 & A5 (I2C)
 *           to be used for user application).
 *           - Uses analog based switch (allowing D2 & D3 to be used for user 
 *           application).	
 *           Adds waiting state when temperature too hot to start reflow process.
 *           Corrected thermocouple disconnect error interpretation (MAX6675).
 * 1.10      Arduino IDE 1.0 compatible.
 * 1.00      Initial public release.
 *******************************************************************************/

// ########## Debug ##########
// Uncomment to enable serial data output
#define DEBUG

// ########## LCD Display Type ##########
// Uncomment only one of the following for
// serial (LiquidTWI/Adafruit i2c LCD Backpack) or
// parallel (LiquidCrystal/Direct) LCD connection
#define LCD_SERIAL
//#define LCD_PARALLEL

// ########## Features ##########
// Comment out whichever of these your oven does not have.
#define DOORSERVO
#define RGBLED
#define BUZZER
#define COOLINGFAN

// ########## Libraries ##########
#include <MAX31855.h>
#include <PID_v1.h>

#ifdef LCD_SERIAL
#include <LiquidTWI.h>
#include <Wire.h>
#else
#include <LiquidCrystal.h>
#endif

#ifdef DOORSERVO
#include <Servo.h>
#endif

// ########## Oven State ##########
boolean ovenStateInit = true;           // Flag to tell the oven state machine to initialize the new state
typedef enum OVEN_STATE {               // Defines the different states and stages of oven operation
  OVEN_STATE_IDLE,                      // Oven is doing nothing and is cooled
  OVEN_STATE_PREHEAT,                   // Preheat stage of the reflow process
  OVEN_STATE_SOAK,                      // Soak stage of the reflow process
  OVEN_STATE_REFLOW,                    // Reflow stage of the reflow process
  OVEN_STATE_COOLING,                   // Cooling stage of the reflow process
  OVEN_STATE_CAUTION,                   // The oven is idle but still hot
  OVEN_STATE_ERROR,                     // A problem with the thermocouple
} ovenState_t;
ovenState_t ovenState;                  // Reflow oven controller state machine state variable

// ########## PID Control ##########
#define TEMPERATURE_IDLE 50             // Oven maximum IDLE temperature.
#define TEMPERATURE_PREHEAT_STEP 9      // Number of degrees C per PERIOD the PREHEAT stage can heat up
#define TEMPERATURE_PREHEAT_PERIOD 3000 // Time between PREHEAT steps
#define TEMPERATURE_PREHEAT_MAX 150     // Target temperature of the PREHEAT stage
#define TEMPERATURE_SOAK_STEP 5         // Maximum number of degrees C per PERIOD the SOAK stage can heat up
#define TEMPERATURE_SOAK_PERIOD 9000    // Time between SOAK steps
#define TEMPERATURE_SOAK_MAX 200        // Target temperature of the SOAK stage
#define TEMPERATURE_REFLOW_STEP 9       // Maximum number of degrees C per PERIOD the REFLOW stage can heat up
#define TEMPERATURE_REFLOW_PERIOD 3000  // Time between REFLOW steps
#define TEMPERATURE_REFLOW_MAX 250      // Target temperature of the REFLOW stage
#define TEMPERATURE_COOLING_STEP 6      // Number of degrees C per PERIOD the COOLING stage can cool down
#define TEMPERATURE_COOLING_PERIOD 2000 // Time between COOLING steps
#define TEMPERATURE_COOLING_MIN 50      // Target temperature of the COOLING stage
#define PID_COMPUTE_PERIOD 250          // How often the PID will be evaluated
// Preheat stage PID tuning
#define PID_KP_PREHEAT 100
#define PID_KI_PREHEAT 0.025
#define PID_KD_PREHEAT 20
// Soak stage PID tuning
#define PID_KP_SOAK 300
#define PID_KI_SOAK 0.05
#define PID_KD_SOAK 250
// Reflow stage PID tuning
#define PID_KP_REFLOW 300
#define PID_KI_REFLOW 0.05
#define PID_KD_REFLOW 350
// Cooling stage PID tuning
#define PID_KP_COOLING 25
#define PID_KI_COOLING 0.0125
#define PID_KD_COOLING 5
// General PID variables
double input;                      // The input is the temperature reading form the thermocouple
double kp = PID_KP_PREHEAT;        // Since the first stage is always PREHEAT, we set it here
double ki = PID_KI_PREHEAT;
double kd = PID_KD_PREHEAT;
double setpoint;                   // The current target temperature of the PID.  This changes a lot.
double setpointMax;                // The maximum setpoint for any given stage.  Eliminates severe overshoot
byte setpointStep;                 // The number of degrees C to increment the setpoint for the current stage
int windowSize = 1000;             // The duration of the PID window in millis()
unsigned long windowStartTime;     // Time in millis() when the PID window started
unsigned long rightNow;            // Current time in millis()
unsigned long stepTimer;           // The timer used to increment setpoint through the PID ramp time
int stepTimerPeriod;               // The time in millis() to increment the timer for the current stage
typedef enum PID_STATE {           // Defines the possibel PID states
  PID_STATE_OFF,                   // PIDs are off
  PID_STATE_HEAT,                  // Heater PID is running
  PID_STATE_COOL,                  // Fan/cooling PID is running
} pidState_t;
pidState_t pidState;               // Reflow oven controller status
// Heat PID specific settings
#define ssrPin 2                   // The Arduino pin used by the SSR
double heatOutput;                 // The heat PID output variable
// Cool PID specific settings
#ifdef COOLINGFAN                  // This section used only if you have a cooling fan
#define fanPin 6                   // The Arduino pin of the cooling fan
double coolOutput;                 // The output variable of the cooling PID
#endif

// ########## LCD Display ##########
const char* lcdMessagesOvenStatus[] = {  // Defines all of the standard LCD messages
  "-- Oven Ready --",                    // Oven IDLE state
  "Stage: Preheat  ",                    // Reflow PREHEAT stage
  "Stage: Soak     ",                    // Reflow SOAK stage
  "Stage: Reflow   ",                    // Reflow REFLOW stage
  "Stage: Cooling  ",                    // Reflow COOLING stage
  "+++ OVEN HOT +++",                    // Oven CAUTION state
  "**** ERROR! ****",                    // Oven ERROR state
};
// LCD pin assignment
#ifdef LCD_PARALLEL                      // Used only for PARALLEL LCD
#define lcdRsPin 12                      // Arduino pins used for the LCD
#define lcdEPin 11
#define lcdD4Pin 5
#define lcdD5Pin 4
#define lcdD6Pin 3
#define lcdD7Pin 2
#else                                     // Used only for the SERIAL LCD
// Serial LCD always uses A4 (DAT/SDA) and A5 (CLK/SCL)
// It's a hardware requirement, cannot be changed
#define lcdI2CAddress 0                   // i2c Address of the serial LCD display
#endif
#define DISPLAY_UPDATE_PERIOD 1000        // How often to update the LCD Display
unsigned char degree[8] = {140,146,146,140,128,128,128,128}; // Creates a nifty little degree symbol
unsigned long nextDisplay;                // The time in millis() when to update the LCD

// ########## Thermocouple ##########
#define thermocoupleSOPin A3              // Themocouple serial data SO pin
#define thermocoupleCSPin A2              // Themocouple serial data CS pin              
#define thermocoupleCLKPin A1             // Themocouple serial data CLK pin
#define INPUT_READ_PERIOD 250             // How often to read the thermocouple
unsigned long nextRead;                   // When to read the thermocouple next

// ########## Reflow Profile ##########
boolean reflowProfile = 0;                // 0: Pb-Free, 1: Pb/Sn

// ########## Buttons ##########
#define buttonPin A0                      // Arduino pin used for the button input
boolean buttonDebouncing;                 // If true, we're waiting to see if the button bounces
byte buttonNumber = 0, lastButton;        // Button ID variables
int buttonReadValue;                      // Value of the analog input pin used for buttons
unsigned long buttonLockoutTime = 0;      // We won't look for a button press within this period
unsigned long lastDebounceTime;           // Switch debounce timer
typedef	enum BUTTON_STATE {               // Defines the button states
  BUTTON_STATE_NONE,                      // No button is pressed
  BUTTON_STATE_1,	                  // Button 1 is pressed; start and cancel reflow
  BUTTON_STATE_2,                         // Button 2 is pressed; select Pb/SN or Pb-Free reflow profile
} buttonState_t;
buttonState_t buttonState;                // Button press status

// ########## Door Servo ##########
#ifdef DOORSERVO                          // Only used is you have a door servo
#define DOOR_OPEN_LIMIT 38                // Servo position when the door is fully open
#define DOOR_CLOSE_LIMIT 168              // Servo position when the door is fully closed
#define doorServoPin 5                    // Arduino pin used for the servo 
boolean doorStateInit = false;            // if true, we just started opening the door
byte doorPosition;                        // Range 0-180 degrees; For my servo, lower number is open and higher is closed
byte doorMovePeriod;                      // How long to wait between increments of door movement
byte doorMoveStep;                        // How far to move the door in each increment
unsigned long doorMoveTime = 0;           // When to move the door next
typedef	enum DOOR_STATE {                 // Define the diffeerent states of the door
  DOOR_STATE_NONE,	                  // The door is not doing anything
  DOOR_STATE_RELEASE,                     // Release the door (detach the servo)
  DOOR_STATE_OPEN,	                  // The door is open
  DOOR_STATE_CLOSED,                      // The way is shut.
} doorState_t;
doorState_t doorState;                    // Door (servo) status
#endif

// ########## LED Indicator ##########
#ifdef RGBLED                             // Only used if you havd an RGB LED
#define ledRedPin 9                       // Arduino pin used for the Red LED
#define ledGreenPin 10                    // Arduino pin used for the Green LED 
#define ledBluePin 11                     // Arduino pin used for the Blue LED
boolean ledPatternInit;                   // If true, we just started runing an LED pattern
boolean ledRedState = 0, ledGreenState = 0, ledBlueState = 0; // LED is on or off
boolean ledState = HIGH;                  // Used in alternating between LED colors
unsigned long ledPatternTime = 0, ledPatternPeriod; // Timer vars for blinking or alternating LED colors
typedef enum LED_PATTERN {                // Defines LED pattern states
  LED_PATTERN_IDLE,                       // Green/Blue slow alternate
  LED_PATTERN_HEATING,                    // Red/Yellow alternate
  LED_PATTERN_COOLING,                    // Cyan/Yellow alternate
  LED_PATTERN_CAUTION,                    // Yellow fast blink
  LED_PATTERN_ERROR,                      // Red very fast blink
} ledPattern_t;
ledPattern_t ledPattern;                  // RGB LED pattern state
#endif

// ########## Buzzer ##########
#ifdef BUZZER                             // Used only if you have a buzzer
#define buzzerPin 3                       // Arduino pin used for the buzzer
boolean buzzerState = LOW;                // Buzzer is off
boolean buzzerPatternInit = true;         // If true, we've just started sounding the buzzer
byte buzzerCycles;                        // How many beeps the buzzer makes
byte buzzerCycleCount;                    // how many beeps the buzzer has made so far
int buzzerPatternPeriod;                  // how long between buzzer beeps
unsigned long buzzerPatternTime;          // When to make the next beep
typedef enum BUZZER_PATTERN {             // Defines the buzzer pattern states
  BUZZER_PATTERN_NONE,                    // The buzzer stays off
  BUZZER_PATTERN_IDLE,                    // One long beep
  BUZZER_PATTERN_START_REFLOW,            // Two short beeps
  BUZZER_PATTERN_END_REFLOW,              // Three long beeps
  BUZZER_PATTERN_CAUTION,                 // Five short beeps
  BUZZER_PATTERN_ERROR,                   // Seven very short beeps
} buzzerPattern_t;
buzzerPattern_t buzzerPattern;            // Buzzer pattern state
#endif

// ########## Interfaces ##########
// PID control
PID heatPID(&input, &heatOutput, &setpoint, kp, ki, kd, DIRECT);  // The heater PID runs in the normal direction
#ifdef COOLINGFAN                         // Only used if you have a fan
PID coolPID(&input, &coolOutput, &setpoint, kp, ki, kd, REVERSE); // The cooling PID runs reverse: more fan = lower temperature
#endif
// LCD
#ifdef LCD_SERIAL                         // Only used if you have a SERIAL LCD display
LiquidTWI lcd(lcdI2CAddress);             // Initialize serial LCD interface
#else                                     // Only if you have a PARALLEL LCD display
LiquidCrystal lcd(lcdRsPin, lcdEPin, lcdD4Pin, lcdD5Pin, lcdD6Pin, lcdD7Pin); // Initialize the parallel LCD interface
#endif
// Thermocouple
MAX31855 thermocouple(thermocoupleSOPin, thermocoupleCSPin, thermocoupleCLKPin); // Initialize the thermocouple interface
// Door servo
#ifdef DOORSERVO                          // Only if you have a door servo
Servo doorServo;                          // Initialize the servo interface
#endif

// ##########  ##########
void setup(){
  // For pin initialization, we set safe/preferred logic level before setting mode
  digitalWrite(ssrPin, LOW);       // SSR pin initialization to ensure reflow oven is off
  pinMode(ssrPin, OUTPUT);         // Now it's safe to set the pin to OUPTPUT
#ifdef BUZZER                             
  digitalWrite(buzzerPin, LOW);    // Buzzer pin initialization to ensure annoying buzzer is off
  pinMode(buzzerPin, OUTPUT);      // Now it's safe to set the pin to OUPTPUT
#endif
#ifdef DOORSERVO
  pinMode(doorServoPin, OUTPUT);   // Now it's safe to set the pin to OUPTPUT
#endif
#ifdef COOLINGFAN
  digitalWrite(fanPin, LOW);       // Fan pin 
  pinMode(fanPin, OUTPUT);         // Now it's safe to set the pin to OUPTPUT
#endif
#ifdef RGBLED
  digitalWrite(ledRedPin, HIGH);   // LED pins initialization and turn off at start-up (active low)
  pinMode(ledRedPin, OUTPUT);      // Now it's safe to set the pins to OUPTPUT
  digitalWrite(ledGreenPin, HIGH); 
  pinMode(ledGreenPin, OUTPUT);
  digitalWrite(ledBluePin, HIGH);
  pinMode(ledBluePin, OUTPUT);
#endif
  // initialize LCD display and show startup info
  lcd.begin(16, 2);              // Using a 16x2 LCD display
  lcd.createChar(0, degree);     // Create a special "Degrees" symbol
  lcd.clear();                   // Make sure there's not display garbage
  lcd.setCursor(0,0);            // Show splash screen
  lcd.print("Reflow Oven");
  lcd.setCursor(0,1);
  lcd.print("Chester's Garage");
  delay(2000);                   // Sit on the sdtartup info for a moment
  lcd.clear();                   // And then clear for operational info
#ifdef DEBUG
  Serial.begin(57600);           // Initialize serial communication at 57600 bps
  // Send header for CSV file
  // If you change these, update displaySerial() also
  Serial.println("Input Setpoint Output");
#endif
  nextRead = millis();           // Initialize thermocouple reading variable
  nextDisplay = millis();        // Initialize LCD/Serial display update variable
  ovenStateInit = true;          // Initialize the oven state
  ovenState = OVEN_STATE_IDLE;   // Set to state IDLE
#ifdef RGBLED
  ledPatternInit = true;         // Initialize the LED pattern
  ledPattern = LED_PATTERN_IDLE; // Set pattern to IDLE
#endif
#ifdef DOORSERVO
  doorStateInit = true;          // Initialize the door state
  doorState = DOOR_STATE_NONE;   // Set door state to inactive
#endif
#ifdef BUZZER
  buzzerPatternInit = true;      // Initialize the buzzer pattern
  buzzerPattern = BUZZER_PATTERN_NONE; // Set the buzzer pattern to off
#endif
}

// ########## Main Loop ##########
void loop(){
  readButtons();                          // Read the state of the buttons
  if ( millis() > nextRead ) {            // If it's time to read the temperature
    nextRead += INPUT_READ_PERIOD;        // Set the next read time
    getOvenTemperature();                 // Read the thermocouple temperature and error conditions
  }
#ifdef DOORSERVO
  controlDoor();                          // Check/update the door
#endif
  computePID();                           // Determine whether we need to be hotter or cooler
  runOvenState();                         // Set up stages and adjust input, output and display info
  if ( millis() > nextDisplay ) {         // If it's time to update the LCD
    nextDisplay += DISPLAY_UPDATE_PERIOD; // Set the next update time
    updateLCDDisplay();                   // Write info to the LCD display
#ifdef DEBUG
    updateSerial();                       // Write info to serial
#endif
  }
#ifdef BUZZER
  activateBuzzer();                       // Check/update the buzzer
#endif
#ifdef RGBLED                             
  updateLed();                            // Check/update the RGB LED
#endif
}

// ########## Door Servo Functions ##########
#ifdef DOORSERVO                                     // Open and close the door
void controlDoor(){
  switch (doorState){
    case DOOR_STATE_NONE:
      // Intentional 'do nothing' state
    break;
  
    case DOOR_STATE_RELEASE:                         // After the door opens or closes, we release the servo
      doorServo.detach();                            // Detach the servio
      doorStateInit = true;                          // Stop movement
      doorState = DOOR_STATE_NONE;                   // Done
    break;
    
    case DOOR_STATE_OPEN:
    // My servo has a high position value when the door is closed
    // and a low position value when the door is open.
      if ( doorStateInit ){                           // If we are initializing
        doorStateInit = false;                        // Flip the initialize bit
        doorMovePeriod = 50;                          // Set the step timer increment
        doorMoveStep = 1;                             // Set the movement increment
        doorServo.attach(doorServoPin);               // Attach the servo
        doorPosition = DOOR_CLOSE_LIMIT;              // Set the current (starting) position
        doorMoveTime = millis() + doorMovePeriod;     // Increment the door step timer
      } else {                                        // If the door is already initialized
        if ( millis() >= doorMoveTime ){              // Then check to see if it's time to move it some more
          doorMoveTime = millis() + doorMovePeriod;   // Increment the step timer
          doorPosition = doorPosition - doorMoveStep; // Increment the position
          if ( doorPosition > DOOR_OPEN_LIMIT ){      // Make sure it's not at the open limit
            doorServo.write(doorPosition);            // Move the door a little more
          } else {                                    // If it is at the open limit
            doorState = DOOR_STATE_RELEASE;           // Release servo control of the door.
          }
        }
      }
    break;
    
    case DOOR_STATE_CLOSED:
      if ( doorStateInit ){                           // If we are initializing
        doorStateInit = false;                        // Flip the initialize bit
        doorMovePeriod = 25;                          // Set the step timer increment
        doorMoveStep = 1;                             // Set the movement increment
        doorServo.attach(doorServoPin);               // Attach the servo
        doorPosition = DOOR_OPEN_LIMIT;               // Set the current (starting) position
        doorMoveTime = millis() + doorMovePeriod;     // Increment the door step timer
      } else {                                        // If the door is already initialized
        if ( millis() >= doorMoveTime ){              // Then check to see if it's time to move it some more
          doorMoveTime = millis() + doorMovePeriod;   // Increment the step timer
          doorPosition = doorPosition + doorMoveStep; // Increment the position
          if ( doorPosition < DOOR_CLOSE_LIMIT ){     // Make sure it's not at the closed limit
            doorServo.write(doorPosition);            // Move the door a little more
          } else {                                    // If it is at the closed limit
            doorState = DOOR_STATE_RELEASE;           // Release servo control of the door.
          }
        }
      }
    break;
  }
}
#endif

// ########## LED Indicator Functions ##########
#ifdef RGBLED
void updateLed() {                              
  // My RGB LED is common anode (+), so LOW=on, HIGH=off
  switch (ledPattern){
    case LED_PATTERN_IDLE:                   // Green/Blue slow alternate
      if ( ledPatternInit ){                 // If we're initializing
        ledPatternInit = false;              // Flip the initialize bit
        ledPatternPeriod = 2000;             // Set the pattern period
        ledState = LOW;                      // Initial state is On
        ledPatternTime = millis() + ledPatternPeriod; // Increment the pattern timer
      } else {                               // If we're not initializing
        checkLedState();                     // Check the timer and flip the led state
        digitalWrite(ledRedPin, HIGH);       // Set the LED states
        digitalWrite(ledGreenPin, ledState); 
        digitalWrite(ledBluePin, !ledState);
      }
    break;
    
    case LED_PATTERN_HEATING:                // Red/Yellow alternate
      if ( ledPatternInit ){
        ledPatternInit = false;
        ledPatternPeriod = 500;
        ledState = LOW;
        ledPatternTime = millis() + ledPatternPeriod;
      } else {
        checkLedState();
        digitalWrite(ledRedPin, LOW);
        digitalWrite(ledGreenPin, ledState);
        digitalWrite(ledBluePin, HIGH);
      }
    break;
    
    case LED_PATTERN_COOLING:                // Cyan/Yellow alternate
      if ( ledPatternInit ){
        ledPatternInit = false;
        ledPatternPeriod = 500;
        ledState = LOW;
        ledPatternTime = millis() + ledPatternPeriod;
      } else {
        checkLedState();
        digitalWrite(ledRedPin, ledState);
        digitalWrite(ledGreenPin, LOW);
        digitalWrite(ledBluePin, !ledState);
      }
    break;
    
    case LED_PATTERN_CAUTION:                // Yellow fast blink
      if ( ledPatternInit ){
        ledPatternInit = false;
        ledPatternPeriod = 250;
        ledState = LOW;
        ledPatternTime = millis() + ledPatternPeriod;
      } else {
        checkLedState();
        digitalWrite(ledRedPin, ledState);
        digitalWrite(ledGreenPin, ledState);
        digitalWrite(ledBluePin, HIGH);
      }
    break;

    case LED_PATTERN_ERROR:                  // Red extra fast blink
      if ( ledPatternInit ){
        ledPatternInit = false;
        ledPatternPeriod = 100;
        ledState = LOW;
        ledPatternTime = millis() + ledPatternPeriod;
      } else {
        checkLedState();
        digitalWrite(ledRedPin, ledState);
        digitalWrite(ledGreenPin, HIGH);
        digitalWrite(ledBluePin, HIGH);
      }
    break;
  }
}

void checkLedState(){                             // Checks the timer and flips the LED state
  if ( millis() > ledPatternTime){                // If it time
    ledState = !ledState;                         // Flip the led state
    ledPatternTime = millis() + ledPatternPeriod; // Increment the timer
  }
}
#endif

// ########## Buzzer Functions ##########
#ifdef BUZZER
void activateBuzzer(){
  switch (buzzerPattern){
    case BUZZER_PATTERN_NONE:
      // Intentional 'do nothing' state
    break;
    
    // All of the buzzer patterns have a silent section at the end.
    // This allows an 'end' buzzer to sequence nicely with a 'begin' buzzer.
    case BUZZER_PATTERN_IDLE:       // One long beep
      if ( buzzerPatternInit ){     // If we're initializing
        buzzerPatternInit = false;  // Flip the initialization bit
        buzzerPatternPeriod = 1000; // Set the pattern period 
        buzzerCycles = 2;           // Beep ONCE (and one 'off' period)
        buzzerCycleCount = 1;       // Start at the beginning
        buzzerState = HIGH;         // Turn on the buzzer
        buzzerPatternTime = millis() + buzzerPatternPeriod; // Set the timer
      } else {
        playBuzzer();
      }
    break;
    
    case BUZZER_PATTERN_START_REFLOW: // Two short beeps
      if ( buzzerPatternInit ){
        buzzerPatternInit = false;
        buzzerPatternPeriod = 250;
        buzzerCycles = 4;
        buzzerCycleCount = 1;
        buzzerState = HIGH;
        buzzerPatternTime = millis() + buzzerPatternPeriod;
      } else {
        playBuzzer();
      }
    break;
    
    case BUZZER_PATTERN_END_REFLOW:   // Three long beeps
      if ( buzzerPatternInit ){
        buzzerPatternInit = false;
        buzzerPatternPeriod = 1000;
        buzzerCycles = 6;
        buzzerCycleCount = 1;
        buzzerState = HIGH;
        buzzerPatternTime = millis() + buzzerPatternPeriod;
      } else {
        playBuzzer();
      }
    break;
    
    case BUZZER_PATTERN_CAUTION:      // Five short beeps
      if ( buzzerPatternInit ){
        buzzerPatternInit = false;
        buzzerPatternPeriod = 250;
        buzzerCycles = 10;
        buzzerCycleCount = 1;
        buzzerState = HIGH;
        buzzerPatternTime = millis() + buzzerPatternPeriod;
      } else {
        playBuzzer();
      }
    break;
    
    case BUZZER_PATTERN_ERROR:        // Seven very short beeps
      if ( buzzerPatternInit ){
        buzzerPatternInit = false;
        buzzerPatternPeriod = 100;
        buzzerCycles = 14;
        buzzerCycleCount = 1;
        buzzerState = HIGH;
        buzzerPatternTime = millis() + buzzerPatternPeriod;
      } else {
        playBuzzer();
      }
    break;
  }
}

void playBuzzer(){                         // Watches for the end of the pattern and sets the buzzer on/off state
  if ( buzzerCycleCount >= buzzerCycles ){ // If we've reaced the end of the pattern
    buzzerPatternInit = true;              // Set initialization for next time
    buzzerPattern = BUZZER_PATTERN_NONE;   // Set the pattern to nothing
  }
  if ( millis() > buzzerPatternTime){      // If the pattern timer has elapsed
    buzzerCycleCount += 1;                 // Increment the cycle count
    buzzerState = !buzzerState;            // Flip the buzzer state
    buzzerPatternTime = millis() + buzzerPatternPeriod; // Increment the timer
  }
  digitalWrite(buzzerPin, buzzerState);    // Set the buzzer
}
#endif

// ########## PID Functions ##########
void managePidSetpoint(){                   // Manages the PID setpoint ramp
  if ( millis() > stepTimer ){              // If the step timer has elapsed
    stepTimer = millis() + stepTimerPeriod; // Increment the timer by the step timer period
    if ( pidState == PID_STATE_HEAT ){      // If we're in HEAT mode
      setpoint += setpointStep;             // Increment the setpoint by the setpoint step
      if ( setpoint > setpointMax ){        // When we pass the max setpoint
        setpoint = setpointMax;             // Force the setpoint back to the max
      }
    }
#ifdef COOLINGFAN
    if ( pidState == PID_STATE_COOL ){       // If we're in COOL mode
      setpoint -= setpointStep;              // Decrement the setpoint by the setpoint step
      if ( setpoint < setpointMax ){         // When we pass the min setpoint (max)
        setpoint = setpointMax;              // Force the setpoint back to min (max)
      }
    }
#endif
  }
}

void computePID() {                                    // Heat/cool PID computation and SSR/fan control
  if ( pidState == PID_STATE_HEAT ){                   // If we're in heating mode
    rightNow = millis();                               // Capture the current time
    heatPID.Compute();                                 // Compute the heating PID
    if ( rightNow > (windowStartTime + windowSize) ){  // If the PID output window has elapsed
      windowStartTime += windowSize;                   // Reset the window start time
    }
    if ( (rightNow - windowStartTime) <= heatOutput ){ // If the elsapsed time is less than the heater PID output
      digitalWrite(ssrPin, HIGH);                      // Make sure the heating element is on
    } else {
      digitalWrite(ssrPin, LOW);                       // Otherwise shut it off
    }
  } else {                                             // If we're NOT in heat node
    digitalWrite(ssrPin, LOW);                         // Turn off the heater element
  }
#ifdef COOLINGFAN
  if ( pidState == PID_STATE_COOL ){                   // If we're in cooling mode
    coolPID.Compute();                                 // Compute the cooling PID
    analogWrite(fanPin,coolOutput);                    // And set the fan speed
  } else {                                             // If we're NOT in cooling mode
    analogWrite(fanPin,0);                             // Turn off the fan
  }
#endif
}

// ########## Reflow Profile Functions ##########
void toggleReflowProfile(){        // Switched between lead and lead free reflow profiles
  reflowProfile = !reflowProfile;  // Flip the reflow profile flag
  lcd.setCursor(0,1);              // Place the LCD cursor on the second line
  if ( reflowProfile ){            // If the profile is now Pb/Sn
    lcd.print("Profile: Pb/Sn  "); // Print Pb/Sn
  }
  else{                            // Otherwise
    lcd.print("Profile: Pb-Free"); // Print Pb-Free
  }
  delay(2000);                     // Wait for 3 seconds
  lcd.clear();                     // And then wipe the screen
}

// ########## Button Functions ##########
void readButtons(){                                          // Debounce for buttons on analog input pin
  if ( millis() > buttonLockoutTime ){                       // If we're past the lockout period
    buttonReadValue = analogRead(buttonPin);                 // Read the button pin
    // Set up the array of buttons.
    // Due to the level of noise I get on the analog input,
    // I dropped the threshold for a button press very low (600).
    if ( buttonReadValue > 600 ){ buttonNumber = 0; }        // Button 0 means no button was pressed
    if ( buttonReadValue >= 0 && buttonReadValue < 510 ){ buttonNumber = 1; }  // Button 1 pressed
    if ( (buttonReadValue > 510) && (buttonReadValue < 600) ){ buttonNumber = 2; }  // button 2 pressed
    if ( (millis() - lastDebounceTime) > 50 ){               // If debounce timer has elapsed
      if ( buttonNumber != 0 && !buttonDebouncing ){         // If a button is being pressed AND we're not already debouncing
        lastDebounceTime = millis();                         // Set the debounce time
        buttonDebouncing = true;                             // Set the debounce flag
      } // If the button number changes, reset the debounce timer
      if ( buttonNumber == 0 ){                              // No button
        buttonState = BUTTON_STATE_NONE;                     // Set the button states
      }
      if ( buttonNumber == 1 ){
        buttonState = BUTTON_STATE_1;
        buttonLockoutTime = millis() + 1000;                 // If we have a valid button press, lock out other presses for 1 second
      }
      if ( buttonNumber == 2 ){
        buttonState = BUTTON_STATE_2;
        buttonLockoutTime = millis() + 1000;
      }
    }
  }
}

// ########## Thermocouple Functions ##########
void getOvenTemperature() {                       // Read the thermocouple value
  input = thermocouple.readThermocouple(CELSIUS); // Read current temperature in degrees celcius
  if ( (input == FAULT_OPEN) || (input == FAULT_SHORT_GND) || (input == FAULT_SHORT_VCC) ){ // If thermocouple problem detected
    ovenStateInit = true;                         // Flag that we are changing oven states
    ovenState = OVEN_STATE_ERROR;                 // Drop to the error state
  }
}

// ########## Serial Output Functions ##########
#ifdef DEBUG
void updateSerial() {
  // Send all stats to serial terminal
  // If you change these, update the header output in void setup() also
  Serial.print(input);
  Serial.print(" ");
  Serial.print(setpoint);
  Serial.print(" ");
  if ( pidState == PID_STATE_HEAT ){
  Serial.print(heatOutput);
#ifdef COOLINGFAN
  } else if ( pidState == PID_STATE_COOL ){
  Serial.print(coolOutput);
#endif
  } else {
  Serial.print("0");
  }
  Serial.println("");
}
#endif

// ########## LCD Display Functions ##########
void updateLCDDisplay() {                        // Updates the LCD display with the oven state and control info
  lcd.setCursor(0,0);                            // Oven state uses top row, all 16 chars
  lcd.print(lcdMessagesOvenStatus[ovenState]);   // Print the oven state
  if (ovenState == OVEN_STATE_ERROR) {           // If currently in ERROR state
    lcd.setCursor(0,1);                          // Error text uses bottom row, all 16 chars
    lcd.print(">>> TC FAULT <<<");               // Show the error text
  }
  else {                                         // Otherwise update input (temperature) and output (heatPid or coolPid) values
    lcd.setCursor(0,1);                          // Temperature uses bottom row, first 9 chars
    lcd.print(input);                            // Print current temperature
    lcd.write((uint8_t)0);                       // Print degree Celsius symbol
    lcd.print("C ");                             // Print the unit and a trailing space to keep the screen clean
    lcd.setCursor(8,1);                          // PID output uses bottom row, last 7 chars
    if ( pidState == PID_STATE_HEAT ) {          // If we're heating
      lcd.print(" H:");                          // Prefix with an "H:" to show we're heating
      lcd.print(int(heatOutput));                // Print the current heatPid windowSize value
      lcd.print("    ");                         // And a trailing space to clean up the display
#ifdef COOLINGFAN
    } else if ( pidState == PID_STATE_COOL ) {   // If we're cooling
      lcd.print(" C:");                          // Prefix with a "C:" to chow we're cooling
      lcd.print(int(coolOutput));                // Print the current coolPid output value
      lcd.print("    ");                         // And a couple trailing spaces to clean up the diplay
#endif
    } else {                                     // If PIDs are off
      lcd.print(" PID Off");                     // Indicate so
    }
  }
}

// ########## Oven State Functions ##########
void runOvenState() {
  // Reflow oven controller state machine
  switch (ovenState){
    case OVEN_STATE_IDLE:
      // Initialize the new state
      if ( ovenStateInit ){                           // If this is a new state
        ovenStateInit = false;                        // Unset the state initialization flag
#ifdef RGBLED
        ledPatternInit = true;                        // Initialize the LED pattern
        ledPattern = LED_PATTERN_IDLE;                // Set inidcator LED to IDLE
#endif
#ifdef BUZZER
        buzzerPattern = BUZZER_PATTERN_IDLE;          // Set the buzzer pattern
#endif
        pidState = PID_STATE_OFF;                     // Disable all the PIDs
#ifdef COOLINGFAN
        coolPID.SetMode(MANUAL);                      // Turn the cool PID off
#endif
        heatPID.SetMode(MANUAL);                      // Turn the he PID off
      }
      // Check the oven temp and jump to CAUTION if above TEMPERATURE_IDLE
      if ( input > TEMPERATURE_IDLE ){                // If the oven temperature is above TEMPERATURE_IDLE
        ovenStateInit = true;                         // Set the state change flag
        ovenState = OVEN_STATE_CAUTION;               // Change to the CAUTION state
        break;
      }
      // Check for button 1 press and advance to PREHEAT state
      if ( buttonState == BUTTON_STATE_1 ){           // If button 1 was presssed
        buttonState = BUTTON_STATE_NONE;              // Clear the button state
        ovenStateInit = true;                         // Set the state change flag
        ovenState = OVEN_STATE_PREHEAT;               // Proceed to preheat stage
        break;
      }
      // Check for button 2 press and toggle reflow profile between Pb/Sn and Pb-free.
      if ( buttonState == BUTTON_STATE_2 ){           // If button 2 was presssed
        buttonState = BUTTON_STATE_NONE;              // Clear the button state
        toggleReflowProfile();                        // Toggle the reflow profile
      }
      break;
  
    case OVEN_STATE_PREHEAT:
      // Initialize the new state
      if ( ovenStateInit ){
        ovenStateInit = false;
#ifdef RGBLED
        ledPatternInit = true;
        ledPattern = LED_PATTERN_HEATING;
#endif
#ifdef DOORSERVO
        doorState = DOOR_STATE_CLOSED;
#endif
#ifdef COOLINGFAN
        coolPID.SetMode(MANUAL);                      // Turn the cool PID off
#endif
        pidState = PID_STATE_HEAT;                    // Set PID to heat
        stepTimerPeriod = TEMPERATURE_PREHEAT_PERIOD; // Set the timer step period
        setpointStep = TEMPERATURE_PREHEAT_STEP;      // Set the setpoint step increment
        setpoint = input;                             // Set the initial setpoint
        setpointMax = TEMPERATURE_PREHEAT_MAX;        // Limit the setpoint
        windowStartTime = millis();                   // Initialize PID control window starting time
        heatPID.SetOutputLimits(0, windowSize);       // Tell the PID to range between 0 and the full window size
        heatPID.SetSampleTime(PID_COMPUTE_PERIOD);    // Set the PID sample time
        heatPID.SetTunings(PID_KP_PREHEAT, PID_KI_PREHEAT, PID_KD_PREHEAT); // PID parameters for preheat ramp
        heatPID.SetMode(AUTOMATIC);                   // Turn the cool PID off
#ifdef BUZZER
        buzzerPattern = BUZZER_PATTERN_START_REFLOW;  // Activate the buzzer
#endif
        stepTimer = millis() + stepTimerPeriod;       // Increment the timer by the step timer period
      }
      // Check for button 1 press and cancel reflow.
      if ( buttonState == BUTTON_STATE_1 ){           // If the button was presssed
        buttonState = BUTTON_STATE_NONE;              // Reset the button press flag
        ovenStateInit = true;                         // Flag that we are changing oven states
        ovenState = OVEN_STATE_CAUTION;               // Proceed to preheat stage
        break;
      }
      // Button 2 has no effect in this state
      // Advance to the SOAK state
      if ( input > TEMPERATURE_PREHEAT_MAX ){         // If we've reached the maximum preheat temperature
        ovenStateInit = true;                         // Flag that we are changing oven states
        ovenState = OVEN_STATE_SOAK;                  // Change to the SOAK state
        break;
      }
      managePidSetpoint();                            // Adjust the setpoint as necessary to ramp temperature
      break;
  
    case OVEN_STATE_SOAK:
      // Initialize the new state
      if ( ovenStateInit ){                        // If this is a new state
        ovenStateInit = false;                     // Unset the state change flag
        stepTimerPeriod = TEMPERATURE_SOAK_PERIOD; // Set the timer step period
        setpointStep = TEMPERATURE_SOAK_STEP;      // Set the setpoint step increment
        setpoint = input;                          // Set initial setpoint based on current temperature
        setpointMax = TEMPERATURE_SOAK_MAX + 5;    // We want to overshoot the setpoint a little to start the reflow ramp
        heatPID.SetTunings(PID_KP_SOAK, PID_KI_SOAK, PID_KD_SOAK); // PID parameters for SOAK state
        stepTimer = millis() + stepTimerPeriod;    // Increment the timer by the step timer period
    }
      // Check for button 1 press and cancel reflow.
      if ( buttonState == BUTTON_STATE_1 ){        // If the button was presssed
        buttonState = BUTTON_STATE_NONE;           // Clear the button state
        ovenStateInit = true;                      // Flag that we are changing oven states
        ovenState = OVEN_STATE_CAUTION;            // Proceed to preheat stage
        break;
      }
      // Button 2 has no effect in this state
      // Advance to the REFLOW state
      if ( input > TEMPERATURE_SOAK_MAX ){         // If we've reached the maximum preheat temperature
        ovenStateInit = true;                      // Flag that we are changing oven states
        ovenState = OVEN_STATE_REFLOW;             // Change to the soaking state
        break;
      }
      managePidSetpoint();                         // Adjust the setpoint as necessary to ramp temperature
      break; 
  
    case OVEN_STATE_REFLOW:
      // Initialize the new state
      if ( ovenStateInit ){                          // If this is a new state
        ovenStateInit = false;                       // Unset the state change flag
        stepTimerPeriod = TEMPERATURE_REFLOW_PERIOD; // Set the timer step period
        setpointStep = TEMPERATURE_REFLOW_STEP;      // Set the setpoint step increment
        setpoint = input + 5;                        // Ramp up to minimum soaking temperature
        setpointMax = TEMPERATURE_REFLOW_MAX;
        heatPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW); // PID parameters for REFLOW state
        stepTimer = millis() + stepTimerPeriod;      // Increment the timer by the step timer period
      }
      // Neither button has any effect in this state
      // Advance to the COOLING state
      if ( input > TEMPERATURE_REFLOW_MAX ){         // If we've reached the maximum preheat temperature
        ovenStateInit = true;                        // Flag that we are changing oven states
        ovenState = OVEN_STATE_COOLING;              // Change to the soaking state
        break;
      }
      managePidSetpoint();                           // Adjust the setpoint as necessary to ramp temperature
      break;   
  
    case OVEN_STATE_COOLING:
      // Initialize the new state
      if ( ovenStateInit ){                           // If this is a new state
        ovenStateInit = false;                        // Unset the state change flag
#ifdef RGBLED
        ledPatternInit = true;
        ledPattern = LED_PATTERN_COOLING;             // Set inidcator LED to IDLE
#endif
        heatPID.SetMode(MANUAL);                      // Turn the cool PID off
#ifdef DOORSERVO
        doorState = DOOR_STATE_OPEN;                  // Open the door
#endif
        pidState = PID_STATE_COOL;                    // Set PID to heat
        stepTimerPeriod = TEMPERATURE_COOLING_PERIOD; // Set the timer step period
        setpointStep = TEMPERATURE_COOLING_STEP;      // Set the setpoint step increment
        setpoint = input;                             // Ramp up to minimum soaking temperature
        setpointMax = TEMPERATURE_COOLING_MIN - 5;
#ifdef COOLINGFAN
        coolPID.SetOutputLimits(0, 255);              // Tell the PID to range between 0 and the full window size
        coolPID.SetSampleTime(PID_COMPUTE_PERIOD);    // Set the PID sample time
        coolPID.SetTunings(PID_KP_COOLING, PID_KI_COOLING, PID_KD_COOLING); // PID parameters for COOLING state
        coolPID.SetMode(AUTOMATIC);                   // Turn the cool PID off
        stepTimer = millis() + stepTimerPeriod;       // Increment the timer by the step timer period
#endif
    }
      // Neither button has any effect in this state
      // Advance to the IDLE state
      if ( input < TEMPERATURE_COOLING_MIN ){         // If we've reached the maximum preheat temperature
#ifdef BUZZER
        buzzerPattern = BUZZER_PATTERN_END_REFLOW;    // Activate the buzzer
#endif
        ovenStateInit = true;                         // Flag that we are changing oven states
        ovenState = OVEN_STATE_IDLE;                  // Change to the soaking state
        break;
      }
      managePidSetpoint();                            // Adjust the setpoint as necessary to ramp temperature
      break;    
  
    case OVEN_STATE_CAUTION:
      // Initialize the new state
      if ( ovenStateInit ){                     // If this is a new state
        ovenStateInit = false;                  // Unset the state change flag
#ifdef RGBLED
        ledPatternInit = true;
        ledPattern = LED_PATTERN_CAUTION;       // Set inidcator LED to IDLE
#endif
        heatPID.SetMode(MANUAL);                // Turn the cool PID off
#ifdef DOORSERVO
        doorState = DOOR_STATE_OPEN;            // Open the door
#endif
        pidState = PID_STATE_COOL;              // Set PID to heat
        setpoint = TEMPERATURE_IDLE - 5;        // We want to cool as quickly as possible
#ifdef COOLINGFAN
        coolPID.SetTunings(PID_KP_COOLING, PID_KI_COOLING, PID_KD_COOLING); // PID parameters for preheat ramp
        coolPID.SetMode(AUTOMATIC);             // Turn the cool PID off
#endif
#ifdef BUZZER
        buzzerPatternInit = true;
        buzzerPattern = BUZZER_PATTERN_CAUTION; // Activate the buzzer
#endif
        stepTimer = millis() + stepTimerPeriod; // Increment the timer by the step timer period
    }
      // Return to the IDLE state
      if ( input < TEMPERATURE_IDLE ){          // If we've reached the maximum preheat temperature
        ovenStateInit = true;                   // Flag that we are changing oven states
        ovenState = OVEN_STATE_IDLE;            // Change to the soaking state
        break;
      }
      break;
  
    case OVEN_STATE_ERROR:
      // Initialize the new state
      if ( ovenStateInit ){                   // If this is a new state
        ovenStateInit = false;                // Unset the state change flag
#ifdef RGBLED
        ledPatternInit = true;
        ledPattern = LED_PATTERN_ERROR;       // Set inidcator LED to IDLE
#endif
#ifdef COOLINGFAN
        coolPID.SetMode(MANUAL);              // Turn the cool PID off
#endif
        heatPID.SetMode(MANUAL);              // Turn the cool PID off
#ifdef DOORSERVO
        doorState = DOOR_STATE_RELEASE;       // Release the door
#endif
        pidState = PID_STATE_OFF;             // Set PID to heat
#ifdef BUZZER
        buzzerPattern = BUZZER_PATTERN_ERROR; // Activate the buzzer
#endif
      }
      // Recheck error condition
      if ((input == FAULT_OPEN) || (input == FAULT_SHORT_GND) || (input == FAULT_SHORT_VCC)) { // If any of the error conditions still exist
        ovenState = OVEN_STATE_ERROR;         // Maintain the error condition
      }
      else {                                  // Otherwise
        ovenStateInit = true;                 // Flag that we are changing oven states
        ovenState = OVEN_STATE_IDLE;          // Return to IDLE state
      }
      break;    
  }
}
