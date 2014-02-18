/*******************************************************************************
 * Title: Reflow Oven Controller
 * Version: 1.20-cg-0.1
 * Date: 2 Feb, 2014
 * Modified By: Mark Chester
 * Company: Chester's Garage
 * Web site: http://www.chestersgarage.com
 * 
 * Updates:
 * - Added LiquidTWI library for Adafruit 12c LCD Backpack
 * - Servo-controlled door
 * - Cooling fan
 * - PID for the cool stage
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

// Uncomment only one of the following for serial (LiquidTWI/Adafruit i2c 
// LCD Backpack) or parallel (LiquidCrystal/Direct) LCD connection
//#define LCD_PARALLEL
#define LCD_SERIAL

// ***** INCLUDES *****
#ifdef LCD_SERIAL
#include <LiquidTWI.h>
#include <Wire.h>
#else
#include <LiquidCrystal.h>
#endif

#include <MAX31855.h>
#include <Servo.h>
#include <PID_v1.h>

// ***** TYPE DEFINITIONS *****
typedef enum REFLOW_STATE
{
  REFLOW_STATE_IDLE,
  REFLOW_STATE_PREHEAT,
  REFLOW_STATE_SOAK,
  REFLOW_STATE_REFLOW,
  REFLOW_STATE_COOL,
  REFLOW_STATE_COMPLETE,
  REFLOW_STATE_TOO_HOT,
  REFLOW_STATE_ERROR
} 
reflowState_t;

typedef enum REFLOW_STATUS
{
  REFLOW_STATUS_OFF,
  REFLOW_STATUS_ON
} 
reflowStatus_t;

typedef	enum SWITCH
{
  SWITCH_NONE,
  SWITCH_1,	
  SWITCH_2
}	
switch_t;

typedef enum DEBOUNCE_STATE
{
  DEBOUNCE_STATE_IDLE,
  DEBOUNCE_STATE_CHECK,
  DEBOUNCE_STATE_RELEASE
} 
debounceState_t;

// ***** CONSTANTS *****
#define TEMPERATURE_ROOM 50
#define TEMPERATURE_SOAK_MIN 150
#define TEMPERATURE_SOAK_MAX 200
#define TEMPERATURE_REFLOW_MAX 250
#define TEMPERATURE_COOL_MIN 100
#define SENSOR_SAMPLING_TIME 1000
#define SOAK_TEMPERATURE_STEP 5
#define SOAK_MICRO_PERIOD 9000
#define DEBOUNCE_PERIOD_MIN 50
#define DOOR_OPEN_LIMIT 38
#define DOOR_CLOSE_LIMIT 168

// ***** PID PARAMETERS *****
// ***** PRE-HEAT STAGE *****
#define PID_KP_PREHEAT 100
#define PID_KI_PREHEAT 0.025
#define PID_KD_PREHEAT 20
// ***** SOAKING STAGE *****
#define PID_KP_SOAK 300
#define PID_KI_SOAK 0.05
#define PID_KD_SOAK 250
// ***** REFLOW STAGE *****
#define PID_KP_REFLOW 300
#define PID_KI_REFLOW 0.05
#define PID_KD_REFLOW 350
// ***** COOL STAGE *****   NEED TO FIGURE OUT WHAT THESE SHOULD BE
#define PID_KP_COOL 300
#define PID_KI_COOL 0.05
#define PID_KD_COOL 350

#define PID_SAMPLE_TIME 1000

// ***** LCD MESSAGES *****
const char* lcdMessagesReflowStatus[] = {
  "-- Oven Ready --",
  "Stage: Pre-Heat ",
  "Stage: Soak     ",
  "Stage: Reflow   ",
  "Stage: Cooling  ",
  "Reflow Complete ",
  "Caution, HOT!   ",
  "**** Error! ****"
};

// ***** DEGREE SYMBOL FOR LCD *****
unsigned char degree[8]  = {
  140,146,146,140,128,128,128,128};

// ***** PIN ASSIGNMENT *****
#ifdef LCD_PARALLEL
#define lcdRsPin 12
#define lcdEPin 11
#define lcdD4Pin 5
#define lcdD5Pin 4
#define lcdD6Pin 3
#define lcdD7Pin 2
#else
// Serial always uses A4 (DAT/SDA) and A5 (CLK/SCL)
#endif

#define ssrPin 2
#define thermocoupleSOPin A3
#define thermocoupleCSPin A2
#define thermocoupleCLKPin A1
#define ledRedPin 9
#define ledGreenPin 10
#define ledBluePin 11
#define buzzerPin 3
#define switchPin A0
#define fanPin 6
#define doorServoPin 5

// ***** PID CONTROL VARIABLES *****
double setpoint;
double input;
double heatOutput;
double coolOutput;
double kp = PID_KP_PREHEAT;
double ki = PID_KI_PREHEAT;
double kd = PID_KD_PREHEAT;
int windowSize = 2000;
unsigned long windowStartTime;
unsigned long nextCheck;
unsigned long nextRead;
unsigned long timerSoak;
unsigned long buzzerPeriod;
byte doorStatus = 3; // 1: open, 2: closed, 3: unknown
byte doorPos; // For my servo, lower number is open and higher is closed

// Current time
unsigned long now;
// Reflow oven controller state machine state variable
reflowState_t reflowState;
// Reflow oven controller status
reflowStatus_t reflowStatus;
// Switch debounce state machine state variable
debounceState_t debounceState;
// Switch debounce timer
long lastDebounceTime;
// Switch press status
switch_t switchStatus;
// Seconds timer
int timerSeconds;
// i2c Addresses
#define lcdI2CAddress 0

// Specify PID control interface
PID heatPID(&input, &heatOutput, &setpoint, kp, ki, kd, DIRECT);
PID coolPID(&input, &coolOutput, &setpoint, kp, ki, kd, REVERSE);
// Specify LCD interface
#ifdef LCD_SERIAL
LiquidTWI lcd(lcdI2CAddress);
#else
LiquidCrystal lcd(lcdRsPin, lcdEPin, lcdD4Pin, lcdD5Pin, lcdD6Pin, lcdD7Pin);
#endif
// Specify thermocouple interface
MAX31855 thermocouple(thermocoupleSOPin, thermocoupleCSPin, thermocoupleCLKPin);
// Door servo
Servo doorServo;

void setup(){
  // SSR pin initialization to ensure reflow oven is off
  digitalWrite(ssrPin, LOW);
  pinMode(ssrPin, OUTPUT);

  // Buzzer pin initialization to ensure annoying buzzer is off
  digitalWrite(buzzerPin, LOW);
  pinMode(buzzerPin, OUTPUT);

  // Door servo pin mode
  doorServo.detach();
  pinMode(doorServoPin, OUTPUT);

  // Fan pin mode
  digitalWrite(fanPin, LOW);
  pinMode(fanPin, OUTPUT);

  // LED pins initialization and turn on upon start-up (active low)
  digitalWrite(ledRedPin, LOW);
  pinMode(ledRedPin, OUTPUT);
  digitalWrite(ledGreenPin, LOW);
  pinMode(ledGreenPin, OUTPUT);
  digitalWrite(ledBluePin, LOW);
  pinMode(ledBluePin, OUTPUT);

  // Start-up splash
  digitalWrite(buzzerPin, HIGH);
  lcd.begin(8, 2);
  lcd.createChar(0, degree);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Reflow Oven 1.21");
  lcd.setCursor(0, 1);
  lcd.print("Chester's Garage");
  digitalWrite(buzzerPin, LOW);
  delay(2500);
  lcd.clear();

  // Serial communication at 57600 bps
  Serial.begin(57600);

  // Initialize time keeping variable
  nextCheck = millis();
  // Initialize thermocouple reading variable
  nextRead = millis();
  // Send header for CSV file
  Serial.println("Time Setpoint Input Output reflowState");
}

void openDoor(){
  doorServo.attach(doorServoPin);
  doorServo.write(DOOR_OPEN_LIMIT+15);
  delay(750);
  for(doorPos = DOOR_OPEN_LIMIT+15; doorPos > DOOR_OPEN_LIMIT; doorPos -= 1){
    doorServo.write(doorPos);
    delay(50);
  }
  doorStatus = 1;
}

void closeDoor(){
  doorServo.attach(doorServoPin);
  doorServo.write(DOOR_CLOSE_LIMIT-15);
  delay(750);
  for(doorPos = DOOR_CLOSE_LIMIT-15; doorPos < DOOR_CLOSE_LIMIT; doorPos += 1){
    doorServo.write(doorPos);
    delay(50);
  }
  doorStatus = 2;
}

void releaseDoor(){
  doorServo.detach();
  doorStatus = 3;
}

void loop(){
  // Time to read thermocouple?
  if (millis() > nextRead){
    // Read thermocouple next sampling period
    nextRead += SENSOR_SAMPLING_TIME;
    // Read current temperature
    input = thermocouple.readThermocouple(CELSIUS);

    // If thermocouple problem detected
    if((input == FAULT_OPEN) || (input == FAULT_SHORT_GND) || (input == FAULT_SHORT_VCC)){
      // Illegal operation
      reflowState = REFLOW_STATE_ERROR;
      reflowStatus = REFLOW_STATUS_OFF;
    }
  }

  if (millis() > nextCheck){
    // Check input in the next seconds
    nextCheck += 1000;
    // If reflow process is on going
    if (reflowStatus == REFLOW_STATUS_ON){
      // Toggle red LED as system heart beat
      digitalWrite(ledRedPin, !(digitalRead(ledRedPin)));
      // Increase seconds timer for reflow curve analysis
      timerSeconds++;
    }
    else
    {
      // Turn off red LED
      digitalWrite(ledRedPin, HIGH);
    }
    // Send temperature and time stamp to serial 
    Serial.print(timerSeconds);
    Serial.print(" ");
    Serial.print(setpoint);
    Serial.print(" ");
    Serial.print(input);
    Serial.print(" ");
    Serial.print(heatOutput);
    Serial.print(" ");
    Serial.print(reflowState);
    Serial.print(" ");
    Serial.println(doorStatus);

    // Print current system state
    lcd.setCursor(0,0);
    lcd.print(lcdMessagesReflowStatus[reflowState]);
    // Move the cursor to the 2 line
    lcd.setCursor(0,1);
    // If currently in error state
    if (reflowState == REFLOW_STATE_ERROR){
      // No thermocouple wire connected
      lcd.print("*** TC ERROR ***");
    }
    else{
      // Print current temperature
      lcd.print(input);
      // Print degree Celsius symbol
      lcd.write((uint8_t)0);
      lcd.print("C ");
    }
    lcd.setCursor(9,1);
    if ( heatOutput > 0 ) {
      lcd.print("H:");
      lcd.print(int(heatOutput));
      lcd.print(" ");
    }
    if ( coolOutput > 0 ) {
      lcd.print("C:");
      lcd.print(int(coolOutput));
      lcd.print(" ");
    }
  }

  // Reflow oven controller state machine
  switch (reflowState){
  case REFLOW_STATE_IDLE:
    // Entry actions
    // Disable PIDs
    // Release door
    if ( doorStatus != 3 ){
      releaseDoor();
    }
    // Turn off fan
    digitalWrite(fanPin, LOW);
      // Long beep
      // Set inidcator LED COOL
      // Set LCD line 0 to OVEN_READY
    
    // While actions
    // Check the oven temp and jump to OVEN_HOT if above TEMPERATURE_ROOM
    if (input > TEMPERATURE_ROOM){
      reflowState = REFLOW_STATE_TOO_HOT;
    }
    // Check for button 1 press and exit to PREHEAT.
    else{
      // If switch is pressed to start reflow process
      if (switchStatus == SWITCH_1){
        // Intialize seconds timer for serial debug information
        timerSeconds = 0;
        // Initialize PID control window starting time
        windowStartTime = millis();
        // Ramp up to minimum soaking temperature
        setpoint = TEMPERATURE_SOAK_MIN;
        // Tell the PID to range between 0 and the full window size
        heatPID.SetOutputLimits(0, windowSize);
        heatPID.SetSampleTime(PID_SAMPLE_TIME);
        // Turn the PID on
        heatPID.SetMode(AUTOMATIC);
        // Proceed to preheat stage
        reflowState = REFLOW_STATE_PREHEAT;
      }
    // Check for button 2 press and toggle PID params between Pb/Sn and Pb-free.
    // Update LCD line 1 with Temperature
    
    // All exit actions
    // Short beep
    // Unset indicator LED COOL
  
    }
    break;

  case REFLOW_STATE_PREHEAT:
    // Entry actions
    // Set PID params for PREHEAT
    // Set indicator LED HOT
    // Set LCD line 0 to PREHEAT
    // Close the door
    
    // While actions
    // Check for button 1 and exit to IDLE (cancel reflow)
    // Update LCD line 1 with temperature and output
    
    // Exit actions
    // Set exit to SOAK
    // Short beep
    
    reflowStatus = REFLOW_STATUS_ON;
    // Close the door and shut off the fan before doing anything else
    digitalWrite(fanPin, LOW);
    if ( doorStatus != 2 ){
      closeDoor();
    }
    // If minimum soak temperature is achieve       
    if (input >= TEMPERATURE_SOAK_MIN){
      // Chop soaking period into smaller sub-period
      timerSoak = millis() + SOAK_MICRO_PERIOD;
      // Set less agressive PID parameters for soaking ramp
      heatPID.SetTunings(PID_KP_SOAK, PID_KI_SOAK, PID_KD_SOAK);
      // Ramp up to first section of soaking temperature
      setpoint = TEMPERATURE_SOAK_MIN + SOAK_TEMPERATURE_STEP;   
      // Proceed to soaking state
      reflowState = REFLOW_STATE_SOAK; 
    }
    break;

  case REFLOW_STATE_SOAK:
    // Entry actions
    // Set PID params for SOAK
    // Set LCD to SOAK
    
    // While actions
    // Check for button 1 and jump to IDLE (cancel reflow)
    // Update LCD with temperature
    // Update LCD with PID output
    
    // Exit actions
    // Set exit to REFLOW
    // Short beep

    // Make sure the fan is off.
    digitalWrite(fanPin, LOW);
    // If the door isn't already closed, close it.
    if ( doorStatus != 2 ){
      closeDoor();
    }
    // If micro soak temperature is achieved       
    if (millis() > timerSoak)
    {
      timerSoak = millis() + SOAK_MICRO_PERIOD;
      // Increment micro setpoint
      setpoint += SOAK_TEMPERATURE_STEP;
      if (setpoint > TEMPERATURE_SOAK_MAX)
      {
        // Set agressive PID parameters for reflow ramp
        heatPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
        // Ramp up to first section of soaking temperature
        setpoint = TEMPERATURE_REFLOW_MAX;   
        // Proceed to reflowing state
        reflowState = REFLOW_STATE_REFLOW; 
      }
    }
    break; 

  case REFLOW_STATE_REFLOW:
    // Entry actions
    // Set PID params for REFLOW
    // Set LCD to REFLOW
    
    // While actions
    // Check for button 1 and jump to IDLE (cancel reflow)
    // Update LCD with temperature
    // Update LCD with PID output
    
    // Exit actions
    // Set exit to COOL
    // Short beep

    // We need to avoid hovering at peak temperature for too long
    // Crude method that works like a charm and safe for the components
    if (input >= (TEMPERATURE_REFLOW_MAX - 2))
    {
      // Set PID parameters for cooling ramp
      heatPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
      // Ramp down to minimum cooling temperature
      setpoint = TEMPERATURE_COOL_MIN;   
      // Proceed to cooling state
      reflowState = REFLOW_STATE_COOL; 
    }
    break;   

  case REFLOW_STATE_COOL:
    // Entry actions
    // Set PID params for COOL
    // Set LCD to COOL
    
    // While actions
    // Update LCD with temperature
    // Update LCD with PID output
    
    // Exit actions
    // Set exit to IDLE
    // Turn off HOT LED
    // Short beep

    // Open the door and turn on the fan for faster cooling
    if ( doorStatus != 1 ){
      openDoor();
    }
    digitalWrite(fanPin, HIGH);
    // If minimum cool temperature is achieve       
    if (input <= TEMPERATURE_COOL_MIN)
    {
      // Retrieve current time for buzzer usage
      buzzerPeriod = millis() + 1000;
      // Turn on buzzer and green LED to indicate completion
      digitalWrite(buzzerPin, HIGH);
      // Turn off reflow process
      reflowStatus = REFLOW_STATUS_OFF;                
      // Proceed to reflow Completion state
      reflowState = REFLOW_STATE_COMPLETE; 
    }         
    break;    

  case REFLOW_STATE_COMPLETE:
    // DELETE THIS STAGE
    // Release the door and shut off the fan
    if ( doorStatus != 3 ){
      releaseDoor();
    }
    digitalWrite(fanPin, LOW);
    if (millis() > buzzerPeriod)
    {
      // Turn off buzzer and green LED
      digitalWrite(buzzerPin, LOW);
      // Reflow process ended
      reflowState = REFLOW_STATE_IDLE; 
    }
    break;

  case REFLOW_STATE_TOO_HOT:
    // Rename to OVEN_HOT

    // Entry actions
    // Set LED to CAUTION
    // Set LCD to CAUTION
    // Long beep
    // Open door
    // Turn on fan
    
    // While actions
    // Update LCD with temperature
    // Update LCD with PID output
    
    // Exit actions
    // Set exit to IDLE
    // Short beep

    // Open the door and turn on the fan to vent 
    if ( doorStatus != 1 ){
      openDoor();
    }
    digitalWrite(fanPin, HIGH);
    // If oven temperature drops below room temperature
    if (input < TEMPERATURE_ROOM)
    {
      // Ready to reflow
      reflowState = REFLOW_STATE_IDLE;
    }
    break;

  case REFLOW_STATE_ERROR:
    // Let go of the door aand shut off the fan
    if ( doorStatus != 3 ){
      releaseDoor();
    }
    digitalWrite(fanPin, LOW);
    // If thermocouple problem is still present
    if((input == FAULT_OPEN) || (input == FAULT_SHORT_GND) || 
      (input == FAULT_SHORT_VCC))
    {
      // Wait until thermocouple wire is connected
      reflowState = REFLOW_STATE_ERROR; 
    }
    else
    {
      // Clear to perform reflow process
      reflowState = REFLOW_STATE_IDLE; 
    }
    break;    
  }    

  // If switch 1 is pressed
  if (switchStatus == SWITCH_1){
    // If currently reflow process is on going
    if (reflowStatus == REFLOW_STATUS_ON)
    {
      // Button press is for cancelling
      // Turn off reflow process
      reflowStatus = REFLOW_STATUS_OFF;
      // Reinitialize state machine
      reflowState = REFLOW_STATE_IDLE;
    }
  } 

  // Simple switch debounce state machine (for switch #1 (both analog & digital
  // switch supported))
  switch (debounceState){
  case DEBOUNCE_STATE_IDLE:
    // No valid switch press
    switchStatus = SWITCH_NONE;
    // If switch #1 is pressed
    if (analogRead(switchPin) == 0)
    {
      // Intialize debounce counter
      lastDebounceTime = millis();
      // Proceed to check validity of button press
      debounceState = DEBOUNCE_STATE_CHECK;
    }	
    break;

  case DEBOUNCE_STATE_CHECK:
    if (analogRead(switchPin) == 0)
    {
      // If minimum debounce period is completed
      if ((millis() - lastDebounceTime) > DEBOUNCE_PERIOD_MIN)
      {
        // Proceed to wait for button release
        debounceState = DEBOUNCE_STATE_RELEASE;
      }
    }
    // False trigger
    else
    {
      // Reinitialize button debounce state machine
      debounceState = DEBOUNCE_STATE_IDLE; 
    }
    break;

  case DEBOUNCE_STATE_RELEASE:
    if (analogRead(switchPin) > 0)
    {
      // Valid switch 1 press
      switchStatus = SWITCH_1;
      // Reinitialize button debounce state machine
      debounceState = DEBOUNCE_STATE_IDLE; 
    }
    break;
  }

  // PID computation and SSR control
  if (reflowStatus == REFLOW_STATUS_ON)
  {
    now = millis();

    heatPID.Compute();

    if((now - windowStartTime) > windowSize) { 
      // Time to shift the Relay Window
      windowStartTime += windowSize;
    }
    if(heatOutput > (now - windowStartTime)) {
      digitalWrite(ssrPin, HIGH);
    }
    else {
      digitalWrite(ssrPin, LOW);   
    }
  }
  // Reflow oven process is off, ensure oven is off
  else 
  {
    digitalWrite(ssrPin, LOW);
  }
}
