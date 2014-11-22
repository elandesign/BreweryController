#include <TimerOne.h>
#include <EEPROM.h>
#include <SM.h>
#include <State.h>
#include <PID_v1.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal.h>
#include <PID_AutoTune_v0.h>


// Temperature Sensor
#define ONE_WIRE_BUS 6

// Rotary rotate
#define ROT_A 4
#define ROT_B 2
#define BUTTON 3
#define ROT_INTERRUPT 1
#define BUTTON_INTERRUPT 0

// LCD
#define LCD_RS A1
#define LCD_ENABLE A0
#define LCD_D4 15
#define LCD_D5 14
#define LCD_D6 16
#define LCD_D7 10

#define RELAY 5

// LED indicator
#define INDICATOR 17

// Menu Items
#define MNU_RUN 0
#define MNU_SET 1
#define MNU_Kp 2
#define MNU_Ki 3
#define MNU_Kd 4
#define MNU_AUTOTUNE 5

SM state(StoppedHead, StoppedBody);

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress tempSensor;

long rotate = 0;
boolean clicked = false;
boolean running = false;
boolean on = false;

double currentTemperature = 0;
double targetTemperature = 70;
double output = 0;

unsigned long windowStartTime;
int windowSize = 5000;
volatile long onTime = 0;

LiquidCrystal lcd(LCD_RS, LCD_ENABLE, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

double kp = 679.06;
double ki = 4.89;
double kd = 0.0;

PID pid(&currentTemperature, &output, &targetTemperature, kp, ki, kd, DIRECT);

// Autotune Variables
byte ATuneModeRemember=2;
double aTuneStep=500;
double aTuneNoise=1;
unsigned int aTuneLookBack=20;
PID_ATune aTune(&currentTemperature, &output);

// EEPROM addresses for persisted data
const int setpointAddress = 0;
const int kpAddress = 8;
const int kiAddress = 16;
const int kdAddress = 24;

void setup() {
//  sensors.begin();
  lcd.begin(16, 2);

  pinMode(ROT_A, INPUT_PULLUP);
  pinMode(ROT_B, INPUT_PULLUP);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(RELAY, OUTPUT);
  pinMode(INDICATOR, OUTPUT);
  digitalWrite(RELAY, LOW);
  digitalWrite(INDICATOR, LOW);

  windowStartTime = millis();
  pid.SetOutputLimits(0, windowSize);
  pid.SetMode(MANUAL);

  attachInterrupt(ROT_INTERRUPT, rotateISR, CHANGE);
  attachInterrupt(BUTTON_INTERRUPT, pushISR, RISING);
  clicked = false;

  sensors.begin();
  if (!sensors.getAddress(tempSensor, 0))
  {
     lcd.setCursor(0, 1);
     lcd.print(F("Sensor Error"));
  }
  sensors.setResolution(tempSensor, 12);
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();

  Serial.begin(9600);

  // Splash Screen
  delay(1000);

  // Initialize the PID and related variables
  LoadParameters();
  pid.SetTunings(kp,ki,kd);

  pid.SetSampleTime(1000);
  pid.SetOutputLimits(0, windowSize);

  Timer1.initialize(15000);
  Timer1.attachInterrupt(TimerInterrupt);
}

void commonLoop() {
  if (sensors.isConversionAvailable(tempSensor))
  {
    digitalWrite(INDICATOR, HIGH);
    currentTemperature = sensors.getTempC(tempSensor);
    digitalWrite(INDICATOR, LOW);
    sensors.requestTemperatures(); // prime the pump for the next one - but don't wait
  }

  showState();
}

void loop() {
  EXEC(state);
}

/********************************
* Interrupt Routines
*********************************/
void rotateISR() {
  int a = digitalRead(ROT_A);
  int b = digitalRead(ROT_B);
  if(a == 1 && b == 0)
    rotate = 1;
  else if (a == 1 && b == 1)
    rotate = -1;
}

void pushISR() {
  Serial.print("clicked!");
  clicked = true;
}


/********************************
* Main Menu
*********************************/
State StoppedHead() {
//  pid.SetMode(MANUAL);
  digitalWrite(RELAY, LOW);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Stop    ");
  running = false;
}

State StoppedBody() {
  commonLoop();
  static int menu = 0;

  lcd.setCursor(0, 0);
  switch(menu) {
    case MNU_RUN:
      lcd.print("Run     ");
      lcd.setCursor(9, 0);
      lcd.print(targetTemperature);
      lcd.print("\337C");
      break;
    case MNU_SET:
      lcd.print("Set T   ");
      lcd.setCursor(9, 0);
      lcd.print(targetTemperature);
      lcd.print("\337C");
      break;
    case MNU_Kp:
      lcd.print("Set Kp  ");
      lcd.setCursor(9, 0);
      lcd.print(kp);
      break;
    case MNU_Ki:
      lcd.print("Set Ki  ");
      lcd.setCursor(9, 0);
      lcd.print(ki);
      break;
    case MNU_Kd:
      lcd.print("Set Kd  ");
      lcd.setCursor(9, 0);
      lcd.print(kd);
      break;
    case MNU_AUTOTUNE:
      lcd.print("Autotune");
      lcd.setCursor(9, 0);
      lcd.print(targetTemperature);
      lcd.print("\337C");
      break;
  }

  if(rotate != 0) {
    lcd.clear();
    menu = constrain(menu + rotate, MNU_RUN, MNU_AUTOTUNE);
    rotate = 0;
  }

  if(clicked) {
    switch(menu) {
      case MNU_RUN:
        state.Set(RunningHead, RunningBody);
        break;
      case MNU_SET:
        state.Set(SetpointHead, SetpointBody);
        break;
      case MNU_Kp:
        state.Set(SetKpHead, SetKpBody);
        break;
      case MNU_Ki:
        state.Set(SetKiHead, SetKiBody);
        break;
      case MNU_Kd:
        state.Set(SetKdHead, SetKdBody);
        break;
      case MNU_AUTOTUNE:
        state.Set(AutotuneHead, AutotuneBody);
        break;
    }
    clicked = false;
  }
}

/********************************
* Run Mode
*********************************/
State RunningHead() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Running ");
  lcd.setCursor(9, 0);
  lcd.print(targetTemperature);
  lcd.print("\337C");
  pid.SetTunings(kp, ki, kd);
  pid.SetMode(AUTOMATIC);
  running = true;
}

State RunningBody() {
  commonLoop();
  pid.Compute();
  onTime = output;

  lcd.setCursor(9, 1);
  lcd.print(currentTemperature);
  lcd.print("\337C");

  if(clicked) {
    pid.SetMode(MANUAL);
    digitalWrite(RELAY, LOW);
    state.Set(StoppedHead, StoppedBody);
    clicked = false;
  }
}

State AutotuneHead() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Tuning");
  lcd.setCursor(9, 0);
  lcd.print(targetTemperature);
  lcd.print("\337C");
  // Remember the mode we were in
  ATuneModeRemember = pid.GetMode();

  // set up the auto-tune parameters
  aTune.SetNoiseBand(aTuneNoise);
  aTune.SetOutputStep(aTuneStep);
  aTune.SetLookbackSec((int)aTuneLookBack);
  running = true;
}

State AutotuneBody() {
  commonLoop();
  lcd.setCursor(9, 1);
  lcd.print(currentTemperature);
  lcd.print("\337C");

  if (aTune.Runtime()) // returns 'true' when done
  {
    state.Set(FinishAutoTune);
  }

  // Time Proportional relay state is updated regularly via timer interrupt.
  onTime = output;

  if(clicked) {
    state.Set(StoppedHead, StoppedBody);
    clicked = false;
  }
}

State FinishAutoTune()
{
   // Extract the auto-tune calculated parameters
   kp = aTune.GetKp();
   ki = aTune.GetKi();
   kd = aTune.GetKd();

   // Re-tune the PID and revert to normal control mode
   pid.SetTunings(kp,ki,kd);
   pid.SetMode(ATuneModeRemember);

   // Persist any changed parameters to EEPROM
   SaveParameters();

   state.Set(StoppedHead, StoppedBody);
}


/********************************
* Setpoint Adjustment
*********************************/
State SetpointHead() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Target");
  lcd.setCursor(9, 0);
  lcd.print(targetTemperature);
  lcd.print("\337C");
}

State SetpointBody() {
  commonLoop();
  if(rotate != 0) {
    targetTemperature = constrain(targetTemperature + rotate, 0, 100);
    rotate = 0;
  }

  lcd.setCursor(9, 0);
  lcd.print(targetTemperature);
  lcd.print("\337C");

  if(clicked) {
    state.Set(StoppedHead, StoppedBody);
    clicked = false;
  }
}


/********************************
* Set Kp
*********************************/
State SetKpHead() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Kp");
  lcd.setCursor(9, 0);
  lcd.print(kp);
}

State SetKpBody() {
  commonLoop();
  if(rotate != 0) {
    kp = constrain(kp + (rotate / 10.0), 0, 1000000);
    rotate = 0;
  }

  lcd.setCursor(9, 0);
  lcd.print(kp);

  if(clicked) {
    pid.SetTunings(kp, ki, kd);
    state.Set(StoppedHead, StoppedBody);
    clicked = false;
  }
}


/********************************
* Set Ki
*********************************/
State SetKiHead() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Ki");
  lcd.setCursor(9, 0);
  lcd.print(ki);
}

State SetKiBody() {
  commonLoop();
  if(rotate != 0) {
    ki = constrain(ki + (rotate / 10.0), 0, 1000000);
    rotate = 0;
  }

  lcd.setCursor(9, 0);
  lcd.print(ki);

  if(clicked) {
    pid.SetTunings(kp, ki, kd);
    state.Set(StoppedHead, StoppedBody);
    clicked = false;
  }
}


/********************************
* Set Kd
*********************************/
State SetKdHead() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Kd");
  lcd.setCursor(9, 0);
  lcd.print(kd);
}

State SetKdBody() {
  commonLoop();
  if(rotate != 0) {
    kd = constrain(kd + (rotate / 10.0), 0, 1000000);
    rotate = 0;
  }

  lcd.setCursor(9, 0);
  lcd.print(kd);

  if(clicked) {
    pid.SetTunings(kp, ki, kd);
    state.Set(StoppedHead, StoppedBody);
    clicked = false;
  }
}



// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void SaveParameters()
{
  if (targetTemperature != EEPROM_readDouble(setpointAddress))
    EEPROM_writeDouble(setpointAddress, targetTemperature);
  if (kp != EEPROM_readDouble(kpAddress))
    EEPROM_writeDouble(kpAddress, kp);
  if (ki != EEPROM_readDouble(kiAddress))
    EEPROM_writeDouble(kiAddress, ki);
  if (kd != EEPROM_readDouble(kdAddress))
    EEPROM_writeDouble(kdAddress, kd);
}

// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters()
{
  // Load from EEPROM
   targetTemperature = EEPROM_readDouble(setpointAddress);
   kp = EEPROM_readDouble(kpAddress);
   ki = EEPROM_readDouble(kiAddress);
   kd = EEPROM_readDouble(kdAddress);

   // Use defaults if EEPROM values are invalid
   if (isnan(targetTemperature))
     targetTemperature = 72;
   if (isnan(kp))
     kp = 850;
   if (isnan(ki))
     ki = 0.5;
   if (isnan(kd))
     kd = 0.1;
}

// ************************************************
// Write floating point values to EEPROM
// ************************************************
void EEPROM_writeDouble(int address, double value)
{
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
    EEPROM.write(address++, *p++);
}

// ************************************************
// Read floating point values from EEPROM
// ************************************************
double EEPROM_readDouble(int address)
{
  double value = 0.0;
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
    *p++ = EEPROM.read(address++);
   return value;
}

void TimerInterrupt()
{
  if (!running)
  {
    digitalWrite(RELAY, LOW);  // make sure relay is off
    on = false;
  }
  else
  {
    DriveOutput();
  }
}

// ************************************************
// Called by ISR every 15ms to drive the output
// ************************************************
void DriveOutput()
{
  long now = millis();
  // Set the output
  // "on time" is proportional to the PID output
  if(now - windowStartTime > windowSize)
  { //time to shift the Relay Window
    windowStartTime += windowSize;
  }
  if((onTime > 100) && (onTime > (now - windowStartTime))) {
    digitalWrite(RELAY, HIGH);
    on = true;
  }
  else {
    digitalWrite(RELAY, LOW);
    on = false;
  }
}

void showState() {
  lcd.setCursor(0, 1);
  if(on)
    lcd.print("On ");
  else
    lcd.print("Off");

}
