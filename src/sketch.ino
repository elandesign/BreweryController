#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal.h>

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

LiquidCrystal lcd(LCD_RS, LCD_ENABLE, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// EEPROM addresses for persisted data
const int setpointAddress = 0;

void setup() {
  lcd.begin(16, 2);

  pinMode(ROT_A, INPUT_PULLUP);
  pinMode(ROT_B, INPUT_PULLUP);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, LOW);

  attachInterrupt(ROT_INTERRUPT, rotateISR, CHANGE);
  attachInterrupt(BUTTON_INTERRUPT, pushISR, RISING);
  clicked = false;

  sensors.begin();
  if (!sensors.getAddress(tempSensor, 0))
  {
     lcd.setCursor(0, 1);
     lcd.print("Sensor Error");
     delay(5000);
  }
  sensors.setResolution(tempSensor, 12);
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();

  // Initialize the PID and related variables
  LoadParameters();

  displayState();
  displayMode();
  displayCurrentTemperature();
  displayTargetTemperature();
}

void loop() {
  if (sensors.isConversionAvailable(tempSensor)) {
    currentTemperature = sensors.getTempC(tempSensor);
    sensors.requestTemperatures(); // prime the pump for the next one - but don't wait

    displayCurrentTemperature();
  }

  if(rotate != 0) {
    targetTemperature = constrain(targetTemperature + rotate, 0, 100);
    rotate = 0;

    displayTargetTemperature();
  }

  if(clicked) {
    running = !running;
    clicked = false;

    displayMode();
  }


  if(running && currentTemperature < targetTemperature && !on) {
    on = true;
    digitalWrite(RELAY, HIGH);
    displayState();
  }
  else if ((on && currentTemperature >= targetTemperature) || !running) {
    on = false;
    digitalWrite(RELAY, LOW);
    displayState();
  }
}

void displayCurrentTemperature() {
  lcd.setCursor(0, 0);
  lcd.print(currentTemperature);
  lcd.print("\337C");
}

void displayTargetTemperature() {
  lcd.setCursor(8, 0);
  lcd.print(targetTemperature);
  lcd.print("\337C ");
}

void displayMode() {
  lcd.setCursor(8, 1);
  if(running)
    lcd.print("RUNNING");
  else
    lcd.print("STOPPED");
}

void displayState() {
  lcd.setCursor(0, 1);
  if(on)
    lcd.print("ON ");
  else
    lcd.print("OFF");
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
  clicked = true;
}

// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void SaveParameters()
{
  if (targetTemperature != EEPROM_readDouble(setpointAddress))
    EEPROM_writeDouble(setpointAddress, targetTemperature);
}

// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters()
{
  // Load from EEPROM
  targetTemperature = EEPROM_readDouble(setpointAddress);

  // Use defaults if EEPROM values are invalid
  if (isnan(targetTemperature))
    targetTemperature = 72;
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
