#include <AccelStepper.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <HX711.h>
#include <Stepper.h>

// Define pins
#define PUL_PIN 26
#define DIR_PIN 27
#define ENA_PIN 25
#define SOLENOID_PIN 15
#define RED_LED_PIN 5
#define HX711_SCK_PIN 4
#define HX711_DT_PIN 16
#define TEMP_PIN 34
#define IN1 12
#define IN2 14
#define IN3 33
#define IN4 32
#define GREEN_LED_PIN 13  // Green LED pin

// Leadscrew and stepper parameters (TB6600 for Z-axis)
const float MM_PER_REV = 8.0;
const int STEPS_PER_REV = 3200;
const float STEPS_PER_MM = STEPS_PER_REV / MM_PER_REV;

// Fixed distances (Z-axis)
const float TARGET_Z = 100.0;
const float MAX_TRAVEL = 200.0;

// Refill system parameters
const int MAX_Z_ATTEMPTS = 2;
int zAttemptCount = 0;
const int MAX_REFILL_ATTEMPTS = 2;
int refillAttempts = 0;
const float STARTUP_FUEL_THRESHOLD = 100.0;
const float OPERATION_FUEL_THRESHOLD_OFFSET = 50.0;
float fuelThreshold = STARTUP_FUEL_THRESHOLD;
float calibrationFactor = -2200.87;
float fuelWeight = 0.0;
float referenceWeight = 0.0;
bool startupComplete = false;

// Temperature control parameters
const float MIN_TEMP = 70.0;
const float MAX_TEMP = 80.0;
const int SAMPLE_INTERVAL = 5000;
const int WINDOW_SIZE = 60;
float tempBuffer[WINDOW_SIZE];
int bufferIndex = 0;
bool bufferFull = false;
float tempSum = 0.0;
int tempCount = 0;
float temperature = 0.0;
const float FUEL_CONSUMPTION_RATE = 1.0;

// Stepper motor parameters (28BYJ-48 for rack-and-pinion)
const int STEPS_PER_REVOLUTION = 2048;
const int MOTOR_SPEED = 15;
const float REVOLUTIONS_FULL = 3.0;
float currentLeverPosition = 0.0;

// Initialize HX711, AccelStepper, LCD, and Stepper
HX711 scale;
AccelStepper stepper(AccelStepper::DRIVER, PUL_PIN, DIR_PIN);
LiquidCrystal_I2C lcd(0x27, 16, 2);
Stepper myStepper(STEPS_PER_REVOLUTION, IN1, IN3, IN2, IN4);

// Current Z position
float zPosition = 0.0;

// Function prototypes
void moveUpwards();
void moveDownwards();
void moveToZero();
bool performRefill();
void indicateRefillNeeded();
float readFuelWeight();
void displayFuelWeight();
float getAverageTemperature(float rawTempC);
void adjustAirflow();
void moveLeverTo(float targetRevolutions);
void updateReferenceWeight();

void setup() {
  pinMode(ENA_PIN, OUTPUT);
  pinMode(SOLENOID_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(TEMP_PIN, INPUT);
  
  digitalWrite(ENA_PIN, LOW);
  digitalWrite(SOLENOID_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Ready");

  scale.begin(HX711_DT_PIN, HX711_SCK_PIN);
  scale.set_scale(calibrationFactor);
  // No tare, using pre-calibrated factor
  
  stepper.setMaxSpeed(5000);
  stepper.setAcceleration(200);
  myStepper.setSpeed(MOTOR_SPEED);
  
  // Initialize temperature buffer
  for (int i = 0; i < WINDOW_SIZE; i++) {
    tempBuffer[i] = 0.0;
  }
  
  fuelWeight = readFuelWeight();
  displayFuelWeight();
  referenceWeight = fuelWeight;
  
  fuelThreshold = STARTUP_FUEL_THRESHOLD;
  startupComplete = false;
  
  if (fuelWeight < STARTUP_FUEL_THRESHOLD) {
    lcd.setCursor(0, 0);
    lcd.print("Startup Refill  ");
    bool refillSuccess = performRefill();
    
    if (!refillSuccess) {
      indicateRefillNeeded();
      while (fuelWeight < STARTUP_FUEL_THRESHOLD) {
        fuelWeight = readFuelWeight();
        displayFuelWeight();
        delay(1000);
      }
      updateReferenceWeight();
    }
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("IGNITE");
    lcd.setCursor(0, 1);
    lcd.print("Fuel: ");
    lcd.print(fuelWeight, 1);
    lcd.print("g");
    delay(10000);
    digitalWrite(GREEN_LED_PIN, HIGH);
  }
  
  startupComplete = true;
  fuelThreshold = referenceWeight - OPERATION_FUEL_THRESHOLD_OFFSET;
}

void loop() {
  int adcVal = analogRead(TEMP_PIN);
  float milliVolt = (adcVal / 4096.0) * 3300.0;
  float rawTempC = milliVolt / 10.0;
  temperature = getAverageTemperature(rawTempC);
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(temperature, 1);
  lcd.print("C");
  displayFuelWeight();
  
  if (fuelWeight < fuelThreshold) {
    bool refillSuccess = performRefill();
    if (!refillSuccess) {
      indicateRefillNeeded();
      while (fuelWeight < fuelThreshold) {
        fuelWeight = readFuelWeight();
        displayFuelWeight();
        delay(1000);
      }
      updateReferenceWeight();
      zAttemptCount = 0;
      refillAttempts = 0;
      digitalWrite(RED_LED_PIN, LOW);
      digitalWrite(GREEN_LED_PIN, LOW);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Refill detected");
      displayFuelWeight();
      delay(2000);
    }
  } else {
    adjustAirflow();
    fuelWeight -= FUEL_CONSUMPTION_RATE;
    lcd.setCursor(0, 0);
    lcd.print("Fuel OK         ");
    refillAttempts = 0;
    zAttemptCount = 0;
    digitalWrite(GREEN_LED_PIN, HIGH);
  }
  
  delay(SAMPLE_INTERVAL);
}

float getAverageTemperature(float newTemp) {
  tempSum -= tempBuffer[bufferIndex];
  tempBuffer[bufferIndex] = newTemp;
  tempSum += tempBuffer[bufferIndex];
  bufferIndex++;
  if (bufferIndex >= WINDOW_SIZE) {
    bufferIndex = 0;
    bufferFull = true;
  }
  if (tempCount < WINDOW_SIZE) tempCount++;
  return tempSum / (bufferFull ? WINDOW_SIZE : tempCount);
}

void adjustAirflow() {
  float targetRevolutions = 0.0;
  if (temperature < 75.0) {
    targetRevolutions = 0.0;
  } else if (temperature >= 75.0 && temperature < 77.0) {
    targetRevolutions = 0.75;
  } else if (temperature >= 77.0 && temperature < 79.0) {
    targetRevolutions = 1.5;
  } else if (temperature >= 79.0 && temperature < 80.0) {
    targetRevolutions = 2.25;
  } else {
    targetRevolutions = 3.0;
  }
  moveLeverTo(targetRevolutions);
}

void moveLeverTo(float targetRevolutions) {
  float deltaRevolutions = targetRevolutions - currentLeverPosition;
  long steps = (long)(deltaRevolutions * STEPS_PER_REVOLUTION);
  if (steps != 0) {
    myStepper.step(steps);
    currentLeverPosition = targetRevolutions;
  }
}

bool performRefill() {
  if (refillAttempts >= MAX_REFILL_ATTEMPTS || zAttemptCount >= MAX_Z_ATTEMPTS * 2) return false;
  
  // Fully close rack and pinion before refueling
  moveLeverTo(REVOLUTIONS_FULL); // Fully close (3 revolutions)
  delay(5000); // Wait 5 seconds to avoid fire accidents
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Refilling...");
  lcd.setCursor(0, 1);
  lcd.print("Attempt: ");
  lcd.print(refillAttempts + 1);
  
  digitalWrite(SOLENOID_PIN, HIGH);
  delay(1000);
  
  moveDownwards();
  while (stepper.distanceToGo() != 0) stepper.run();
  zAttemptCount++;
  delay(5000);
  
  moveUpwards();
  while (stepper.distanceToGo() != 0) stepper.run();
  zAttemptCount++;
  
  digitalWrite(SOLENOID_PIN, LOW);
  refillAttempts++;
  
  fuelWeight = readFuelWeight();
  if (fuelWeight >= fuelThreshold) {
    refillAttempts = 0;
    zAttemptCount = 0;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("IGNITE");
    displayFuelWeight();
    moveLeverTo(0.0); // Fully open (0 revolutions)
    delay(3000); // Wait 3 seconds
    digitalWrite(GREEN_LED_PIN, HIGH); // Turn on green LED
    return true;
  }
  
  return true;
}

void indicateRefillNeeded() {
  moveToZero();
  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(GREEN_LED_PIN, LOW);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ADD FUEL");
  displayFuelWeight();
}

float readFuelWeight() {
  if (scale.is_ready()) {
    float weight = scale.get_units(5);
    return weight > 0 ? weight : 0;
  }
  lcd.setCursor(0, 0);
  lcd.print("Scale Error     ");
  return fuelWeight;
}

void displayFuelWeight() {
  lcd.setCursor(0, 1);
  lcd.print("Fuel: ");
  lcd.print(fuelWeight, 1);
  lcd.print("g       ");
}

void moveUpwards() {
  zPosition = -TARGET_Z;
  long targetSteps = zPosition * STEPS_PER_MM;
  stepper.moveTo(targetSteps);
}

void moveDownwards() {
  zPosition = TARGET_Z;
  long targetSteps = zPosition * STEPS_PER_MM;
  stepper.moveTo(targetSteps);
}

void moveToZero() {
  zPosition = 0.0;
  stepper.moveTo(0);
  while (stepper.distanceToGo() != 0) stepper.run();
}

void updateReferenceWeight() {
  referenceWeight = readFuelWeight();
  fuelThreshold = referenceWeight - OPERATION_FUEL_THRESHOLD_OFFSET;
}