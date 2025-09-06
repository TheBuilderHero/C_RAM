// C-RAM (Counter Random Attacking Monsters)
#include <AccelStepper.h> //needed for motor control
#include <SoftwareSerial.h>// RX, TX pins messaging.


//C-RAM alarm Code://////////////////////////////////////////
#define BUZZER_PIN 9 // Example pin
bool run_alarm = false;

// Alarm timing state
unsigned long lastAlarmTime = 0;
const unsigned long alarmInterval = 650; // Interval between alarms

// Sweep state
enum AlarmState { IDLE, SWEEP_UP, SWEEP_DOWN, PAUSE };
AlarmState alarmState = IDLE;

int currentFreq = 0;
unsigned long lastStepTime = 0;
const unsigned long stepDelay = 5;  // Time between frequency changes
const int upStartFreq = 1500;
const int upEndFreq = 5000;
const int downEndFreq = 3500;


////////////////////////////////////////////////////////////////////////

// Motor Code: /////////////////////////////////////////

// Stepper interface type: driver (step/dir)
#define DRIVER_TYPE 1

const byte stepPinX = 2;   // x axis > 2, y axis > 3, z axis > 4 a axis > 12
const byte dirPinX = 5;    // x axis > 5, y axis > 6, z axis > 7 a axis > 13
const byte stepPinY = 3;
const byte dirPinY = 6;
const byte enablePin = 8;  // enable on the CNC shield is held HIGH (disabled) by default

// Stepper object for Y
AccelStepper stepperY(DRIVER_TYPE, stepPinY, dirPinY);
// Stepper object for X
AccelStepper stepperX(DRIVER_TYPE, stepPinX, dirPinX);

//////////////////////////////////////////////////////////

// Homing Code ///////////////////////////

long posX = 0, posY = 0;     // step counters
long minX = 0, maxX = 0;
long minY = 0, maxY = 0;

bool homingDone = false;

const int limitSwitchPinX = 9;
const int limitSwitchPinY = 10;
const int limitSwitchPinZ = 11;

unsigned long stepTime = 2400;
bool buttonReleased = true;
int direction = LOW;
int switchState;             
int lastSwitchState = LOW;   
unsigned long lastDebounceTime = 0;  
unsigned long debounceDelay = 50;  

// Object Tracking Code ////////////////////////////////////////

// Internal storage for last position
struct Position {
  int x;
  int y;
  int distance_mm;
};

static Position savedPosition = {-1, -1, -1};

// Messaging between Nicla Vision ////////////////////////////////////

SoftwareSerial SerialNicla(13, 12); // RX, TX pins



void setup() {
  Serial.begin(9600);     // USB serial for debug
  SerialNicla.begin(19200);  // RX/TX connected to Nicla Vision
  pinMode(stepPinX, OUTPUT);
  pinMode(dirPinX, OUTPUT);
  pinMode(stepPinY, OUTPUT);
  pinMode(dirPinY, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(limitSwitchPinX, INPUT_PULLUP); // Assuming HIGH = not pressed, LOW = pressed
  pinMode(limitSwitchPinY, INPUT_PULLUP); // Assuming HIGH = not pressed, LOW = pressed
  pinMode(limitSwitchPinZ, INPUT_PULLUP); // Assuming HIGH = not pressed, LOW = pressed
  digitalWrite(enablePin, LOW);  // enable steppers
  digitalWrite(dirPinX, direction);
  digitalWrite(dirPinY, direction);
  digitalWrite(stepPinX, LOW);
  digitalWrite(stepPinY, LOW);  
  stepperY.setMaxSpeed(1000);    // Adjust for your setup
  stepperY.setAcceleration(500); // Smooth acceleration
  pinMode(BUZZER_PIN, OUTPUT);
  
  C_ram_ararm_delayed();
  playPowerUpSound();
  homeAxes();
  moveHome();
  
  playReadySound();
}

void loop() {
  playStartLoopSound();
  //Get Data From Nicla
  if (SerialNicla.available()) { // is camera Identifies Threat
    Active_alarm(); //runs for specified amount of time before deactivating.


    String data = SerialNicla.readStringUntil('\n');
    Serial.println("Received: " + data);
    // You can split at ',' to get X, Y, and Distance

    int firstComma = data.indexOf(',');
    int secondComma = data.indexOf(',', firstComma + 1);

    if (firstComma > 0 && secondComma > firstComma) {
      int xPix = data.substring(0, firstComma).toInt();
      int yPix = data.substring(firstComma + 1, secondComma).toInt();
      int distance_mm = data.substring(secondComma + 1).toInt();
      int tempX = -1;
      int tempY = -1;
      int tempDis = -1;
      if (getSavedPosition(tempX, tempY, tempDis)){
        //Compare distance from old position to new position.

        Position oldPos = {tempX, tempY, tempDis};
        Position newPos = {xPix, yPix, distance_mm};
        Position changeInPos = {0 ,0 ,0};

        float totalChange = distanceBetweenPositions(oldPos, newPos, changeInPos);

        Serial.println(
          String("Total change: ") + totalChange +
          "; Change X: " + changeInPos.x +
          "; Change Y: " + changeInPos.y +
          "; Change Distance: " + changeInPos.distance_mm
        );

      }
      savePosition(xPix,yPix,distance_mm);
      moveStepsBlocking(stepperY, 1000);
      moveStepsBlocking(stepperX, 1000);
      delay(10000); //Stack forever until remove this line. I dont want it to repeat a ton of times and damage the C-RAM.
    }
  } else { // if no threat then we search.

  }

  C_ram_alarm_delayed_v2(); //run alarm if should be active.
}

// Moves the stepper motor 'stepper' by 'steps' steps (positive or negative).
// Blocks until all steps are done.
void moveStepsBlocking(AccelStepper &stepper, long steps) {
    // Set target position relative to current position
    Serial.println("IM MOVIN");
    long target = stepper.currentPosition() + steps;
    stepper.moveTo(target);

    // Keep running the stepper until it reaches target
    while (stepper.distanceToGo() != 0) {
        stepper.run(); // This is non-blocking, but we call it in a blocking loop
    }
}

// Save current position
void savePosition(int currentX, int currentY, int distance_mm) {
  savedPosition.x = currentX;
  savedPosition.y = currentY;
  savedPosition.distance_mm = distance_mm;
}

// Get saved position (returns via reference params)
//Returns true if values are stored else will return false if no values where previously saved.
bool getSavedPosition(int &outX, int &outY, int &outDistance) {
  outX = savedPosition.x;
  outY = savedPosition.y;
  outDistance = savedPosition.distance_mm;
  if (outX == -1 && outY == -1 && outDistance == -1) {
    return false;
  } else {
    return true;
  }
}

float distanceBetweenPositions(const Position &pos1, const Position &pos2, Position &change) {
  change.x = pos2.x - pos1.x;
  change.y = pos2.y - pos1.y;
  change.distance_mm = pos2.distance_mm - pos1.distance_mm;

  return sqrt(change.x*change.x + change.y*change.y + change.distance_mm*change.distance_mm); //output total change.
}

void oneStepX() {
  Serial.println("Limit switch pressed X!");
  static unsigned long timer = 0;
  unsigned long interval = stepTime;
  if (micros() - timer >= interval) {
    timer = micros();
    digitalWrite(stepPinX, HIGH);
    delayMicroseconds(10);
    digitalWrite(stepPinX, LOW);
  }
}

void oneStepY() {
  Serial.println("Limit switch pressed Y!");
  static unsigned long timer = 0;
  unsigned long interval = stepTime;
  if (micros() - timer >= interval) {
    timer = micros();
    digitalWrite(stepPinY, HIGH);
    delayMicroseconds(10);
    digitalWrite(stepPinY, LOW);
  }
}

void changeDir(){
  int reading = digitalRead(limitSwitchPinZ);

  if (reading != lastSwitchState) {
    lastDebounceTime = millis(); // Reset the debounce timer
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != switchState) {
      switchState = reading;
      if (switchState == LOW) { // Assuming normally open switch, LOW means pressed
        Serial.println("Limit switch pressed!");
        // Your code to handle the press goes here (e.g., stop a motor)
        if (direction == LOW){
          direction = HIGH;
          digitalWrite(dirPinX, direction);
          digitalWrite(dirPinY, direction);
        } else {
          direction = LOW;
          digitalWrite(dirPinX, direction);
          digitalWrite(dirPinY, direction);
        }
      } else {
        Serial.println("Limit switch released!");
        // Your code to handle the release goes here
      }
    }
  }
  lastSwitchState = reading;
}

void Active_alarm(){
  run_alarm = true;
}

void Deactive_alarm(){
  run_alarm = false;
}

void C_ram_ararm_delayed(){
  static unsigned long lastAlarmTime = 0;
  static unsigned long alarmStartTime = 0;
  const unsigned long alarmInterval = 1000; // Time between alarms in ms
  const unsigned long maxAlarmDuration = 10000; // 10 seconds
  unsigned long currentTime = millis();

  if (run_alarm) {
    // Record when alarm started
    if (alarmStartTime == 0) {
      alarmStartTime = currentTime;
    } 

    // Check if alarm duration exceeded 10 seconds
    if (currentTime - alarmStartTime >= maxAlarmDuration) {
      run_alarm = false;
      alarmStartTime = 0; // reset for next time
      return; // exit early, no alarm
    }

    // Check if enough time has passed to call the alarm
    if (currentTime - lastAlarmTime >= alarmInterval && run_alarm) {
      C_ram_alarm();
      lastAlarmTime = currentTime;
    }
  } else {
    alarmStartTime = 0;
    noTone(BUZZER_PIN);  // Stop any ongoing tone immediately
  }
}

void C_ram_alarm_delayed_v2() {
  unsigned long currentTime = millis();

  // Start a new alarm cycle if time has passed
  if (run_alarm && alarmState == IDLE && (currentTime - lastAlarmTime >= alarmInterval)) {
    alarmState = SWEEP_UP;
    currentFreq = upStartFreq;
    lastStepTime = currentTime;
  }

  // Handle the sweep-up state
  if (alarmState == SWEEP_UP && currentTime - lastStepTime >= stepDelay) {
    tone(BUZZER_PIN, currentFreq);
    currentFreq += 50;
    lastStepTime = currentTime;

    if (currentFreq > upEndFreq) {
      alarmState = SWEEP_DOWN;
      currentFreq = upEndFreq;
    }
  }

  // Handle the sweep-down state
  else if (alarmState == SWEEP_DOWN && currentTime - lastStepTime >= stepDelay) {
    tone(BUZZER_PIN, currentFreq);
    currentFreq -= 50;
    lastStepTime = currentTime;

    if (currentFreq < downEndFreq) {
      alarmState = PAUSE;
      noTone(BUZZER_PIN);
      lastStepTime = currentTime;
    }
  }

  // Pause briefly after sweep
  else if (alarmState == PAUSE && currentTime - lastStepTime >= 200) {
    alarmState = IDLE;
    lastAlarmTime = currentTime;
  }
}

void C_ram_alarm() {
  // Rising sweep
  for (int freq = 1500; freq <= 5000; freq += 50) {
    tone(BUZZER_PIN, freq);
    delay(5);  // Faster = more aggressive
  }

  // Falling sweep
  for (int freq = 5000; freq >= 3500; freq -= 50) {
    tone(BUZZER_PIN, freq);
    delay(5);
  }

  // Short break between sweeps
  noTone(BUZZER_PIN);
}

void playPowerUpSound() {
  int startFreq = 100;
  int endFreq = 400;
  int step = 10;

  // Powering up rising sweep with decreasing delay
  for (int freq = startFreq; freq <= endFreq; freq += step) {
    tone(BUZZER_PIN, freq);

    // Delay decreases exponentially as frequency rises
    int dynamicDelay = map(freq, startFreq, endFreq, 50, 2);
    delay(dynamicDelay);
  }

  // Dramatic pause before "ready" pings
  noTone(BUZZER_PIN);
  delay(300);

  // Final activation beeps (like "system ready")
  for (int i = 0; i < 3; i++) {
    tone(BUZZER_PIN, 1000 + (i * 200)); // Higher pitch each time
    delay(100);
    noTone(BUZZER_PIN);
    delay(100);
  }
}


void playReadySound() {
  // First note (lower tone, short)
  tone(BUZZER_PIN, 1500);
  delay(150);
  noTone(BUZZER_PIN);
  delay(50);

  // Second note (higher tone, longer)
  tone(BUZZER_PIN, 2000);
  delay(200);
  noTone(BUZZER_PIN);
  delay(50);

  // Quick final chirp
  tone(BUZZER_PIN, 2500);
  delay(100);
  noTone(BUZZER_PIN);
}

void playStartLoopSound() {
  // First note (lower tone, short)
  tone(BUZZER_PIN, 500);
  delay(150);
  noTone(BUZZER_PIN);
  delay(50);

  // Second note (higher tone, longer)
  tone(BUZZER_PIN, 250);
  delay(200);
  noTone(BUZZER_PIN);
  delay(50);

  // Quick final chirp
  tone(BUZZER_PIN, 500);
  delay(100);
  noTone(BUZZER_PIN);
}

//Homing fucntion:
void homeAxes() {
  // =====================
  // Home Y toward min
  // =====================
  stepperY.setSpeed(-200); // Negative = toward min
  while (digitalRead(limitSwitchPinY) == HIGH) { // Not pressed
    stepperY.runSpeed();
  }

  // Back off slightly
  stepperY.setSpeed(200); // Move away from switch
  unsigned long startBackoff = millis();
  while (millis() - startBackoff < 300) { // Move for 300ms
    stepperY.runSpeed();
  }

  // Re-approach slowly for precision
  stepperY.setSpeed(-50);
  while (digitalRead(limitSwitchPinY) == HIGH) {
    stepperY.runSpeed();
  }
  minY = 0;
  stepperY.setCurrentPosition(minY);

  // Back off slightly
  stepperY.setSpeed(200); // Move away from switch
  startBackoff = millis();
  while (millis() - startBackoff < 300) { // Move for 300ms
    stepperY.runSpeed();
  }
  // =====================
  // Move Y toward max
  // =====================
  stepperY.setSpeed(200); // Positive direction
  while (digitalRead(limitSwitchPinY) == HIGH) { // Replace with Y max switch if available
    stepperY.runSpeed();
  }

  // Back off slightly
  stepperY.setSpeed(-200);
  startBackoff = millis();
  while (millis() - startBackoff < 300) {
    stepperY.runSpeed();
  }

  // Re-approach slowly
  stepperY.setSpeed(50);
  while (digitalRead(limitSwitchPinY) == HIGH) {
    stepperY.runSpeed();
  }

  maxY = stepperY.currentPosition();

  homingDone = true;
  Serial.print("Y range: "); Serial.print(minY); Serial.print(" to "); Serial.println(maxY);
}

void moveHome(){
  long middleY = (minY + maxY) / 2; // calculate halfway point
  stepperY.moveTo(middleY);         // tell stepper to go there
  while (stepperY.distanceToGo() != 0) {
    stepperY.run();                 // non-blocking acceleration movement
  }
}