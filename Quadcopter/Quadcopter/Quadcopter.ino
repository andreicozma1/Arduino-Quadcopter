#define BLYNK_PRINT Serial
#include <BlynkSimpleSerialBLE.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <Wire.h>

//^^ LIBRARIES

const char auth[] = "6a641f95ede44ef4b2dc25048a44f01c";


const int diagMsgInterval = 1500;

int isQuadTurnedOn = false;
bool hasStartedProps = false;
bool shouldTakeOff = false;
bool shouldLand = false;
bool isLanded = true;
bool shouldStabilize = false;

const float minSpeed = 20.0; // MINIMUM MOTOR SPEED THAT STILL SPINS THE PROPS
const float maxSpeed = 255.0; // MAXIMUM MOTOR SPEED
float baseSpeed = 150; // BASE HOVER OR FLYING MOTOR SPEED
float currentSpeed = 0; // THE SPEED OF THE MOTORS ONCE QUAD IS POWERED ON

float m1Spd, m2Spd, m3Spd, m4Spd; // STORES EACH SPEED VALUE FOR EACH MOTOR

float gyroMarginOfError = 0.008; // THE MARGIN OF ERROR FOR THE GYRO&ACCEL MODULE
WidgetLCD lcd(V9);
const int ledPin = 8; // ON-BOARD LED PIN USED TO SIGNAL DISCONNECTIONS AND MALFUNCTIONS
const int buttonPin = 7; // ON-BOARD BUTTON PIN USED TO CALIBRATE THE GYRO&ACCEL MODULE
const int led1 = 11;
const int led2 = 10;
const int led3 = 5;
const int led4 = 4;
//^^ PINS

long accelXoff, accelYoff, accelZoff; //OFFSETS FOR ACCELEROMETER TO PROVIDE A ZERO VALUE FOR FLAT SURFACES

long accelX, accelY, accelZ; // RAW DATA
float gForceX, gForceY, gForceZ; // PROCESSED DATA
long gyroX, gyroY, gyroZ; // RAW DATA
float rotX, rotY, rotZ; // PROCESSED DATA

bool isCalibrated = true;
int calibrateButtonState = 0;
//^^ VAR DECLARATIONS

BlynkTimer timer;

void setup() {

  Serial.begin(9600);

  Serial1.begin(9600);

  Blynk.begin(Serial1, auth);
  resetAppControls();

  Wire.begin();
  pinMode(ledPin, OUTPUT);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);
  pinMode(buttonPin, INPUT);

  timer.setInterval(1000, timer1);
  //timer.setInterval(1013, timer2);
  //timer.setInterval(1513, timer3);
  //timer.setInterval(513, timer4);
  setupMPU();
  readOffsetsFromMemory();
}

void timer1() {
  lcd.clear();
  lcd.print(0, 1, String(millis() / 1000) + "s");
  timer2();
  timer3();
}
void timer2() {
  Blynk.virtualWrite(V10, isQuadTurnedOn * 255);
  Blynk.virtualWrite(V11, currentSpeed);
}
void timer3() {
  Blynk.virtualWrite(V12, !isLanded * 255);
  Blynk.virtualWrite(V13, shouldStabilize * 255);
}
void timer4() {
  Blynk.virtualWrite(V4, !isLanded);
  Blynk.virtualWrite(V0, isQuadTurnedOn);
  Blynk.virtualWrite(V1, currentSpeed);
}

void resetAppControls() {
  timer2();
  timer3();
  timer4();
}

void resetAppControlsLed() {
  timer2();
  timer3();
}

BLYNK_CONNECTED() {
  Serial.println("Blynk Connected.");
  lcd.print(0, 0, "Blynk Connected.");
}

BLYNK_APP_CONNECTED() {
  Serial.println("App Connected.");
  lcd.print(0, 0, "App Connected.");
}

BLYNK_WRITE(V0) {
  // IF QUAD IS FLYING AND ISN'T TURNED ON
  isQuadTurnedOn = param.asInt();
  Blynk.virtualWrite(V10, isQuadTurnedOn * 255);
  if (!isLanded && currentSpeed > baseSpeed / 2) {
    // IF QUAD IS FLYING THEN LAND
    land();
  }

  if (hasStartedProps) {
    currentSpeed = 0;
    shouldStabilize = false;
    hasStartedProps = false;
  }
}

int sliderVal;
BLYNK_WRITE(V1) {
  sliderVal = param.asInt();
  if (isQuadTurnedOn && sliderVal != 1 & sliderVal != 2) {
    currentSpeed = sliderVal;

    if (currentSpeed > baseSpeed) {
      isLanded = false;
    } else if (currentSpeed < baseSpeed / 2) {
      isLanded = true;
    }
    if (currentSpeed > minSpeed) {
      shouldStabilize = true;
    } else {
      shouldStabilize = false;
    }
  } else {
    resetAppControls();
    Serial.println("Quad is OFF");
    lcd.print(0, 0, "Quad is OFF");
  }


  /*
    if (isQuadTurnedOn) {
      if (param.asInt() != 1 && param.asInt() != 2) {
        //currentSpeed = ((maxSpeed - minSpeed) / (maxSpeed)) * param.asInt() + minSpeed;

      }
    } else { // SEPARATE IF STATEMENT USED TO AVOID SLIDER SETTING ITSELF TO 2 WHEN CONTROLLER DISCONNECTED
      Serial.println("Must turn quadcopter ON first!");
      resetAppControls();
    }
  */
}
BLYNK_WRITE(V2) {
  if (isQuadTurnedOn) {


  } else {
    resetAppControls();
    Serial.println("Quad is OFF");
    lcd.print(0, 0, "Quad is OFF");
  }
}
BLYNK_WRITE(V3) {
  if (isQuadTurnedOn) {


  } else {
    resetAppControls();
    Serial.println("Quad is OFF");
    lcd.print(0, 0, "Quad is OFF");
  }

}
BLYNK_WRITE(V4) {
  if (isQuadTurnedOn) {
    if (param.asInt() == 1 && isLanded) {
      shouldTakeOff = true;
    } else if (currentSpeed > baseSpeed / 2) {
      shouldLand = true;
    }
  } else {
    resetAppControls();
    Serial.println("Quad is OFF");
    lcd.print(0, 0, "Quad is OFF");
  }
}


int disconnectLandTimeout = 30; // SECONDS TO WAIT TILL LANDING AFTER DISCONNECT
unsigned long disconnectMillis; // STORES TIME AT WHICH DISCONNECT OCCURED

void loop() {

  Blynk.run();
  timer.run();

  if (Blynk.connected()) {

    // SET disconnectMillis BACK TO 0, INDICATING THAT IT HASN'T BEEN DISCONNECTED
    if (disconnectMillis != 0) {
      disconnectMillis = 0;
    }

    // INITIATE GYRO CALIBRATION ON PHYSICAL BUTTON PRESS
    if (calibrateButtonState != digitalRead(buttonPin)) {
      calibrateButtonState = digitalRead(buttonPin);
      if (calibrateButtonState == 0) {
        Serial.println("Calibrating..");
        isCalibrated = false;
        // SETTING isCalibrated TO false WILL CAUSE THE LOOP TO GO INTO CALIBRATION STATE
      }
    }

    // IF NOT CALIBRATED, CALIBRATE
    if (!isCalibrated) {
      calibrate();
    } else {
      // ELSE, IF CALIBRATED...
      recordAccelRegisters();
      recordGyroRegisters();
      // READ GYRO & ACCEL MODULE VALUES

      // IF QUADCOPTER IS TURNED ON...
      if (isQuadTurnedOn == 1) {
        // IF PROPS AREN'T STARTED, START THEM
        if (!hasStartedProps) {
          startProps();
        } else if (shouldTakeOff) {
          // IF PROPS ARE STARTED AND shouldTakeOff = true, TAKE OFF
          takeOff();
        } else if (shouldLand) {
          // IF PROPS ARE STARTED AND shouldLand = true, LAND
          land();
        } else {
          // ELSE, IS IN FLYING MODE. STABILIZE QUADCOPTER IN PLACE


        }
      } else {
        // IF QUADCOPTER IS TURNED OFF


      }

      if (shouldStabilize) {
        stabilize();
      }
      printDiagnostics();
    }

  } else {
    // IF DISCONNECTED, SET DISCONNECTMILLIS BACK TO A NON-0 VALUE AND PERFORM THE REST OF THE DISCONNECT SEQUENCE.

    Blynk.connect();
    Serial.println("Disconnected...");
    //lcd.print(0, 0, "Disconnected...");
    /*
      if (!isLanded) {
      if (disconnectMillis = 0) {
        disconnectMillis = millis();
      }
      Serial.println("Smartphone disconnected from quadcopter! Landing... Disconnected at " + disconnectMillis);
      // IF THE DISCONNECT TIME PASSES THE disconnectLandTimeout, INITIATE LANDING
      if (millis() - disconnectMillis * 1000 > disconnectLandTimeout) {
        land();
      }
      }

      // FLASH LED ON AND OFF AT AN INTERVAL OF 500Ms
      digitalWrite(ledPin, HIGH);
      delay(500);
      digitalWrite(ledPin, LOW);
      delay(500);
    */
  }
}

unsigned long startPropsMillis;
void startProps() {

  currentSpeed = maxSpeed;
  Serial.println("Starting Props! " + String(currentSpeed));
  lcd.print(0, 0, "Starting Props!");
  startPropsMillis = millis();

  while (currentSpeed > minSpeed) {
    currentSpeed = ((cos((millis() - startPropsMillis) / 200.0) + 1) / 2) * maxSpeed;
    rotateMotors();
    Blynk.virtualWrite(V1, currentSpeed);
    Blynk.virtualWrite(V11, currentSpeed);
    if (currentSpeed < minSpeed) {
      currentSpeed = minSpeed;
      Blynk.virtualWrite(V1, minSpeed);
      Blynk.virtualWrite(V11, currentSpeed);
      hasStartedProps = true;
      Serial.println("Started props! ... Speed: " + String(currentSpeed));
    } else {
      Serial.println("Starting Props! ... Speed: " + String(currentSpeed) + " || " + String(millis()) + " millis.");
    }

  }

}

unsigned long takeOffMillis;

void takeOff() {

  shouldStabilize = true;
  currentSpeed = maxSpeed;
  resetAppControlsLed();
  Serial.println("Taking off! " + String(currentSpeed));
  lcd.print(0, 0, "Taking off!" );
  takeOffMillis = millis();

  while (currentSpeed > baseSpeed) {
    Blynk.virtualWrite(V4, 1);
    currentSpeed = ((cos((millis() - takeOffMillis) / 1000.0) + 1) / 2) * maxSpeed;
    Blynk.virtualWrite(V1, currentSpeed);
    Blynk.virtualWrite(V11, currentSpeed);
    if (currentSpeed < baseSpeed) {
      currentSpeed = baseSpeed;
      Blynk.virtualWrite(V1, baseSpeed);
      Blynk.virtualWrite(V11, currentSpeed);
      isLanded = false;
      shouldTakeOff = false;
      Serial.println("Took off! ... Speed: " + String(currentSpeed));
    } else {
      Serial.println("Taking off! ... Speed: " + String(currentSpeed) + " || " + String(millis()) + " millis.");
    }

  }

}

unsigned long landMillis;
void land() {
  resetAppControlsLed();
  Serial.println("Landing! " + String(currentSpeed));
  lcd.print(0, 0, "Landing!" );
  landMillis = millis();

  while (currentSpeed > minSpeed) {
    Blynk.virtualWrite(V4, 0);
    currentSpeed = ((cos((millis() - landMillis) / 4000.0) + 1) / 2) * currentSpeed;
    Blynk.virtualWrite(V1, currentSpeed);
    Blynk.virtualWrite(V11, currentSpeed);
    if (currentSpeed < minSpeed) {
      currentSpeed = minSpeed;
      Blynk.virtualWrite(V1, minSpeed);
      Blynk.virtualWrite(V11, currentSpeed);
      isLanded = true;
      shouldLand = false;
      shouldStabilize = false;
      Serial.println("Landed! ... Speed: " + String(currentSpeed));
    } else {
      Serial.println("Landing! ... Speed: " + String(currentSpeed) + " || " + String(millis()) + " millis.");
    }
  }
}


void stabilize() {

  float absForceX = abs(gForceX);
  float absForceY = abs(gForceY);

  // IF QUAD IS WITHIN MARGIN OF ERROR WITHIN ALL AXES, KEEP SPEED EQUAL ON ALL MOTORS
  if (gForceX > -gyroMarginOfError && gForceX < gyroMarginOfError && gForceY > -gyroMarginOfError && gForceY < gyroMarginOfError) {
    rotateMotors();
  }
  else if (gForceZ > 0) {
    // IF QUADCOPTER IS NOT UPSIDE DOWN, EXECUTE THE REST.
    // THIS MAKES SURE THAT IF THE QUADCOPTER IS EVER FLIPPED, THE MOTORS STAY IN THE LATEST SETTING, WHICH HELPS UNFLIP AND STABILIZE;

    if (gForceX > gyroMarginOfError && gForceY < -gyroMarginOfError && gForceX < 1 && gForceY > -1) {
      setSpeed(1, ((absForceX > absForceY) ? getAddedSpeed(absForceX, 1) : getAddedSpeed(absForceY, 1)));

      setSpeed(3, currentSpeed);
      if (absForceX > absForceY) {
        setSpeed(4, getAddedSpeed(absForceY, 4));
        setSpeed(2, getAddedSpeed(absForceX, 2));
      }
      else {
        setSpeed(2, getAddedSpeed(absForceX, 2));
        setSpeed(4, getAddedSpeed(absForceY, 4));
      }
    }
    if (gForceX > gyroMarginOfError && gForceY > gyroMarginOfError && gForceX < 1 && gForceY < 1) {
      setSpeed(2, ((absForceX > absForceY) ? getAddedSpeed(absForceX, 2) : getAddedSpeed(absForceY, 2)));

      setSpeed(4, currentSpeed);
      if (absForceX > absForceY) {
        setSpeed(3, getAddedSpeed(absForceY, 3));
        setSpeed(1, getAddedSpeed(absForceX, 1));
      }
      else {
        setSpeed(1, getAddedSpeed(absForceX, 1));
        setSpeed(3, getAddedSpeed(absForceY, 3));
      }
    }
    if (gForceX < -gyroMarginOfError && gForceY > gyroMarginOfError && gForceX > -1 && gForceY < 1) {
      setSpeed(3, ((absForceX > absForceY) ? getAddedSpeed(absForceX, 3) : getAddedSpeed(absForceY, 3)));

      setSpeed(1, currentSpeed);
      if (absForceX > absForceY) {
        setSpeed(2, getAddedSpeed(absForceY, 2));
        setSpeed(4, getAddedSpeed(absForceX, 4));
      }
      else {
        setSpeed(4, getAddedSpeed(absForceX, 4));
        setSpeed(2, getAddedSpeed(absForceY, 2));
      }
    }
    if (gForceX  < -gyroMarginOfError && gForceY < -gyroMarginOfError && gForceX > -1 && gForceY > -1) {
      setSpeed(4, ((absForceX > absForceY) ? getAddedSpeed(absForceX, 4) : getAddedSpeed(absForceY, 4)));

      setSpeed(2, currentSpeed);
      if (absForceX  > absForceY) {
        setSpeed(1, getAddedSpeed(absForceY, 1));
        setSpeed(3, getAddedSpeed(absForceX, 3));
      }
      else {
        setSpeed(3, getAddedSpeed(absForceX, 3));
        setSpeed(1, getAddedSpeed(absForceY, 1));
      }
    }
  }
}


void processAccelData();
void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6);
  //while (Wire.available() < 6);
  accelX = Wire.read() << 8 | Wire.read();
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();
  processAccelData();
}
void processAccelData() {
  gForceX = (accelX - accelXoff) / 16384.0;
  gForceY = (accelY - accelYoff) / 16384.0;
  gForceZ = (accelZ + 16348.0 - accelZoff) / 16384.0;
}

void processGyroData();
void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6);
  //while (Wire.available() < 6);
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
  processGyroData();
}
void processGyroData() {
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0;
  rotZ = gyroZ / 131.0;
}


void calibrate() {

  long aX, aY, aZ;
  long gX, gY, gZ;

  int calCt = 1000;
  for (int x = 0; x < calCt; x++) {
    digitalWrite(ledPin, HIGH);
    recordAccelRegisters();
    aX += accelX;
    aY += accelY;
    aZ += accelZ;

    recordGyroRegisters();
    gX += gyroX;
    gY += gyroY;
    gZ += gyroZ;

    Serial.println("Reading Gyro & Accel for calibration. Attempt " + String(x) + " out of " + String(calCt) + ".");
  }

  accelXoff = aX / calCt;
  saveOffset(0, accelXoff);

  accelYoff = aY / calCt;
  saveOffset(4, accelYoff);

  accelZoff = aZ / calCt;
  saveOffset(8, accelZoff);

  readOffsetsFromMemory();

  Serial.println("Finished Calibrating!");
  isCalibrated = true;
  digitalWrite(ledPin, LOW);
}

void setupMPU() {
  // SETUP THE SETTINGS REGISTERS OF THE GYRO&ACCEL MODULE
  Wire.beginTransmission(0b1101000);
  Wire.write(0x6B);
  Wire.write(0b00000000);
  Wire.endTransmission();
  // ^ SET SLEEP MODE OFF

  Wire.beginTransmission(0b1101000);
  Wire.write(0x1B);
  Wire.write(0b00000000);
  Wire.endTransmission();
  // ^ SET GYROSCOPE SENSITIVITY;

  Wire.beginTransmission(0b1101000);
  Wire.write(0x1C);
  Wire.write(0b00000000);
  Wire.endTransmission();
  // ^ SET ACCELEROMETER SENSITIVITY;

}


void saveOffset(int address, long val) {
  // TAKES THE ADDRESS OF THE EEPROM TO SAVE TO, AS WELL AS THE VALUE

  // CONVERTS THE val INTO BYTES TO SAVE EACH ONE TO EEPROM
  byte byte1 = val & 0xFF;
  byte byte2 = val >> 8 & 0xFF;
  byte byte3 = val >> 16 & 0xFF;
  byte byte4 = val >> 24 & 0xFF;

  // SAVES GYRO&ACCEL OFFSETS TO ARDUINO EEPROM MEMORY
  EEPROM.write(address, byte1);
  EEPROM.write(address + 1, byte2);
  EEPROM.write(address + 2, byte3);
  EEPROM.write(address + 3, byte4);

  Serial.println("#################### SAVING " + address);
  Serial.println(val);
  Serial.println(byte1);
  Serial.println(byte2);
  Serial.println(byte3);
  Serial.println(byte4);

  Serial.println("Saved " + String(val) + " in EEPROM address " + String(address));
}

long readOffset(int address) {
  // TAKES THE EEPROP ADDRESS TO READ FROM

  // READS 4 SUBSEQUENT BYTES OF MEMORY FROM THE EEPROM
  byte one = EEPROM.read(address);
  byte two = EEPROM.read(address + 1);
  byte three = EEPROM.read(address + 2);
  byte four = EEPROM.read(address + 3);

  Serial.println("#################### READING " + address);
  Serial.println(one);
  Serial.println(two);
  Serial.println(three);
  Serial.println(four);

  // CONVERTS THE BYTES BACK INTO A long VALUE USING BITSHIFT AND RETURNS THE RESULT
  long result = (four << 24) + (three << 16) + (two << 8) + (one << 0);
  Serial.println("Read " + String(result) + " from EEPROM address " + String(address));
  return result;
}

void readOffsetsFromMemory() {
  // THIS FUNCTION IS CALLED IN SETUP TO READ THE EEPROM OFFSETS FOR THE GYRO&ACCEL MODULE
  accelXoff = readOffset(0);
  accelYoff = readOffset(4);
  accelZoff = readOffset(8);
}

void rotateMotors() {
  if ((m1Spd + m2Spd + m3Spd + m4Spd) / 2 != currentSpeed) {
    setSpeed(1, currentSpeed);
    setSpeed(2, currentSpeed);
    setSpeed(3, currentSpeed);
    setSpeed(4, currentSpeed);
  }
}

void setSpeed(int led, float speed) {
  // THIS FUNCTION TAKES IN A MOTOR/LED NUMBER AND SPEED TO SET FOR THAT MOTOR
  switch (led) {
    case 1:
      analogWrite(led1, speed);
      m1Spd = speed;
      break;
    case 2:
      analogWrite(led2, speed);
      m2Spd = speed;
      break;
    case 3:
      analogWrite(led3, speed);
      m3Spd = speed;
      break;
    case 4:
      analogWrite(led4, speed);
      m4Spd = speed;
      break;
  }

}

float getAddedSpeed(float force, int motor) {
  float result = (maxSpeed - currentSpeed) * force + currentSpeed;

  return result;
}

String lastDiagMsg;
unsigned long lastDiagMsgMillis;
void printDiagnostics() {
  String diagMsg = "State: " + String(isQuadTurnedOn) + " || propsOn?: " + String(hasStartedProps) +  " || isLanded?: " + String(isLanded) + " || shouldStabilize: " + String(shouldStabilize) + " || Spd: " + String(currentSpeed) + " || Speeds: (1: " + String(m1Spd) + ") (2: " + String(m2Spd) + ") (3: " + String(m3Spd) + ") (4: " + String(m4Spd) + ") || ";
  if (diagMsg != lastDiagMsg || (millis() - lastDiagMsgMillis) > diagMsgInterval) {
    Serial.print(diagMsg);
    Serial.println(String(millis()) + " millis.");
    lastDiagMsg = diagMsg;
    lastDiagMsgMillis = millis();
  }
}

