#include <SoftwareSerial.h>
#include <BlynkSimpleSerialBLE.h>
#include <EEPROM.h>
#include <Wire.h>

//^^ LIBRARIES

const int diagMsgInterval = 1500;


struct data {
  bool shouldEmergencyStop = false;
  int isQuadTurnedOn = false;
  bool hasStartedProps = false;
  bool shouldTakeOff = false;
  bool shouldLand = false;
  bool isLanded = true;
  bool shouldStabilize = false;

  const float minSpeed = 20.0; // MINIMUM MOTOR SPEED THAT STILL SPINS THE PROPS
  const float maxSpeed = 255.0; // MAXIMUM MOTOR SPEED
  float baseSpeed = 150.0; // BASE HOVER OR FLYING MOTOR SPEED
  float currentSpeed = 0.0; // THE SPEED OF THE MOTORS ONCE QUAD IS POWERED ON
};
struct data transmitData;

float m1Spd, m2Spd, m3Spd, m4Spd; // STORES EACH SPEED VALUE FOR EACH MOTOR

float gyroMarginOfError = 0.008; // THE MARGIN OF ERROR FOR THE GYRO&ACCEL MODULE

const int ledPin = 4; // ON-BOARD LED PIN USED TO SIGNAL DISCONNECTIONS AND MALFUNCTIONS
const int buttonPin = 2; // ON-BOARD BUTTON PIN USED TO CALIBRATE THE GYRO&ACCEL MODULE
const int led1 = 3;
const int led2 = 6;
const int led3 = 5;
const int led4 = 9;
//^^ PINS

long accelXoff, accelYoff, accelZoff; //OFFSETS FOR ACCELEROMETER TO PROVIDE A ZERO VALUE FOR FLAT SURFACES

long accelX, accelY, accelZ; // RAW DATA
float gForceX, gForceY, gForceZ; // PROCESSED DATA
long gyroX, gyroY, gyroZ; // RAW DATA
float rotX, rotY, rotZ; // PROCESSED DATA

bool isCalibrated = true;
int calibrateButtonState = 0;
//^^ VAR DECLARATIONS

void setup() {

  Serial.begin(9600);

  Wire.begin();
  pinMode(ledPin, OUTPUT);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);
  pinMode(buttonPin, INPUT);

  //setupMPU();
  //readOffsetsFromMemory();
}


int disconnectLandTimeout = 30; // SECONDS TO WAIT TILL LANDING AFTER DISCONNECT
unsigned long disconnectMillis; // STORES TIME AT WHICH DISCONNECT OCCURED

int testBttnState = LOW;

void loop() {



}

void quadLogic() {

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
    if (transmitData.isQuadTurnedOn == 1) {
      // IF PROPS AREN'T STARTED, START THEM
      if (!transmitData.hasStartedProps) {
        startProps();
      } else if (transmitData.shouldTakeOff) {
        // IF PROPS ARE STARTED AND transmitData.shouldTakeOff = true, TAKE OFF

        takeOff();
      } else if (transmitData.shouldLand) {
        // IF PROPS ARE STARTED AND transmitData.shouldLand = true, LAND
        land();
      } else {
        // ELSE, IS IN FLYING MODE. STABILIZE QUADCOPTER IN PLACE


      }
    } else {
      // IF QUADCOPTER IS TURNED OFF
      if (!transmitData.isLanded && transmitData.currentSpeed > transmitData.baseSpeed / 2 && !transmitData.shouldEmergencyStop) {
        // IF QUAD IS FLYING THEN LAND
        land();
      } else if (transmitData.hasStartedProps) {
        transmitData.currentSpeed = 0;
        transmitData.shouldStabilize = false;
        transmitData.hasStartedProps = false;
        transmitData.shouldEmergencyStop = false;
      }
    }

    if (transmitData.shouldStabilize) {
      stabilize();
    }
    printDiagnostics();
  }
}

unsigned long startPropsMillis;
void startProps() {

  transmitData.currentSpeed = transmitData.maxSpeed;
  Serial.println("Starting Props! " + String(transmitData.currentSpeed));
  startPropsMillis = millis();

  while (transmitData.currentSpeed > transmitData.minSpeed) {
    transmitData.currentSpeed = ((cos((millis() - startPropsMillis) / 200.0) + 1) / 2) * transmitData.maxSpeed;
    rotateMotors();
    if (transmitData.currentSpeed < transmitData.minSpeed) {
      transmitData.currentSpeed = transmitData.minSpeed;

      transmitData.hasStartedProps = true;
      Serial.println("Started props! ... Speed: " + String(transmitData.currentSpeed));
    } else {
      Serial.println("Starting Props! ... Speed: " + String(transmitData.currentSpeed) + " || " + String(millis()) + " millis.");
    }

  }

}

unsigned long takeOffMillis;

void takeOff() {

  transmitData.shouldStabilize = true;
  transmitData.currentSpeed = transmitData.maxSpeed;
  Serial.println("Taking off! " + String(transmitData.currentSpeed));
  takeOffMillis = millis();

  while (transmitData.currentSpeed > transmitData.baseSpeed) {
    transmitData.currentSpeed = ((cos((millis() - takeOffMillis) / 1000.0) + 1) / 2) * transmitData.maxSpeed;
    if (transmitData.currentSpeed < transmitData.baseSpeed) {
      transmitData.currentSpeed = transmitData.baseSpeed;
      transmitData.isLanded = false;
      transmitData.shouldTakeOff = false;
      Serial.println("Took off! ... Speed: " + String(transmitData.currentSpeed));
    } else {
      Serial.println("Taking off! ... Speed: " + String(transmitData.currentSpeed) + " || " + String(millis()) + " millis.");
    }

  }

}

unsigned long landMillis;
void land() {
  Serial.println("Landing! " + String(transmitData.currentSpeed));
  landMillis = millis();

  while (transmitData.currentSpeed > transmitData.minSpeed) {
    transmitData.currentSpeed = ((cos((millis() - landMillis) / 4000.0) + 1) / 2) * transmitData.currentSpeed;
    if (transmitData.currentSpeed < transmitData.minSpeed) {
      transmitData.currentSpeed = transmitData.minSpeed;
      transmitData.isLanded = true;
      transmitData.shouldLand = false;
      transmitData.shouldStabilize = false;
      Serial.println("Landed! ... Speed: " + String(transmitData.currentSpeed));
    } else {
      Serial.println("Landing! ... Speed: " + String(transmitData.currentSpeed) + " || " + String(millis()) + " millis.");
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

      setSpeed(3, transmitData.currentSpeed);
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

      setSpeed(4, transmitData.currentSpeed);
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

      setSpeed(1, transmitData.currentSpeed);
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

      setSpeed(2, transmitData.currentSpeed);
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
  if ((m1Spd + m2Spd + m3Spd + m4Spd) / 2 != transmitData.currentSpeed) {
    setSpeed(1, transmitData.currentSpeed);
    setSpeed(2, transmitData.currentSpeed);
    setSpeed(3, transmitData.currentSpeed);
    setSpeed(4, transmitData.currentSpeed);
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
  float result = (transmitData.maxSpeed - transmitData.currentSpeed) * force + transmitData.currentSpeed;

  return result;
}

String lastDiagMsg;
unsigned long lastDiagMsgMillis;
void printDiagnostics() {
  String diagMsg = "State: " + String(transmitData.isQuadTurnedOn) + " || propsOn?: " + String(transmitData.hasStartedProps) +  " || isLanded?: " + String(transmitData.isLanded) + " || shouldStabilize: " + String(transmitData.shouldStabilize) + " || Spd: " + String(transmitData.currentSpeed) + " || Speeds: (1: " + String(m1Spd) + ") (2: " + String(m2Spd) + ") (3: " + String(m3Spd) + ") (4: " + String(m4Spd) + ") || ";
  if (diagMsg != lastDiagMsg || (millis() - lastDiagMsgMillis) > diagMsgInterval) {
    Serial.print(diagMsg);
    Serial.println(String(millis()) + " millis.");
    lastDiagMsg = diagMsg;
    lastDiagMsgMillis = millis();
  }
}

