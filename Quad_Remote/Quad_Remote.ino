#include <SoftwareSerial.h>
#include <BlynkSimpleSerialBLE.h>


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

int powerPin = 8;
int landTakePin = 7;

int accelJoyPin = A0;
int dirJoyPinY = A1;
int dirJoyPinX = A2;



int shouldStabilizeLedPin = A5;
int isFlyingLedPin = A4;
int powerLedPin = 6;
int currentSpeedLedPin = 5;

WidgetLCD lcd(V9);

BlynkTimer timer;

int accelValOff;
int dirJoyXOff;
int dirJoyYOff;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  pinMode(powerPin, INPUT);
  pinMode(landTakePin, INPUT);


  pinMode(accelJoyPin, INPUT);
  pinMode(dirJoyPinX, INPUT);
  pinMode(dirJoyPinY, INPUT);


  pinMode(powerLedPin, OUTPUT);
  pinMode(currentSpeedLedPin, OUTPUT);
  pinMode(isFlyingLedPin, OUTPUT);
  pinMode(shouldStabilizeLedPin, OUTPUT);

  accelValOff = analogRead(accelJoyPin);
  dirJoyXOff = analogRead(dirJoyPinX);
  dirJoyYOff = analogRead(dirJoyPinY);

  //Blynk.begin(Serial, auth);
  //resetAppControls();
  //resetAppControlsLed();
  //timer.setInterval(1000, timer1);
}

int powerPinState = 0;
int powerPinStateLast = 0;
long int  powerPinPushTime = 0;
int  powerPinPushDuration = 0;

int landTakePinState = 0;
int landTakePinStateLast = 0;


long int currentSpeedLedPinMillis;
bool currentSpeedLedPinRepeat = true;

unsigned long disconnectMillis;


void loop() {
  if(Serial.available()){
    Serial.write(Serial.read());
  }
  ////////////////////////  ////////////////////////
  digitalWrite(isFlyingLedPin, !transmitData.isLanded);
  digitalWrite(shouldStabilizeLedPin, transmitData.shouldStabilize);;
  handleAccelJoy();
  speedLedBlink();
  handlePowerBttn();
  handleLandTakeBttn();
  ////////////////////////  ////////////////////////

}

BLYNK_CONNECTED() {
  lcd.print(0, 0, "Blynk Connected.");
}

BLYNK_APP_CONNECTED() {
  lcd.print(0, 0, "App Connected.");
}

BLYNK_WRITE(V0) {
  // IF QUAD IS FLYING AND ISN'T TURNED ON
  transmitData.isQuadTurnedOn = param.asInt();
  Blynk.virtualWrite(V10, transmitData.isQuadTurnedOn * 255);



}

int sliderVal;
BLYNK_WRITE(V1) {
  sliderVal = param.asInt();
  if (transmitData.isQuadTurnedOn && sliderVal != 1 & sliderVal != 2) {
    transmitData.currentSpeed = sliderVal;

    if (transmitData.currentSpeed > transmitData.baseSpeed) {
      transmitData.isLanded = false;
    } else if (transmitData.currentSpeed < transmitData.baseSpeed / 2) {
      transmitData.isLanded = true;
    }
    if (transmitData.currentSpeed > transmitData.minSpeed) {
      transmitData.shouldStabilize = true;
    } else {
      transmitData.shouldStabilize = false;
    }
  } else {
    resetAppControls();
    lcd.print(0, 0, "Quad is OFF");
  }


  /*
    if (transmitData.isQuadTurnedOn) {
      if (param.asInt() != 1 && param.asInt() != 2) {
        //transmitData.currentSpeed = ((transmitData.maxSpeed - transmitData.minSpeed) / (transmitData.maxSpeed)) * param.asInt() + transmitData.minSpeed;

      }
    } else { // SEPARATE IF STATEMENT USED TO AVOID SLIDER SETTING ITSELF TO 2 WHEN CONTROLLER DISCONNECTED
      resetAppControls();
    }
  */
}
BLYNK_WRITE(V2) {
  if (transmitData.isQuadTurnedOn) {


  } else {
    resetAppControls();
    lcd.print(0, 0, "Quad is OFF");
  }
}
BLYNK_WRITE(V3) {
  if (transmitData.isQuadTurnedOn) {


  } else {
    resetAppControls();
    lcd.print(0, 0, "Quad is OFF");
  }

}
BLYNK_WRITE(V4) {
  if (transmitData.isQuadTurnedOn) {
    if (param.asInt() == 1 && transmitData.isLanded) {
      transmitData.shouldTakeOff = true;
    } else if (transmitData.currentSpeed > transmitData.baseSpeed / 2) {
      transmitData.shouldLand = true;
    }
  } else {
    resetAppControls();
    lcd.print(0, 0, "Quad is OFF");
  }
}

void land() {
  // send signal to quad to land
}


void timer1() {
  lcd.clear();
  lcd.print(0, 1, String(millis() / 1000) + "s");
  timer2();
  timer3();
}
void timer2() {
  Blynk.virtualWrite(V10, transmitData.isQuadTurnedOn * 255);
  Blynk.virtualWrite(V11, transmitData.currentSpeed);
}
void timer3() {
  Blynk.virtualWrite(V12, !transmitData.isLanded * 255);
  Blynk.virtualWrite(V13, transmitData.shouldStabilize * 255);
}
void timer4() {
  Blynk.virtualWrite(V4, !transmitData.isLanded);
  Blynk.virtualWrite(V0, transmitData.isQuadTurnedOn);
  Blynk.virtualWrite(V1, transmitData.currentSpeed);
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

void handleAccelJoy() {
  float accelVal = analogRead(accelJoyPin) - accelValOff;
  if (accelVal < -30) {
    if ( transmitData.currentSpeed < transmitData.maxSpeed) {
      transmitData.currentSpeed += pow((abs(accelVal) / 200.0), 2);
    } else {
      transmitData.currentSpeed = 255;
    }

  }
  if (accelVal > 30) {
    if (transmitData.currentSpeed > 0) {
      transmitData.currentSpeed -= pow((abs(accelVal) / 250.0), 2);
    } else {
      transmitData.currentSpeed = 0;
    }

  }
}

void handlePowerBttn() {
  powerPinState = digitalRead(powerPin);
  if (powerPinState == LOW && powerPinStateLast == HIGH) {
    powerPinPushTime = millis();
    if (transmitData.isQuadTurnedOn) {
      transmitData.isQuadTurnedOn = false;
      Serial.println("TURNING OFF " + String(millis()));
      digitalWrite(powerLedPin, LOW);
    } else {
      transmitData.isQuadTurnedOn = true;
      Serial.println("TURNING ON " + String(millis()));
      digitalWrite(powerLedPin, HIGH);
    }

  } else if (powerPinState == HIGH && transmitData.isQuadTurnedOn) { //&& !transmitData.shouldEmergencyStop) {
    powerPinPushDuration = 0;
    powerPinPushDuration += millis() - powerPinPushTime;
    if (powerPinPushDuration > 1000) {
      digitalWrite(powerLedPin, LOW);
    }
    if (powerPinPushDuration > 1500) {
      digitalWrite(powerLedPin, HIGH);
    }
    if (powerPinPushDuration > 2000) {
      digitalWrite(powerLedPin, LOW);
    }
    if (powerPinPushDuration > 2500) {
      digitalWrite(powerLedPin, HIGH);
    }
    if (powerPinPushDuration > 3000) {
      digitalWrite(powerLedPin, LOW);
      transmitData.shouldEmergencyStop = true;

      Serial.println("EMERGENCY STOP " + String(millis()));
    }
  }
  powerPinStateLast = powerPinState;
}

void handleLandTakeBttn() {
  landTakePinState = digitalRead(landTakePin);
  if (landTakePinState == LOW && landTakePinStateLast == HIGH) {
    if (transmitData.isLanded) {
      transmitData.shouldTakeOff = true;
      Serial.println("TAKING OFF " + String(millis()));
    } else {
      transmitData.shouldLand = true;
      Serial.println("LANDING " + String(millis()));
    }

  }
  landTakePinStateLast = landTakePinState;
}

void speedLedBlink() {
  if (transmitData.currentSpeed == transmitData.maxSpeed) {
    if (currentSpeedLedPinRepeat) {
      digitalWrite(currentSpeedLedPin, LOW);
      currentSpeedLedPinMillis = millis();
      currentSpeedLedPinRepeat = false;
    }
    if (millis() - currentSpeedLedPinMillis > 200) {
      digitalWrite(currentSpeedLedPin, HIGH);
    }
    if (millis() - currentSpeedLedPinMillis > 1000) {
      currentSpeedLedPinRepeat = true;
    }
  } else {
    analogWrite(currentSpeedLedPin, transmitData.currentSpeed);
  }
}

