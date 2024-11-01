#include <SparkFun_LIS331.h>
// #include <Watchdog.h>
#include <Wire.h>
#include "BluetoothSerial.h"

//hardware
LIS331 accel1;
LIS331 accel2;
BluetoothSerial bt;
// Watchdog watchdog; //Watchdog is disabled until i fix it lol.

//Drive pins
int pinLeftDrive = 4;
int pinRightDrive = 16;
//LED pins
int pinLED = 5;

//whatever the fuck ledc channels are?
int chanLeftDrive = 0;
int chanRightDrive = 1;

// Misc
float accelDist = 8.23;
int accel1Orientation = 2;  //orientation of accelerometer1: x=0 y=1 z=2
int accel2Orientation = 2;  //orientation of accelerometer2: x=0 y=1 z=2
bool accelSame = false;     //whether one accel is flipped
long guideLEDwidth = 900;   //width of the guide
long driveLEDwidth = 100;   //width of the drive indicator
long LEDoffset = -450;         //direction of LED relative to "Forwards" in degrees*10
long driveWidth = 450;      //valid times to go in direction
int ticksPerCheck = 5;
int ticksPerAccelCalc = 10;
int ticksPerSend = 500;
int timeout = 3000;               //milliseconds since last heartbeat to be declared dead
const int runningAvgLen = 50;     //how many datapoints to use for the running average
int driveTimeout = 1000;          //how long it drives in a direction with timedmove
int accelReadZone = 450;          // how large of an area after drive width that the accelerometers are allowed to be read in
int forceAccelReadTime = 500000;  //roughly the time it takes to read from the accelerometers in microseconds
int driveControlFreq = 4000;      //Oneshot125 is 4000hz
int maxPower = 220;               //Maximum PWM duty (max 255)
float rpmPerPower = 10;           //number of RPM per 1/255 power. rpmPerPower*255 should be equal or larger that max rpm.
int powerFloor = 32;              //minimum rpmPerPower duty (max 255)

//voltage reader
float r1 = 47000;
float r2 = 10250;
int voltagePin = 33;
float fuckupvoltmult = 1;  //fuck this number, it should be 1

//VARIABLES
float rpm;
float meltyTick;     //number of microseconds per revolution
long direction = 0;  //direction that the bot is pointing in melty mode, degrees * 10
long lastTick = 0;   //last time time was calculated
long ticks = 0;      //loop counter
bool melting = 0;    //if in spinny mode
long heartbeat = 0;  //time of last heartbeat
long meltyStart;     //time of engaging meltymode
int driveTime = 0;   //last time timedmove was issued
TaskHandle_t accelLoop;
float battVoltage = 0;

char currentCommand;
int driveAngle = 0;  //out of 360
int driveSpeed = 0;  //0 to 1
int tempdriveSpeed = 0;
int tempdriveAngle = 0;
int tempweaponSpeed = 0;  //MELTY SPEED
int weaponSpeed = 0;
long leftMotor = 0;
long rightMotor = 0;
bool flippedAngle = 0;

//accel outputs
int16_t a1x;
int16_t a2x;
int16_t a1y;
int16_t a2y;
int16_t a1z;
int16_t a2z;
int16_t a1val;
int16_t a2val;

//accel running avg
float a1Running = 0;
float a2Running = 0;
int a1ArrPos = 0;
int a2ArrPos = 0;
int avgArrPos = 0;
float a1Arr[runningAvgLen];
float a2Arr[runningAvgLen];
float avgArr[runningAvgLen];
long lastAccelRead = 0;

void setup() {
  Serial.begin(115200);
  bt.begin();
  Wire.begin();
  Serial.println(F("Hello!"));


  // accel initialize
  accel1.setI2CAddr(0x18);
  accel2.setI2CAddr(0x19);
  accel1.begin(LIS331::USE_I2C);
  accel2.begin(LIS331::USE_I2C);
  accel1.setFullScale(LIS331::HIGH_RANGE);
  accel2.setFullScale(LIS331::HIGH_RANGE);
  accel1.setODR(LIS331::DR_1000HZ);
  accel2.setODR(LIS331::DR_1000HZ);

  // motor initialize
  ledcSetup(chanLeftDrive, driveControlFreq, 11);
  ledcAttachPin(pinLeftDrive, chanLeftDrive);
  setMotor(chanLeftDrive, 0);

  ledcSetup(chanRightDrive, driveControlFreq, 11);
  ledcAttachPin(pinRightDrive, chanRightDrive);
  setMotor(chanRightDrive, 0);

  // LED initialize
  pinMode(pinLED, OUTPUT);
  digitalWrite(pinLED, HIGH);
  delay(500);
  digitalWrite(pinLED, LOW);
  delay(500);
  digitalWrite(pinLED, HIGH);
  delay(500);
  digitalWrite(pinLED, LOW);
  char str[20];
  int i = 0;
  float a1Total = 0;
  float a2Total = 0;
  while (true) {

    readAccel();
    a1Total += accel1.convertToG(400, a1val);
    a2Total += accel1.convertToG(400, a2val);
    if (i % 100 == 0) {
      Serial.print("Voltage: ");
      Serial.println(readVoltage());
      btwrite("*Z0*");     //RPM
      btwrite("*Xdist:");  //Accel dist
      dtostrf(accelDist, 1, 2, str);
      btwrite(str);
      btwrite("*");
      btwrite("*V-69*");  //TPS

      dtostrf(a1Running, 1, 2, str);  //gforce of a1
      btwrite("*BGs a1:");
      btwrite(str);
      dtostrf(a2Running, 1, 2, str);
      btwrite(", a2:");
      btwrite(str);
      btwrite("*");

      Serial.print("a1AVG: ");
      Serial.print(a1Running);
      Serial.print(" a2AVG: ");
      Serial.println(a2Running);

      Serial.print("X1:");
      Serial.print(accel1.convertToG(400, a1x));
      Serial.print(" Y1:");
      Serial.print(accel1.convertToG(400, a1y));
      Serial.print(" Z1:");
      Serial.print(accel1.convertToG(400, a1z));
      Serial.print("\n");

      Serial.print("X2:");
      Serial.print(accel2.convertToG(400, a2x));
      Serial.print(" Y2:");
      Serial.print(accel2.convertToG(400, a2y));
      Serial.print(" Z2:");
      Serial.print(accel2.convertToG(400, a2z));
      Serial.print("\n");
      Serial.println("-----------------------------------------");
    }
    i++;
    delay(1);
    if (i == 1000) {
      digitalWrite(pinLED, HIGH);
      a1Running = a1Running - (a1Total / i);
      a2Running = a2Running - (a2Total / i);
    } else if (i > 1000) {
      if ((char)bt.read() == '?') { break; }
    }
  }
  setMotor(chanRightDrive, 0);
  setMotor(chanRightDrive, 0);
  delay(1000);
  setMotor(chanRightDrive, 255);
  setMotor(chanRightDrive, 255);
  delay(100);
  setMotor(chanRightDrive, 0);
  setMotor(chanRightDrive, 0);
  delay(500);
  xTaskCreatePinnedToCore(accelLoopCode, "accelLoop", 10000, NULL, 1, &accelLoop, 0);

  // watchdog.enable(Watchdog::TIMEOUT_2S);
}

void loop() {
  while (true) {  //esp32 loop() doesnt run at full speed?????
    // watchdog.reset();
    if (heartbeat + timeout < millis()) {

      digitalWrite(pinLED, HIGH);
      driveSpeed = 0;
      driveAngle = 0;
      freeze();
      if (millis() % 1000 == 0) {
        Serial.print((millis() - heartbeat) / 1000);
        Serial.println(F(" since last beat"));
        delay(1);
      }
      setMotor(chanLeftDrive, 0);
      setMotor(chanRightDrive, 0);
      checkCommands();

    } else {
      // driveCheck();
      if (melting) {
        calc();
        meltyLED();
        melty();
        ticks++;

        //WE MULTITHREADED NOW
        // if (((direction - (driveWidth / 2) + (driveAngle * 10 * driveSpeed)) % 1800 < accelReadZone) or micros() > lastAccelRead + forceAccelReadTime) {
        //   readAccel();
        //   lastAccelRead = micros();
        // }

        if (ticks % ticksPerSend == 0) {
          send();
        }
        if (ticks % ticksPerCheck == 0) {
          checkCommands();
        }
        delay(1);
      } else {
        checkCommands();
        // if (!driveSpeed) { readAccel(); }
        calc();
        send();
      }
    }
  }
}
void accelOrientation() {
  if (accel1Orientation == 0) {
    a1val = a1x;
  } else if (accel1Orientation == 1) {
    a1val = a1y;
  } else {  //it may seem like im picking the z axis as my favorite. Yes. Yes I am.
    a1val = a1z;
  }
  if (accel2Orientation == 0) {
    a2val = a2x;
  } else if (accel2Orientation == 1) {
    a2val = a2y;
  } else {  //it may seem like im picking the z axis as my favorite. Yes. Yes I am.
    a2val = a2z;
  }
}

void accelLoopCode(void* pvParameters) {
  while (true) {
    readAccel();
  }
}
//Reads accelerometers and calculates RPM and ticktime
void readAccel() {
  accel1.readAxes(a1x, a1y, a1z);
  accel2.readAxes(a2x, a2y, a2z);
  accelOrientation();

  //progress running arrays arrays
  a1Running = a1Running + (accel1.convertToG(400, a1val) / runningAvgLen) - a1Arr[a1ArrPos];
  a1Arr[a1ArrPos] = accel1.convertToG(400, a1val) / runningAvgLen;
  a1ArrPos = (a1ArrPos + 1) % runningAvgLen;
  a2Running = a2Running + (accel2.convertToG(400, a2val) / runningAvgLen) - a2Arr[a2ArrPos];
  a2Arr[a2ArrPos] = accel2.convertToG(400, a2val) / runningAvgLen;
  a2ArrPos = (a2ArrPos + 1) % runningAvgLen;

  //calculate RPM and add to array
  if (accelSame) {  //if they are in the same direction, they subtract from each other
    avgArr[avgArrPos] = sqrt(abs(a1Running - a2Running) / (accelDist * 0.00001118));
  } else {  //double negatives ig
    avgArr[avgArrPos] = sqrt(abs(a1Running + a2Running) / (accelDist * 0.00001118));
  }


  rpm = avgArr[avgArrPos] * 1.25 - (avgArr[(avgArrPos + 1) % runningAvgLen] / 4);
  // rpm = avgArr[avgArrPos];
  avgArrPos = (avgArrPos + 1) % runningAvgLen;
  meltyTick = 1000000 / abs(rpm / 60);
}


//calculates direction pointing (degrees*10)
void calc() {
  long newTime = micros();
  direction = long(direction + 3600 * ((newTime - lastTick) / meltyTick)) % 3600;
  lastTick = newTime;
}

void meltyLED() {
  //Guide LED
  if ((direction + LEDoffset + (guideLEDwidth / 2)) % 3600 < guideLEDwidth) {
    digitalWrite(pinLED, HIGH);
  }else  if (((direction + LEDoffset + (driveAngle * 10) + (driveLEDwidth / 2)) % 3600 < driveLEDwidth)&&(driveSpeed>5)) {
    digitalWrite(pinLED, HIGH);
    }else {
    digitalWrite(pinLED, LOW);
  }
}

void melty() {
  Serial.println(driveSpeed);
  if (driveSpeed > 5) {
    //LeftDrive
    Serial.println((direction + (driveAngle * 10) + (driveWidth / 2)) % 3600);

    // if ((direction + (driveAngle * 10) + (driveWidth / 2)) % 3600 < driveWidth) {
    //   setMotor(chanLeftDrive, 0);
    //   // setMotor(chanRightDrive, 255);
    // } else {
    //   setMotor(chanLeftDrive, -weaponSpeed);
    //   // setMotor(chanRightDrive, weaponSpeed);
    // }
    // // RightDrive
    // if ((direction + (driveAngle * 10) - 1800 + (driveWidth / 2)) % 3600 < driveWidth) {
    //   // setMotor(chanLeftDrive, -255);
    //   setMotor(chanRightDrive, 0);
    // } else {
    //   // setMotor(chanLeftDrive, -weaponSpeed);
    //   setMotor(chanRightDrive, weaponSpeed);
    // }
    if ((direction + (driveAngle * 10) + (driveWidth / 2)) % 3600 < driveWidth) {
      setMotor(chanLeftDrive, 0);
      setMotor(chanRightDrive, 255);
      // Backwards
    } else if ((direction + (driveAngle * 10) - 1800 + (driveWidth / 2)) % 3600 < driveWidth) {
      setMotor(chanLeftDrive, -255);
      setMotor(chanRightDrive, 0);
    } else {
      setMotor(chanLeftDrive, -weaponSpeed);
      setMotor(chanRightDrive, weaponSpeed);
    }
  } else {
    setMotor(chanLeftDrive, -weaponSpeed);
    setMotor(chanRightDrive, weaponSpeed);
  }
}

void melt() {
  melting = 1;
  ticks = 1;
  meltyStart = millis();
  Serial.println("MELTY TIME");
  delay(2);
}
void freeze() {
  melting = 0;
  setMotor(chanLeftDrive, 0);
  setMotor(chanRightDrive, 0);
  digitalWrite(pinLED, HIGH);
}

void checkCommands() {
  while (bt.available() > 0) {
    char nextChar = (char)bt.read();
    Serial.print(nextChar);
    if (nextChar == '?') {
      heartbeat = millis();
    } else if (nextChar == 'W') {
      tempweaponSpeed = 0;
      currentCommand = 'W';
    } else if (nextChar == 'R') {
      tempdriveSpeed = 0;
      currentCommand = 'R';
    } else if (nextChar == 'A') {
      tempdriveAngle = 0;
      currentCommand = 'A';
    } else if (nextChar == 'M') {
      if (currentCommand == 'M') { melt(); }
      currentCommand = 'M';
    } else if (nextChar == 'm') {
      if (currentCommand == 'm') { freeze(); }
      currentCommand = 'm';
    } else if (nextChar == 'H') {
      changeAccelDist(1);
    } else if (nextChar == 'J') {
      changeAccelDist(0.1);
    } else if (nextChar == 'K') {
      changeAccelDist(0.01);
    } else if (nextChar == 'L') {
      changeAccelDist(0.001);
    } else if (nextChar == 'h') {
      changeAccelDist(-1);
    } else if (nextChar == 'j') {
      changeAccelDist(-0.1);
    } else if (nextChar == 'k') {
      changeAccelDist(-0.01);
    } else if (nextChar == 'l') {
      changeAccelDist(-0.001);
    } else if (nextChar == 'F') {
      flippedAngle = 1;
    } else if (nextChar == 'f') {
      flippedAngle = 0;
    } else if (nextChar == 'O') {
      LEDoffset += 50;
      LEDoffset = LEDoffset % 3600;
    } else if (nextChar == 'o') {
      LEDoffset -= 50;
      LEDoffset = LEDoffset % 3600;
    } else if (nextChar == '!') {
      set();
    } else {
      if (nextChar < 48 || nextChar > 57) { nextChar = 48; }
      switch (currentCommand) {
        case 'R':
          tempdriveSpeed = tempdriveSpeed * 10 + nextChar - 48;
          break;
        case 'A':
          tempdriveAngle = tempdriveAngle * 10 + nextChar - 48;
          break;
        case 'W':
          tempweaponSpeed = tempweaponSpeed * 10 + nextChar - 48;
      }
    }
  }
}
void driveCheck() {
  if (millis() > driveTime + driveTimeout) { driveSpeed = 0; }
}
void set() {
  if (currentCommand == 'W') {
    weaponSpeed = tempweaponSpeed;
  }
  if (tempdriveAngle > 359 || tempdriveAngle < 0) {
    return;
  }
  driveSpeed = tempdriveSpeed;
  if (melting) {
    if (flippedAngle == 1) {
      tempdriveAngle = 359 - tempdriveAngle;
    }
  }
  driveAngle = tempdriveAngle;
  if (!melting) {  //Normal driving

    //Crude mixing
    leftMotor = driveSpeed * (cos(float(driveAngle) * PI / 180) + sin(float(driveAngle) * PI / 180) / 2);
    rightMotor = driveSpeed * (cos(float(driveAngle) * PI / 180) - sin(float(driveAngle) * PI / 180) / 2);

    //clamp to 255
    if (max(abs(leftMotor), abs(rightMotor)) > 255) {
      int m = max(abs(leftMotor), abs(rightMotor));
      leftMotor = leftMotor * 255 / m;
      rightMotor = rightMotor * 255 / m;
    }

    //write to motors
    setMotor(chanLeftDrive, leftMotor);
    setMotor(chanRightDrive, rightMotor);
  }
  Serial.println("");
}

//set ledcWrite for drive motor
void setMotor(int chan, int speed) {
  //clamp between -255 to 255
  if (speed > maxPower) {
    speed = maxPower;
  } else if (speed < -maxPower) {
    speed = -maxPower;
  }
  //
  ledcWrite(chan, 512 + speed);
}

//ramp up the meltybrain in melty mode to stop the fucking pulleys from melting.
void setMeltyMotor(int chan, int speed) {
  if (abs(speed) > rpm / rpmPerPower) {
    speed = abs(speed) / speed * rpm / rpmPerPower;
    if (abs(speed) < powerFloor) {
      speed = abs(speed) / speed * powerFloor;
    }
  }

  setMotor(chan, speed);
}
float readVoltage() {
  battVoltage = fuckupvoltmult * (r1 + r2) * 3.3 * analogRead(voltagePin) / 4096 / r2;
  return battVoltage;
}
void send() {
  char str[20];

  // btwrite("*VGyro:");
  // dtostrf(g1Xrunning, 1, 0, str);
  // btwrite(str);
  // btwrite("*");

  // btwrite("*XCalc1:");
  // dtostrf(rpmBig, 1, 0, str);
  // btwrite(str);
  // btwrite("*");
  if (melting == 1) {
    btwrite("*VTPS:");
    dtostrf((ticks * 1000 / (millis() - meltyStart)), 1, 0, str);
    btwrite(str);
    btwrite("*");
    // delay(2);
  }


  btwrite("*ZRPM");
  dtostrf(60000000 / meltyTick, 1, 0, str);
  btwrite(str);
  btwrite("*");

if(millis()%4000>2000){
btwrite("*BLEDO:");
dtostrf(LEDoffset, 1, 0, str);
btwrite(str);
btwrite("*");
}else{
    //send accel values
  btwrite("*BA1:");
  int tempval = accel1.convertToG(400, a1val);
  if (tempval < 0) {
    btwrite("-");
    tempval = tempval * -1;
  } else {
    btwrite("+");
  }
  if (tempval < 100) {
    btwrite("0");
  }
  if (tempval < 10) {
    btwrite("0");
  }
  dtostrf(tempval, 1, 0, str);
  btwrite(str);
  btwrite(" A2:");
  tempval = accel2.convertToG(400, a1val);
  if (tempval < 0) {
    btwrite("-");
    tempval = tempval * -1;
  } else {
    btwrite("+");
  }
  if (tempval < 100) {
    btwrite("0");
  }
  if (tempval < 10) {
    btwrite("0");
  }
  dtostrf(tempval, 1, 0, str);
  btwrite(str);
  btwrite("*");
  }

  btwrite("*NBatt: ");
  dtostrf(readVoltage(), 1, 1, str);
  btwrite(str);
  btwrite("V*");
}
void btwrite(String str) {
  uint8_t buf[str.length()];
  memcpy(buf, str.c_str(), str.length());
  bt.write(buf, str.length());
}
void changeAccelDist(float offset) {
  char str[20];
  accelDist += offset;
  btwrite("*Xdist:");
  dtostrf(accelDist, 1, 3, str);
  btwrite(str);
  btwrite("*");
}