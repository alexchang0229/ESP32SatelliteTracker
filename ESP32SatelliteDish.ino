#include <WiFi.h>
#include <HTTPClient.h>
#include <AccelStepper.h>
#include <ESP32Time.h>
#include <Sgp4.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <HardwareSerial.h>
#include "time.h"
#include <SerLCD.h>
#include <math.h>
#include <strings.h>

// Replace with your network credentials
const char* ssid = "";
const char* password = "";


char satnames[8][30] = {"R2", "NEOSSAT", "SCISAT", "RCM1", "RCM2", "RCM3", "ISS"}; // Names of satellites.
char satURL[8][40] = {"/NORAD/elements/gp.php?CATNR=32382", "/NORAD/elements/gp.php?CATNR=39089",
                      "/NORAD/elements/gp.php?CATNR=27858", "/NORAD/elements/gp.php?CATNR=44322",
                      "/NORAD/elements/gp.php?CATNR=44324", "/NORAD/elements/gp.php?CATNR=44323",
                      "/NORAD/elements/gp.php?CATNR=25544"
                     }; // URL of Celestrak TLEs for satellites (In same order as names).
char TLE1[8][70];
char TLE2[8][70];

Sgp4 sat;
float minPassElevation = 10.0;
float myLat = 45.5106; float myLong = -73.4384; float myAlt = 27;   // Your latitude, longitude and altitude.
int numSats = 7;    // Number of satellites to track.
int SAT; int nextSat; int AZstart; long passEnd; int satVIS;
int  year; int mon; int day; int hr; int minute; double sec; int today;
long nextpassEpoch; long nextpassEndEpoch; long upcomingPasses[7]; long passDuration;
char satname[] = " ";
int passStatus = 0;

//EL 26, 18, 19, 23 Limit: 32
//AZ 22, 21, 17 ,16 Limit: 27
// Azimuth stepper pins //
#define AZmotorPin1  22      // IN1 on the ULN2003 driver
#define AZmotorPin2  21     // IN2 on the ULN2003 driver
#define AZmotorPin3  17     // IN3 on the ULN2003 driver
#define AZmotorPin4  16     // IN4 on the ULN2003 driver
#define AZLimit  27
// Elevation stepper pins //
#define ELmotorPin1  26
#define ELmotorPin2  18
#define ELmotorPin3  19
#define ELmotorPin4  23
#define ELLimit 32

HardwareSerial LCD(2);

int satAZsteps; int satELsteps; int turns = 0;
float oneTurn = 4096;

#define MotorInterfaceType 8  // Define the AccelStepper interface type; 4 wire motor in half step mode:
AccelStepper stepperAZ = AccelStepper(MotorInterfaceType, AZmotorPin1, AZmotorPin3, AZmotorPin2, AZmotorPin4);
AccelStepper stepperEL = AccelStepper(MotorInterfaceType, ELmotorPin1, ELmotorPin3, ELmotorPin2, ELmotorPin4);

ESP32Time rtc(0);
const int timeZone = -5;
unsigned long timeNow = 0;
const char* ntpServer = "pool.ntp.org";

unsigned long getTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    //Serial.println("Failed to obtain time");
    return (0);
  }
  time(&now);
  return now;
}

int nextSatPass(long _nextpassEpoch[7]) { // Replace with number of satellites
  for (int i = 0; i < numSats; ++i) {
    if ( _nextpassEpoch[0] - timeNow >= _nextpassEpoch[i] - timeNow) {
      _nextpassEpoch[0] = _nextpassEpoch[i];
      nextSat = i;
    }
  }
  return nextSat;
}

void initMotors() {
  // Setup stepper movements //
  stepperEL.setMaxSpeed(1000);
  stepperEL.setCurrentPosition(0); // Elevation stepper starts at -227 steps (20 degrees above horizon).
  stepperEL.setAcceleration(100);
  stepperAZ.setMaxSpeed(300);
  stepperAZ.setCurrentPosition(0);  // Azimuth stepper starts at 0.
  stepperAZ.setAcceleration(100);

  unsigned long startTime = millis();
  unsigned long currentTime;

  // Zero //
  stepperAZ.runToNewPosition(-600);
  while (true) {
    currentTime = millis();
    if (currentTime - startTime > 40 * 1000) {
      Serial.print("Zeroing timed out");
      while (true);
    }
    if (digitalRead(AZLimit) == LOW) {
      stepperAZ.setCurrentPosition(0);
      stepperAZ.runToNewPosition(-200);
      stepperAZ.setCurrentPosition(0);
      break;
    }
    if (stepperAZ.distanceToGo() == 0) { // if motor moved to the maximum position
      stepperAZ.setCurrentPosition(0);   // reset position to 0
      stepperAZ.moveTo(2000);       // move the motor to maximum position again
    }
    stepperAZ.run();
  }
  digitalWrite(AZmotorPin1, LOW);
  digitalWrite(AZmotorPin2, LOW);
  digitalWrite(AZmotorPin3, LOW);
  digitalWrite(AZmotorPin4, LOW);

  stepperEL.runToNewPosition(-100);
  startTime = millis();
  while (true) {
    currentTime = millis();
    if (currentTime - startTime > 40 * 1000) {
      Serial.println("Zeroing timed out");
      while (true);
    }
    if (digitalRead(ELLimit) == LOW) {
      stepperEL.setCurrentPosition(0);
      stepperEL.runToNewPosition(-80);
      stepperEL.setCurrentPosition(0);
      break;
    }
    if (stepperEL.distanceToGo() == 0) { // if motor moved to the maximum position
      stepperEL.setCurrentPosition(0);   // reset position to 0
      stepperEL.moveTo(1000);       // move the motor to maximum position again
    }
    stepperEL.run();
  }


  digitalWrite(ELmotorPin1, LOW);
  digitalWrite(ELmotorPin2, LOW);
  digitalWrite(ELmotorPin3, LOW);
  digitalWrite(ELmotorPin4, LOW);
  delay(1000);
};

void clearLCD() {
  LCD.write(0xFE);  // send the special command
  LCD.write(0x01);  // send the set cursor command
}

SerLCD lcd;

void setup() {
  pinMode(ELLimit, INPUT_PULLUP); // init pins for limit switches
  pinMode(AZLimit, INPUT_PULLUP);
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  Serial.begin(115200);
  LCD.begin(9600, SERIAL_8N1, 20, 25); // Rx pin not used
  lcd.begin(LCD);
  delay(1000);

  // Connect to Wi-Fi network
  clearLCD();
  lcd.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(2000);
    WiFi.begin(ssid, password);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi!");

  // Get Unix timestamp
  clearLCD();
  LCD.write("Fetching time...");
  while (timeNow == 0) {
    configTime(0, 0, ntpServer);
    timeNow = getTime();
  }
  Serial.println("unixtime: " + String(timeNow));
  rtc.setTime(timeNow);

  Serial.println("Zeroing");
  clearLCD();
  lcd.print("Zeroing...");
  initMotors();

  // Fetch TLE
  clearLCD();
  LCD.write("Fetching TLEs");
  HTTPClient http;
  sat.site(myLat, myLong, myAlt); //set location latitude[°], longitude[°] and altitude[m]
  for (SAT = 0; SAT < numSats; SAT++) {
    Serial.println("Request #: " + String(SAT) + " For: " + String(satnames[SAT]));
    http.begin("https://celestrak.org/" + String(satURL[SAT]));
    int httpCode = http.GET();
    String TLE;

    if (httpCode == HTTP_CODE_OK) {
      TLE = http.getString();
      TLE.substring(26, 96).toCharArray(TLE1[SAT], 70);
      TLE.substring(97, 167).toCharArray(TLE2[SAT], 70);
      Serial.println(TLE1[SAT]);
      Serial.println(TLE2[SAT]);

      sat.init(satname, TLE1[SAT], TLE2[SAT]);   //initialize satellite parameters
      upcomingPasses[SAT] = Predict(timeNow); // find next pass and store it array
    } else {
      Serial.println("Failed to fetch TLE data for " + String(satnames[SAT]));
    }
    delay(100);
  }
  http.end();

  nextSat = nextSatPass(upcomingPasses); // find earliest pass and track that sat
  Serial.println("Next satellite: " + String(nextSat));
  sat.init(satname, TLE1[nextSat], TLE2[nextSat]); 
  Predict(timeNow);
}

String printHHmmss (unsigned long _secondsIn) {
  unsigned int _hour = floor(_secondsIn / 60 / 60);
  unsigned int _minute = floor((_secondsIn / 60) % 60) ;
  unsigned int _seconds = _secondsIn % 60; \
  char DateAndTimeString[20]; //19 digits plus the null char
  sprintf(DateAndTimeString, "%02d:%02d:%02d", _hour, _minute, _seconds);
  return DateAndTimeString;
}

void loop() {
  timeNow = rtc.getEpoch();
  sat.findsat(timeNow);
  satAZsteps = -round(sat.satAz * oneTurn / 360); //Convert degrees to stepper steps
  satELsteps = -round(sat.satEl * oneTurn / 360);

  invjday(sat.satJd , timeZone, true, year, mon, day, hr, minute, sec);
  Serial.println("\nLocal time: " + String(day) + '/' + String(mon) + '/' + String(year) + ' ' + String(hr) + ':' + String(minute) + ':' + String(sec));
  Serial.println("azimuth = " + String( sat.satAz) + " elevation = " + String(sat.satEl) + " distance = " + String(sat.satDist));
  Serial.println("latitude = " + String( sat.satLat) + " longitude = " + String( sat.satLon) + " altitude = " + String( sat.satAlt));
  Serial.println("AZsteps = " + String(satAZsteps));

  if (nextpassEpoch - timeNow < 300 && nextpassEpoch + 5 - timeNow > 0) {
    Serial.println("Status: Pre-pass");
    Serial.println("Next satellite is #: " + String(satnames[nextSat]) + " in: " + String(nextpassEpoch - timeNow));
    clearLCD();
    lcd.print(satnames[nextSat]);
    lcd.print(" " + printHHmmss(nextpassEpoch - timeNow));
    lcd.setCursor(0, 1);
    lcd.print("AZ:" + String(int(sat.satAz)) + " EL:" + String(int(sat.satEl)));
    prepass();
  } else if (sat.satVis != -2) {
    Serial.println("Status: In pass");
    clearLCD();
    lcd.print(satnames[nextSat]);
    lcd.print(" " + String(nextpassEndEpoch - timeNow));
    lcd.setCursor(0, 1);
    lcd.print("AZ:" + String(int(sat.satAz)) + " EL:" + String(int(sat.satEl)));
    inPass();
  } else if (timeNow - passEnd < 60) {
    Serial.println("Status: Post-pass");
    clearLCD();
    lcd.print("Post-pass:" + String(passEnd - timeNow));
    postpass();
  } else if (sat.satVis == -2) {
    Serial.println("Status: Standby");
    Serial.println("Next satellite is: " + String(satnames[nextSat]) + " in: " + String(nextpassEpoch - timeNow));
    clearLCD();
    lcd.print(satnames[nextSat]);
    lcd.print(" " + printHHmmss(nextpassEpoch - timeNow));
    lcd.setCursor(0, 1);
    lcd.print("AZ:" + String(int(sat.satAz)) + " EL:" + String(int(sat.satEl)));
    standby();
  }



  delay(1000);
}

void standby () {

  // Azimuth //
  stepperAZ.runToNewPosition(0);
  // ELEVATION //
  stepperEL.runToNewPosition(-280); //Standby at 20 degrees above horizon

  digitalWrite(AZmotorPin1, LOW);
  digitalWrite(AZmotorPin2, LOW);
  digitalWrite(AZmotorPin3, LOW);
  digitalWrite(AZmotorPin4, LOW);

  digitalWrite(ELmotorPin1, LOW);
  digitalWrite(ELmotorPin2, LOW);
  digitalWrite(ELmotorPin3, LOW);
  digitalWrite(ELmotorPin4, LOW);
}

void prepass() {
  // Pass is less than 300 seconds (5 mins) away, move antenna to start location and wait.
  if (AZstart < 360 && AZstart > 180) {
    AZstart = AZstart - 360; //Goes to start counter-clockwise if closer.
  }
  stepperAZ.runToNewPosition(-AZstart * oneTurn / 360);
  stepperEL.runToNewPosition(0);

  digitalWrite(AZmotorPin1, LOW);
  digitalWrite(AZmotorPin2, LOW);
  digitalWrite(AZmotorPin3, LOW);
  digitalWrite(AZmotorPin4, LOW);

  digitalWrite(ELmotorPin1, LOW);
  digitalWrite(ELmotorPin2, LOW);
  digitalWrite(ELmotorPin3, LOW);
  digitalWrite(ELmotorPin4, LOW);

}

void inPass() {

  // Handle zero crossings
  if (AZstart > 0) {
    satAZsteps = satAZsteps - oneTurn;
  }
  if (satAZsteps - stepperAZ.currentPosition() > 100) {
    stepperAZ.setCurrentPosition(stepperAZ.currentPosition() + oneTurn);
    turns--;
  }
  if (satAZsteps - stepperAZ.currentPosition() < -100) {
    stepperAZ.setCurrentPosition(stepperAZ.currentPosition() - oneTurn);
    turns++;
  }

  // Update stepper position
  stepperAZ.runToNewPosition(satAZsteps);
  stepperEL.runToNewPosition(satELsteps);
  passEnd = timeNow;
  passStatus = 1;
}

void postpass() {
  Serial.println("Post pass time left: " + String(passEnd + 60 - timeNow));
  if (timeNow - passEnd > 30) {
    if (turns > 0) {
      stepperAZ.setCurrentPosition(stepperAZ.currentPosition() + oneTurn);
      turns--;
    }
    if (turns < 0) {
      stepperAZ.setCurrentPosition(stepperAZ.currentPosition() - oneTurn);
      turns++;
    }
  }
  if (passStatus == 1 && timeNow - passEnd > 50) {
    for (SAT = 0; SAT < numSats; SAT++) {
      sat.init(satname, TLE1[SAT], TLE2[SAT]);
      upcomingPasses[SAT] = Predict(timeNow + 200); // + 100 seconds so next pass is for sure not the one just ending.
      Serial.println("Next pass for Satellite #: " + String(SAT) + " in: " + String(upcomingPasses[SAT] - timeNow));
    }
    nextSat = nextSatPass(upcomingPasses);
    sat.init(satname, TLE1[nextSat], TLE2[nextSat]);
    Predict(timeNow);
    passStatus = 0;
  }

}
