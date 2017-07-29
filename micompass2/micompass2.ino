#include <SoftwareSerial.h>
#include "TinyGPS++.h"
#include <Wire.h>
//#include <Adafruit_Sensor.h>
//#include <Adafruit_HMC5883_U.h>
#include <TimerOne.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <EEPROM.h>

// Credits/Copyright
// All 9-DOF code taken from Kris Winer

#define PI 3.14159

//
//       Pinout
//
#define BUTTON      3 // Screen switching button
// LCD Display
//Software SPI (slower updates, more flexible pin options)
#define SPI_CLK     4 // Serial clock out (CLK)
#define SPI_DIN     5 // Serial data out (DIN)
#define LCD_DC      6 // Data/Command select (D/C)
#define LCD_RST     8 // LCD reset (RST)
// GPS serial
#define GPS_RX      11 // "RX" on Trinket, "TX" on GPS
#define GPS_TX      12 // "TX" on Trinket, "RX" on GPS
#define BUTTON2     13 // Action button
#define NONEXISTANT_OUTPUT 10

//
//       EEPROM Data Addresses
//
#define CURRENT_VERSION  0x01
enum DataLocations {
  VERSION_ADDRESS        = 0,
  SCREEN_ADDRESS         = 1,
  GPS_ADDRESS_LAT        = 2,
  GPS_ADDRESS_LONG       = 6,
  GPS_ADDRESS_ALTITUDE   = 10,
  GPS_ADDRESS_DATE       = 14,
  GPS_ADDRESS_TIME       = 18,
  GPS_ADDRESS            = 24,
  MAG_BIAS               = 58,
  MAG_SCALE              = 70,
  NEXT                   = 82,
};

// 
//       Screens
//
enum Screens {
  SCREEN_GPS_SEARCH          = 0x01,
  SCREEN_GPS                 = 0x02,
  SCREEN_9DOF_ADDR           = 0x03,
  SCREEN_9DOF_RAW            = 0x04,
  SCREEN_9DOF_ACCEL          = 0x05,
  SCREEN_9DOF_GYRO           = 0x06,
  SCREEN_9DOF_MAG_CAL        = 0x07,
  SCREEN_9DOF_MAG            = 0x08,
  SCREEN_9DOF_TEMP           = 0x09,
  SCREEN_MAG_ADDR            = 0x0a,
  SCREEN_COMPASS             = 0x0b,
  SCREEN_9DOF_MAG_CAL_BIAS   = 0x0c,
  SCREEN_9DOF_MAG_CAL_SCALE  = 0x0d,
  SCREEN_9DOF_MAG_RAW        = 0x0e,
};
#define DEFAULT_SCREEN           SCREEN_GPS

//
//       GPS Module
//
SoftwareSerial mySerial(GPS_RX, GPS_TX);
#define GPS_SERIAL_RATE          9600

//
//       LCD Module
//
Adafruit_PCD8544 display = Adafruit_PCD8544(SPI_CLK, SPI_DIN, LCD_DC, NONEXISTANT_OUTPUT, LCD_RST);

//
//       9-DOF sensor module
//
// Wire ADO = 0 (ground)
// AK8962 (Magnometer on 9-DOF)
#define AK8963_ADDRESS           0x0C
#define AK8963_WHO_AM_I          0x00  // should return 0x48
#define AK8963_ST1               0x02  // data ready status bit 0
#define AK8963_XOUT_L            0x03  // data
#define AK8963_CNTL              0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASAX              0x10  // Fuse ROM x-axis sensitivity adjustment value

// MPU9250 (6-axis)
#ifdef ADO
#define MPU9250_ADDRESS          0x69  // Device address when ADO = 1
#else
#define MPU9250_ADDRESS          0x68  // Device address when ADO = 0
#endif
#define INT_PIN_CFG              0x37
#define INT_ENABLE               0x38
#define ACCEL_XOUT_H             0x3B
#define TEMP_OUT_H               0x41
#define GYRO_XOUT_H              0x43
#define PWR_MGMT_1               0x6B
#define WHO_AM_I_MPU9250         0x75  // Should return 0x71
enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};
uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0}, magscale[3] = {0, 0, 0};  // Factory mag calibration and mag biases from hand calibration

//
//       Global variables
//
TinyGPSPlus gps;
TinyGPSLocation gTargetLocation;
int gLedState = LOW;
int gpsLoaded = 0;
int screenToShow = DEFAULT_SCREEN;
int ledBlinking = 0;

void setup()
{
  pinMode(BUTTON, INPUT);
  digitalWrite(BUTTON, HIGH);
  pinMode(BUTTON2, INPUT);
  digitalWrite(BUTTON2, HIGH);
  pinMode(LED_BUILTIN, OUTPUT);

  // set the data rate for the SoftwareSerial port
  mySerial.begin(GPS_SERIAL_RATE);

  // I2C for 9-DOF sensor
  Wire.begin();

  // Get magnetometer calibration from AK8963 ROM
  initMPU9250();
  initAK8963(magCalibration);
    
  int storedVersion = EEPROM.read(VERSION_ADDRESS);
  if (storedVersion != CURRENT_VERSION) {
    for (int i = 0 ; i < EEPROM.length() ; i++) {
      EEPROM.write(i, 0);
    }
    EEPROM.write(SCREEN_ADDRESS, DEFAULT_SCREEN);
    EEPROM.write(VERSION_ADDRESS, CURRENT_VERSION);
  }
  screenToShow = EEPROM.read(SCREEN_ADDRESS);
  magbias[0] = EEPROM.read(MAG_BIAS);
  magbias[1] = EEPROM.read(MAG_BIAS+4);
  magbias[2] = EEPROM.read(MAG_BIAS+8);
  magscale[0] = EEPROM.read(MAG_SCALE);
  magscale[1] = EEPROM.read(MAG_SCALE+4);
  magscale[2] = EEPROM.read(MAG_SCALE+8);
  //for (int i=0; i<sizeof(gTargetLocation); i++) {
  //  ((byte*)gTargetLocation)[i] = EEPROM.read(GPS_ADDRESS+i);
  //}

  Timer1.initialize(1000000);
  Timer1.attachInterrupt(blinkLED); // blinkLED to run every 0.15 seconds
  ledBlinking=1;

  display.begin();
  display.setContrast(50);
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.clearDisplay();

  screenGPSSearch();
}

volatile unsigned long gDelay = 100000;

void blinkLED(void)
{
  // Toggle the LED
  if (gLedState == LOW) {
    gLedState = HIGH;
  } else {
    gLedState = LOW;
  }
  digitalWrite(LED_BUILTIN, gLedState);

  //Timer1.setPeriod(gDelay);
}

int gpsDataEverReceived;
int buttonDown=0;
int button2Down=0;
void loop() {
  //float heading;
  //double courseTo;

  // TODO: error conditions?

  bool dataReceived = false;
  while (mySerial.available() > 0) {
    gps.encode(mySerial.read());
    dataReceived = true;
    gpsDataEverReceived = true;
  }

  gpsLoaded = gpsLoaded | gps.location.isUpdated();
  if (ledBlinking && gpsLoaded) { gLedState = HIGH; blinkLED(); Timer1.stop(); }
  switch (screenToShow) {
    case SCREEN_GPS: screenGPS(); break;
    case SCREEN_9DOF_ADDR: screen9DOFAddr(); break;
    case SCREEN_9DOF_ACCEL: screen9DOFAccel(); break;
    case SCREEN_9DOF_MAG: screen9DOFMag(); break;
    case SCREEN_9DOF_MAG_RAW: screen9DOFMagRaw(); break;
    case SCREEN_9DOF_MAG_CAL: screen9DOFMagCal(); break;
    case SCREEN_9DOF_MAG_CAL_BIAS: screen9DOFMagCalBias(); break;
    case SCREEN_9DOF_GYRO: screen9DOFGyro(); break;
    case SCREEN_9DOF_TEMP: screen9DOFTemp(); break;
    case SCREEN_9DOF_RAW: screen9DOFRaw(); break;
    case SCREEN_GPS_SEARCH: screenGPSSearch(); break;
    case SCREEN_MAG_ADDR: screenMagAddr(); break;
    case SCREEN_COMPASS: screenCompass(); break;
    case SCREEN_9DOF_MAG_CAL_SCALE: screen9DOFMagCalScale(); break;
  }
    
  int buttonState = !digitalRead(BUTTON);
  int buttonPress = (buttonState && !buttonDown);
  buttonDown = buttonState;

  int button2State = !digitalRead(BUTTON2);
  int button2Press = (buttonState && !buttonDown);
  button2Down = button2State;

  if (button2Press) {
    // Action button
    switch (screenToShow) {
      case SCREEN_9DOF_MAG_CAL_BIAS:
      case SCREEN_9DOF_MAG_CAL_SCALE:
        magcalMPU9250(magbias, magscale);
        EEPROM.write(MAG_BIAS, magbias[0]);
        EEPROM.write(MAG_BIAS+4, magbias[1]);
        EEPROM.write(MAG_BIAS+8, magbias[2]);
        EEPROM.write(MAG_SCALE, magscale[0]);
        EEPROM.write(MAG_SCALE+4, magscale[1]);
        EEPROM.write(MAG_SCALE+8, magscale[2]);
        break;
      case SCREEN_GPS:
        //Serial.println("Updating target to current loc");
        gTargetLocation = gps.location;
        EEPROM.write(GPS_ADDRESS_LAT, gTargetLocation.lat());
        EEPROM.write(GPS_ADDRESS_LONG, gTargetLocation.lng());
        EEPROM.write(GPS_ADDRESS_ALTITUDE, gps.altitude.value());
        EEPROM.write(GPS_ADDRESS_DATE, gps.date.value());
        EEPROM.write(GPS_ADDRESS_TIME, gps.time.value());
        //for (int i=0; i<sizeof(gTargetLocation); i++) {
        //   EEPROM.write(GPS_ADDRESS+i, ((byte*)(void*)gTargetLocation)[i]);
        //}
        break;
      default:
        break; 
    }
  }
  if (buttonPress) {
    // Rotate screens
    switch (screenToShow) {
      case SCREEN_GPS:
      case SCREEN_GPS_SEARCH:
        screenToShow = SCREEN_9DOF_ACCEL; break;
      case SCREEN_9DOF_ADDR:
      case SCREEN_9DOF_RAW:
      case SCREEN_9DOF_ACCEL:
        screenToShow = SCREEN_9DOF_MAG_RAW; break;
      case SCREEN_9DOF_MAG_RAW:
        screenToShow = SCREEN_9DOF_MAG; break;
      case SCREEN_MAG_ADDR:
      case SCREEN_9DOF_MAG:
        screenToShow = SCREEN_9DOF_MAG_CAL; break;
      case SCREEN_9DOF_MAG_CAL:
        screenToShow = SCREEN_9DOF_MAG_CAL_BIAS; break;
      case SCREEN_9DOF_MAG_CAL_BIAS:
        screenToShow = SCREEN_9DOF_MAG_CAL_SCALE; break;
      case SCREEN_9DOF_MAG_CAL_SCALE:
        screenToShow = SCREEN_COMPASS; break;
      case SCREEN_COMPASS:
        screenToShow = SCREEN_GPS; break;
      case SCREEN_9DOF_GYRO:
      case SCREEN_9DOF_TEMP:
      default:
        screenToShow = SCREEN_GPS; break;
    }

    EEPROM.write(SCREEN_ADDRESS, screenToShow);    
  }

  delay(1);
}


void screen9DOFGyro() {
  display.clearDisplay();
  int16_t gyroData[3];
  readGyroData(gyroData);
  display.setCursor(0, 0 ); display.print("9-DOF Gyro");
  char gyro[13];
  sprintf(gyro, "%0.4x%0.4x%0.4x", gyroData[0], gyroData[1], gyroData[2]);
  display.setCursor(0, 10); display.print(gyro);
  display.setCursor(0, 20); display.print("X "); display.println(gyroData[0]);
  display.setCursor(0, 30); display.print("Y "); display.println(gyroData[1]);
  display.setCursor(0, 40); display.print("Z "); display.println(gyroData[2]);
  display.display();
}

void screen9DOFMagCal() {
   display.clearDisplay();
   display.setCursor(20,0); display.print("Mag F. Bias");
   display.setCursor(0,10); display.print("X MULT "); display.setCursor(50,10); display.print(magCalibration[0], 2);
   display.setCursor(0,20); display.print("Y MULT "); display.setCursor(50,20); display.print(magCalibration[1], 2);
   display.setCursor(0,30); display.print("Z MULT "); display.setCursor(50,30); display.print(magCalibration[2], 2);
   display.display();
}

void screen9DOFMagCalBias() {
   display.clearDisplay();
   display.setCursor(20,0); display.print("Mag Bias");
   display.setCursor(0,10); display.print("X BIAS "); display.setCursor(50,10); display.print(magbias[0], 2);
   display.setCursor(0,20); display.print("Y BIAS "); display.setCursor(50,20); display.print(magbias[1], 2);
   display.setCursor(0,30); display.print("Z BIAS "); display.setCursor(50,30); display.print(magbias[2], 2);
   display.display();
}

void screen9DOFMagCalScale() {
   display.clearDisplay();
   display.setCursor(20,0); display.print("Mag Scale");
   display.setCursor(0,10); display.print("X SCALE "); display.setCursor(50,10); display.print(magscale[0], 2);
   display.setCursor(0,20); display.print("Y SCALE "); display.setCursor(50,20); display.print(magscale[1], 2);
   display.setCursor(0,30); display.print("Z SCALE "); display.setCursor(50,30); display.print(magscale[2], 2);
   display.display();
}

void screen9DOFMagRaw() {
  display.setContrast(50);
  
  display.setTextSize(1);
  display.setTextColor(BLACK);
  int16_t magData[3];
  readMagData(magData);

  float mRes = getMres();

  // Calculate the magnetometer values in milliGauss
  // Include factory calibration per data sheet and user environmental corrections
  float mx,my,mz,mm;
  //if (magData[0] == 0) return;
  mx = (float)magData[0]*mRes*magCalibration[0];  // get actual magnetometer value, this depends on scale being set
  my = (float)magData[1]*mRes*magCalibration[1];
  mz = (float)magData[2]*mRes*magCalibration[2];
  mm = norm(mx, my, mz);

  display.clearDisplay();
  display.setCursor(0, 0 ); display.print("9-DOF Mag Raw");
  display.setCursor(0, 10); display.print("X   "); display.println(mx);
  display.setCursor(0, 20); display.print("Y   "); display.println(my);
  display.setCursor(0, 30); display.print("Z   "); display.println(mz);
  display.setCursor(0, 40); display.print("|M| "); display.println(mm);
  display.display();
}

void screen9DOFMag() {
  display.setContrast(50);
  
  display.setTextSize(1);
  display.setTextColor(BLACK);
  int16_t magData[3];
  
  readMagData(magData);

  float mRes = getMres();

  // Calculate the magnetometer values in milliGauss
  // Include factory calibration per data sheet and user environmental corrections
  float mx,my,mz,mm;
  if (magData[0] == 0) return;
  mx = ((float)magData[0]*mRes*magCalibration[0] - magbias[0]) * magscale[0];  // get actual magnetometer value, this depends on scale being set
  my = ((float)magData[1]*mRes*magCalibration[1] - magbias[1]) * magscale[1];
  mz = ((float)magData[2]*mRes*magCalibration[2] - magbias[2]) * magscale[2];
  mm = norm(mx, my, mz);

  if (mx != 0) {
    display.clearDisplay();
    display.setCursor(0, 0 ); display.print("9-DOF Mag");
    display.setCursor(0, 10); display.print("X   "); display.println(mx);
    display.setCursor(0, 20); display.print("Y   "); display.println(my);
    display.setCursor(0, 30); display.print("Z   "); display.println(mz);
    display.setCursor(0, 40); display.print("|M| "); display.println(mm);
    display.display();
  }
}


void screen9DOFAccel() {
  display.setContrast(50);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(BLACK);
  int16_t accelData[3];
  readAccelData(accelData);

  float aRes = 2.0/32768.0, ax, ay, az, am;
  // Now we'll calculate the accleration value into actual g's
  ax = (float)accelData[0]*aRes; // - accelBias[0];  // get actual g value, this depends on scale being set
  ay = (float)accelData[1]*aRes; // - accelBias[1];
  az = (float)accelData[2]*aRes; // - accelBias[2];
  am = norm(ax, ay, az);
  
  display.setCursor(0, 0 ); display.print("9-DOF Accel");
  //char accel[13];
  //sprintf(accel, "%0.4x%0.4x%0.4x", accelData[0], accelData[1], accelData[2]);
  //display.setCursor(0, 10); display.print(accel);
  display.setCursor(0, 10); display.print("X   "); display.println(ax, 3);
  display.setCursor(0, 20); display.print("Y   "); display.println(ay, 3);
  display.setCursor(0, 30); display.print("Z   "); display.println(az, 3);
  display.setCursor(0, 40); display.print("|A| "); display.println(am, 3);
  display.display();
}

void screen9DOFTemp() {
  display.setContrast(50);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(BLACK);
  int16_t tempRaw[1];
  tempRaw[1] = readTempData();
  char temp[5];
  display.setCursor(0, 0 ); display.print("9-DOF Temp");
  sprintf(temp, "%0.4x", tempRaw);
  display.setCursor(0, 10); display.print("raw: "); display.print(temp);
  display.display();
}

void screen9DOFRaw() {
  int16_t accelData[3], magData[3], gyroData[3], tempData[1];
  readAccelData(accelData);
  readGyroData(gyroData);
  readMagData(magData);
  tempData[0] = readTempData();
  
  display.clearDisplay();
  display.setCursor(0, 0 ); display.print("9-DOF raw");
  char accel[13],gyro[13],mag[13],temp[5];
  sprintf(accel, "%0.4x%0.4x%0.4x", accelData[0], accelData[1], accelData[2]);
  display.setCursor(0, 10); display.print(accel);
  sprintf(gyro, "%0.4x%0.4x%0.4x", gyroData[0], gyroData[1], gyroData[2]);
  display.setCursor(0, 20); display.print(gyro);
  sprintf(mag, "%0.4x%0.4x%0.4x", magData[0], magData[1], magData[2]);
  display.setCursor(0, 30); display.print(mag);
  sprintf(temp, "^AGM-T> %0.4x", tempData[0]);
  display.setCursor(0, 40); display.print(temp);
  display.display();
}

void screen9DOFAddr() {
  byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250

  display.clearDisplay();
  display.setCursor(20,0); display.print("MPU9250");
  display.setCursor(0,10); display.print("I AM");
  display.setCursor(0,20); display.print(c, HEX);
  display.setCursor(0,30); display.print("I Should Be");
  display.setCursor(0,40); display.print(0x71, HEX);
  display.display();
}

void screenMagAddr() {
  byte c = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);  // Read WHO_AM_I register for MPU-9250

  display.clearDisplay();
  display.setCursor(20,0); display.print("AK8963");
  display.setCursor(0,10); display.print("I AM");
  display.setCursor(0,20); display.print(c, HEX);
  display.setCursor(0,30); display.print("I Should Be");
  display.setCursor(0,40); display.print(0x48, HEX);
  display.display();
}

void screenCompass() {
  int16_t magData[3];
  readMagData(magData);

  float mRes = getMres();

  // Calculate the magnetometer values in milliGauss
  // Include factory calibration per data sheet and user environmental corrections
  float mx,my,mz,mm;
  if (magData[0] == 0) return;
  mx = ((float)magData[0]*mRes*magCalibration[0] - magbias[0]) * magscale[0];  // get actual magnetometer value, this depends on scale being set
  my = ((float)magData[1]*mRes*magCalibration[1] - magbias[1]) * magscale[1];
  mz = ((float)magData[2]*mRes*magCalibration[2] - magbias[2]) * magscale[2];
  mm = norm(mx, my, mz);

  int16_t accelData[3];
  readAccelData(accelData);
  float aRes = 2.0/32768.0, ax, ay, az, am;
  // Now we'll calculate the accleration value into actual g's
  ax = (float)accelData[0]*aRes; // - accelBias[0];  // get actual g value, this depends on scale being set
  ay = (float)accelData[1]*aRes; // - accelBias[1];
  az = (float)accelData[2]*aRes; // - accelBias[2];
  am = norm(ax, ay, az);
  
  display.clearDisplay();
  display.setCursor(0, 0 ); display.print("Mag. Compass");
  display.setCursor(0, 10); display.println(cood2rad(mx, my));
  display.setCursor(0, 20); display.println(rad2deg(cood2rad(mx, my)));
  display.display();
}

void screenGPSSearch() {
  display.clearDisplay();
  display.print("Searching for GPS signal");
  display.display();
}

void screenGPS() {
  if (!gpsLoaded) {
      int percentLoaded = 0;
      if (gpsDataEverReceived) percentLoaded=10;
      if (gps.time.value() != 0) percentLoaded=20;
      if (gps.date.value() != 0) percentLoaded=40;
      if (gps.location.lat() != 0) percentLoaded=60;
      if (gps.location.lng() != 0) percentLoaded=80;
      if (gps.satellites.value() != 0) percentLoaded=100;

      display.clearDisplay();
      display.print("searching "); display.print(percentLoaded); display.print("%");
      display.print(" lat "); display.println(gps.location.lat());
      display.print("long "); display.println(gps.location.lng());
      display.print("date "); display.println(gps.date.value());
      display.print("time "); display.println(gps.time.value());
      display.print("#sat "); display.println(gps.satellites.value());
      display.display();
    } else if (gps.location.isUpdated()) {
    //  double distanceM = gps.distanceBetween(
    //                       gps.location.lat(),
    //                       gps.location.lng(),
    //                       gTargetLocation.lat(),
    //                       gTargetLocation.lng()
    //                     );
  
    //  courseTo = gps.courseTo(
    //               gps.location.lat(),
    //               gps.location.lng(),
    //               gTargetLocation.lat(),
    //               gTargetLocation.lng()
    //             );
  
    //  heading = getHeading();
  
    //  unsigned long myDelay = ((unsigned long)getDelay(courseTo, heading)) * 1000;
  
    //  noInterrupts();
    //  gDelay = myDelay;
    //  interrupts();
  
    //  Serial.print("DISTANCE="); Serial.println(distanceM);
    //  Serial.print("COURSE=");   Serial.println(courseTo);
    //  Serial.print("HEADING=");  Serial.println(heading);
    //  Serial.print("DELAY=");    Serial.println(myDelay / 1000);
    //  Serial.println("---------------------------------");
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(BLACK);
      display.print(" lat "); display.println(gps.location.lat(), 3);
      display.print("long "); display.println(gps.location.lng(), 3);
      char date[9], time[9];
      sprintf(date, "%0.2d-%0.2d-%0.2d", gps.date.year()-2000, gps.date.month(), gps.date.day());
      sprintf(time, "%.2d:%0.2d:%0.2d", gps.time.hour(), gps.time.minute(), gps.time.second());
      display.print("date "); display.println(date);
      display.print("time "); display.println(time);
      //display.print("hdop "); display.println(gps.hdop.value());
      
      
    //  display.print("comp "); display.println(heading);
    //  display.print("dist "); display.println(distanceM);
    //  display.print("head "); display.println(courseTo);
      display.print(" alt "); display.println(gps.altitude.value(), 5);
      display.print("#sat "); display.println(gps.satellites.value());
      display.display();
    }
}

float mod(float a, float n) {
  return a - (floor(a / n) * n);
}

float norm_rad(float rad) {
  while (rad < 0) rad += PI * 2;
  while (rad > 2*PI) rad -= PI * 2;
  return rad;
}

float rad2deg(float rad) {
  return rad/PI*180;
}

float cood2rad(float x, float y) {
  if (x > 0 && y >= 0) return norm_rad(atan(y/x));
  if (x < 0 && y >= 0) return norm_rad(PI-atan(-y/x));
  if (x > 0 && y < 0) return norm_rad(-atan(-y/x));
  if (x < 0 && y < 0) return norm_rad(PI+atan(y/x));
  if (x == 0 && y >= 0) return PI/2;
  if (x == 0 && y < 0) return PI * (3/2);
}

int getDelay(float courseToLoc, float heading) {
  int angleBetween = courseToLoc - heading;
  angleBetween = mod(angleBetween + 180, 360) - 180;
  float delayFactor = abs(angleBetween / 180.0); // Between 0 and 1, 0 = on heading
  return 50 + 500 * delayFactor;
}

void initMPU9250() {
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
}

void initAK8963(float * destination) {
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3] = {0,0,0};  // x/y/z gyro calibration data stored here
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  delay(10);
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(10);
  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128)/256. + 1.;
  destination[2] =  (float)(rawData[2] - 128)/256. + 1.;
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
  delay(10);
}

/*
float getHeading()
{
  // Get a new sensor event
  sensors_event_t event;
  mag.getEvent(&event);

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);

  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0; //0.22;
  heading += declinationAngle;

  // Correct for when signs are reversed.
  if (heading < 0)
    heading += 2 * PI;

  // Check for wrap due to addition of declination.
  if (heading > 2 * PI)
    heading -= 2 * PI;

  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180 / M_PI;

  return headingDegrees;
}
*/

float norm(float x, float y, float z) {
  return sqrt(x*x+y*y+z*z);
}

//
// 9-DOF Sensor
//

float getMres() {
  switch (Mscale)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS: return 10.*4912./8190.; // Proper scale to return milliGauss
    case MFS_16BITS: return 10.*4912./32760.0; // Proper scale to return milliGauss
  }
  return 0;
}

void magcalMPU9250(float * dest1, float * dest2) 
{
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0};
  int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

  display.clearDisplay();
  display.println("Mag Calibration: Wave device in a figure eight until done!");
  display.display();

  delay(4000);
  if(Mmode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
  if(Mmode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
  for(ii = 0; ii < sample_count; ii++) {
    readMagData(mag_temp);  // Read the mag data   
    while (mag_temp[0]==0) { delay(10); readMagData(mag_temp); }
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    if(Mmode == 0x02) delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
    if(Mmode == 0x06) delay(12);  // at 100 Hz ODR, new mag data is available every 10 ms
  }

  display.clearDisplay();
  display.println("Mag cal done");
  display.display();

  mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

  float mRes = getMres();
  dest1[0] = (float) mag_bias[0]*mRes*magCalibration[0];  // save mag biases in G for main program
  dest1[1] = (float) mag_bias[1]*mRes*magCalibration[1];   
  dest1[2] = (float) mag_bias[2]*mRes*magCalibration[2];          

  float mag_scale[3];
  mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
  mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
  mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

  float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
  avg_rad /= 3.0;

  dest2[0] = avg_rad/((float)mag_scale[0]);
  dest2[1] = avg_rad/((float)mag_scale[1]);
  dest2[2] = avg_rad/((float)mag_scale[2]);
}


void readAccelData(int16_t * destination)
{
  uint8_t rawData[6] = {0,0,0,0,0,0};  // x/y/z accel register data stored here
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

void readGyroData(int16_t * destination)
{
  uint8_t rawData[6]={0,0,0,0,0,0};  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

void readMagData(int16_t * destination)
{
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  destination[0]=destination[1]=destination[2]=(int16_t)0;
  if(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
  readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
  uint8_t c = rawData[6]; // End data read by reading ST2 register
   if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
    destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
    destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
   }
  }
}

int16_t readTempData()
{
  uint8_t rawData[2]={0,0};  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
  return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}

// Wire.h read and write protocols
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    Wire.beginTransmission(address);  // Initialize the Tx buffer
    Wire.write(subAddress);           // Put slave register address in Tx buffer
    Wire.write(data);                 // Put data in Tx buffer
    Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
    uint8_t data = 0xFE; // `data` will store the register data     
    Wire.beginTransmission(address);         // Initialize the Tx buffer
    Wire.write(subAddress);                  // Put slave register address in Tx buffer
    Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address 
    data = Wire.read();                      // Fill Rx buffer with result
    return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
    Wire.beginTransmission(address);   // Initialize the Tx buffer
    Wire.write(subAddress);            // Put slave register address in Tx buffer
    Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
        Wire.requestFrom(address, count);  // Read bytes from slave register address 
    while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}

