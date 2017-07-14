#include <SoftwareSerial.h>
#include "TinyGPS++.h"
#include <Wire.h>
//#include <Adafruit_Sensor.h>
//#include <Adafruit_HMC5883_U.h>
#include <TimerOne.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

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
#define NONEXISTANT_OUTPUT 10

// 
//       Screens
//
#define SCREEN_GPS_SEARCH        0x01
#define SCREEN_GPS               0x02
#define SCREEN_9DOF_ADDR         0x03
#define SCREEN_9DOF              0x04
#define DEFAULT_SCREEN SCREEN_GPS

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
// Set ADO = 0 (ground)
#define MPU9250_ADDRESS          0x68  // Device address when ADO = 0
#define WHO_AM_I_MPU9250         0x75 // Should return 0x71

//
//       Global variables
//
TinyGPSPlus gps;
//TinyGPSLocation gTargetLocation;
int gLedState = LOW;
int gpsLoaded = 0;
int screenToShow = DEFAULT_SCREEN;
int ledBlinking = 0;

void setup()
{
  pinMode(BUTTON, INPUT);
  digitalWrite(BUTTON, HIGH);
  pinMode(LED_BUILTIN, OUTPUT);

  // set the data rate for the SoftwareSerial port
  mySerial.begin(GPS_SERIAL_RATE);

  // I2C for 9-DOF sensor
  Wire.begin();

  Timer1.initialize(1000000);
  Timer1.attachInterrupt(blinkLED); // blinkLED to run every 0.15 seconds
  ledBlinking=1;

  display.begin();
  display.setContrast(50);
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
    case SCREEN_9DOF: screen9DOF(); break;
    case SCREEN_GPS_SEARCH: screenGPSSearch(); break;
  }
    
  int buttonState = !digitalRead(BUTTON);
  int buttonPress = (buttonState && !buttonDown);
  buttonDown = buttonState;
  if (buttonPress) {
    // Rotate screens
    switch (screenToShow) {
      case SCREEN_GPS:
      case SCREEN_GPS_SEARCH:
        screenToShow = SCREEN_9DOF_ADDR; break;
      case SCREEN_9DOF_ADDR:
        screenToShow = SCREEN_9DOF; break;
      case SCREEN_9DOF:
        screenToShow = SCREEN_GPS; break;
      default:
        screenToShow = SCREEN_GPS; break;
    }
    
    //Serial.println("Updating target to current loc");
    //gTargetLocation = gps.location;
    
  }

  delay(1);
}

void screen9DOF() {
  display.setContrast(50);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(BLACK);  
  display.print("Not done");
  display.display();
}

void screen9DOFAddr() {
  byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  display.setContrast(50);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(BLACK);  
  display.setCursor(20,0); display.print("MPU9250");
  display.setCursor(0,10); display.print("I AM");
  display.setCursor(0,20); display.print(c, HEX);
  display.setCursor(0,30); display.print("I Should Be");
  display.setCursor(0,40); display.print(0x71, HEX);
  display.display();
}

void screenGPSSearch() {
  display.setContrast(50);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(BLACK);
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
      display.setTextSize(0);
      display.setTextColor(BLACK);
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
      display.print(" lat "); display.println(gps.location.lat(), 5);
      display.print("long "); display.println(gps.location.lng(), 5);
      char date[9], time[9];
      sprintf(date, "%0.2d-%0.2d-%0.2d", gps.date.year()-2000, gps.date.month(), gps.date.day());
      sprintf(time, "%.2d:%0.2d:%0.2d", gps.time.hour(), gps.time.minute(), gps.time.second());
      display.print("date "); display.println(date);
      display.print("time "); display.println(time);
      //display.print("hdop "); display.println(gps.hdop.value());
      
      
    //  display.print("comp "); display.println(heading);
    //  display.print("dist "); display.println(distanceM);
    //  display.print("head "); display.println(courseTo);
      display.print("#sat "); display.println(gps.satellites.value());
      display.display();
    }
}

float mod(float a, float n) {
  return a - (floor(a / n) * n);
}

int getDelay(float courseToLoc, float heading) {
  int angleBetween = courseToLoc - heading;
  angleBetween = mod(angleBetween + 180, 360) - 180;
  float delayFactor = abs(angleBetween / 180.0); // Between 0 and 1, 0 = on heading
  return 50 + 500 * delayFactor;
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

