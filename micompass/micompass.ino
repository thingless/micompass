#include <SoftwareSerial.h>
#include "TinyGPS++.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <TimerOne.h>

TinyGPSPlus gps;

SoftwareSerial mySerial(6, 7); // RX, TX

TinyGPSLocation gTargetLocation;
int gLedState = LOW;

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

void setup()
{
  pinMode(10, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Open serial communications to the PC and wait for port to open:
  Serial.begin(57600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  Serial.println("Welcome!");

  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);

  Timer1.initialize(1000000);
  Timer1.attachInterrupt(blinkLED); // blinkLED to run every 0.15 seconds
}

volatile unsigned long gDelay = 1000000;

void blinkLED(void)
{
  // Toggle the LED
  if (gLedState == LOW) {
    gLedState = HIGH;
  } else {
    gLedState = LOW;
  }
  digitalWrite(LED_BUILTIN, gLedState);

  Timer1.setPeriod(gDelay);
}

void loop() // run over and over
{
  float heading;
  double courseTo;
  
  // TODO: error conditions?
  mag.begin();
  
  while (mySerial.available() > 0) {
    gps.encode(mySerial.read());
  }

  if(gps.location.isUpdated()) {
    Serial.print("LAT=");  Serial.println(gps.location.lat(), 6);
    Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
    Serial.print("ALT=");  Serial.println(gps.altitude.meters());
    Serial.print("DATE="); Serial.println(gps.date.value());
    Serial.print("TIME="); Serial.println(gps.time.value());
    Serial.print("HDOP="); Serial.println(gps.hdop.value());
    Serial.print("SATS="); Serial.println(gps.satellites.value());

    double distanceM = gps.distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      gTargetLocation.lat(),
      gTargetLocation.lng()
    );

    courseTo = gps.courseTo(
      gps.location.lat(),
      gps.location.lng(),
      gTargetLocation.lat(),
      gTargetLocation.lng()
    );

    heading = getHeading();

    unsigned long myDelay = ((unsigned long)getDelay(courseTo, heading)) * 1000;
    
    noInterrupts();
    gDelay = myDelay;
    interrupts();
    
    Serial.print("DISTANCE="); Serial.println(distanceM);
    Serial.print("COURSE=");   Serial.println(courseTo);
    Serial.print("HEADING=");  Serial.println(heading);
    Serial.print("DELAY=");    Serial.println(myDelay / 1000);
    Serial.println("---------------------------------");
  }

  int state = digitalRead(10);
  if (state) {
    Serial.println("Updating target to current loc");
    gTargetLocation = gps.location;
  }

  delay(1);
}

float mod(float a, float n) {
   return a - (floor(a/n) * n);
}

int getDelay(float courseToLoc, float heading) {
  int angleBetween = courseToLoc - heading;
  angleBetween = mod(angleBetween + 180, 360) - 180;
  float delayFactor = abs(angleBetween / 180.0); // Between 0 and 1, 0 = on heading
  return 50 + 500 * delayFactor;
}

float getHeading() 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.22;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI;
  
  return headingDegrees;
}

