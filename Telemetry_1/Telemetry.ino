/*
   Radiolink R9D telemetry
   2019, wyfinger@yahoo.com
 */

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// DEBUG MODE
// false - without debug
// true - write raw sensor data to serial for SerialPlot (https://bitbucket.org/hyOzd/serialplot)
#define DEBUG true

// Battery Voltage pin
#define pinVoltage A3

// Accelerometer pins (GY_61, ADXL335)
// this module doesn't make much sense, for pitch, roll and yaw we need gyroscope
#define pinAccelX A0
#define pinAccelY A1
#define pinAccelZ A2

// Altimeter sensor (GY-BME280) software SPI pins
// https://www.aliexpress.com/item/-/32719675323.html
#define pinAirSCK  6     // SCL on module
#define pinAirMOSI 5     // SDA
#define pinAirCS   4     // CSB 
#define pinAirMISO 3     // SDO

// The BME280 is an integrated environmental sensor developed specifically for mobile
// applications where size and low power consumption are key design constraints.
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme(pinAirCS, pinAirMOSI, pinAirMISO, pinAirSCK);

// All telemetry info fields on Radiolink
struct RLTelemetry
{
  uint8_t  GPSSat;         // Numbers of GPS satellites
  float    altitude;       // 00.0 meters
  float    yaw;            // 00.0 deg
  float    speed;          // 00.0 m/s
  float    roll;           // 00.0 deg
  float    pitch;          // 00.0 deg
  float    distance;       // 00.0 m  
  float    rise;           // 00.0 m/s
  float    voltage;        // 00.0 V
  int32_t  longt;          // longitude
  int32_t  latit;          // latitude
} TI;

float zeroAlt = 1013.25;  // zero altitude pressure

// raw sensors data
int16_t rawVoltage, accelX, accelY, accelZ;

// recive packets might be two different types, they should be sent one by one
boolean packetSet = false;

// time markers
long voltesMark = 0, accelMark = 0, airMark = 0, logMark = 0;


void setup()
{
#if DEBUG
  Serial.begin(9600);  
#endif

  memset(&TI, 0, sizeof(TI));

  // voltage pins mode initialize
  // voltage sensor is a simple potential divider, it check one of full battery
  // voltage.
  pinMode(pinVoltage, INPUT);

  // Accel pins mode initialize
  pinMode(pinAccelX, INPUT);
  pinMode(pinAccelY, INPUT);
  pinMode(pinAccelZ, INPUT);
  // TODO: read adjustment rates

  // initialize air sensor
  bme.begin();
  zeroAlt = bme.readAltitude(SEALEVELPRESSURE_HPA);

  // initialize I2C
  Wire.begin(4);                // radiolink Telemetry port (I2C) initialize
  Wire.onRequest(requestEvent); // it must be connected to A4, A5 pins (hardware I2C)
}

void loop()
{
  checkVoltes();
  checkAccel();
  checkAir();
  printLog();
}

void checkVoltes()
{  
  if (millis() - voltesMark >= 500)
  {
    rawVoltage = analogRead(pinVoltage);
    TI.voltage = rawVoltage / 78.8091;   // <<--- voltage coefficient !!!!
    voltesMark = millis();
  }
}

void checkAccel()
{  
  if (millis() - accelMark >= 300)
  {
    accelX = analogRead(pinAccelX);
    accelY = analogRead(pinAccelY);
    accelZ = analogRead(pinAccelZ);
    TI.pitch = (map(accelY, 280, 430, 0, 1800)-900) / 10.0;   // <<--- accel coefficient !!!
    TI.roll  = (map(accelX, 280, 430, 0, 1800)-900) / 10.0;   // <<--- accel coefficient !!!
    accelMark = millis();
  }
}

void checkAir()
{
  if (millis() - airMark >= 400)
  {
    TI.altitude = bme.readAltitude(SEALEVELPRESSURE_HPA) - zeroAlt;
    airMark = millis();
  }
}

void printLog()
{
  if (millis() - logMark >= 250)
  {
#if DEBUG
    Serial.print(rawVoltage); Serial.print(","); 
    Serial.print(TI.voltage); Serial.print(",");
    Serial.print(accelX); Serial.print(","); 
    Serial.print(accelY); Serial.print(",");
    Serial.print(accelZ); Serial.print(",");
    Serial.print(TI.pitch); Serial.print(",");
    Serial.print(TI.roll); Serial.print(",");
    Serial.println(TI.altitude);
#endif
    logMark = millis();
  }
}

void requestEvent()
{
  if (packetSet)
  {
    packetSet = false;
    int alt = (int)(TI.altitude * 10);
    int yaw = (int)(TI.yaw * 10);
    int speed = (int)(TI.speed * 10);
    int roll = (int)(TI.roll * 10);
    int pitch = (int)(TI.pitch * 10.0);
    int distance = (int)(TI.distance * 10);
    byte buffer[16] = {0x89, 0xAB,
                        TI.GPSSat,
                        highByte(alt), lowByte(alt),
                        highByte(yaw), lowByte(yaw),
                        highByte(speed), lowByte(speed),
                        highByte(roll), lowByte(roll),
                        highByte(pitch), lowByte(pitch),
                        highByte(distance), lowByte(distance),
                        0x00};
    Wire.write(buffer, 16);
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else
  {
    packetSet = true;
    int rise = (int)(TI.rise * 10);
    uint16_t voltes = (TI.voltage * 1000.0);
    union u32_tag  {
      byte  b[4];
      uint32_t ui32;
    } latit, longt;
    longt.ui32 = TI.longt;
    latit.ui32 = TI.latit;
    byte buffer[16] = {0x89, 0xCD,
                        TI.GPSSat,
                        highByte(rise), lowByte(rise),
                        highByte(voltes), lowByte(voltes),
                        longt.b[3], longt.b[2], longt.b[1] , longt.b[0],
                        latit.b[3], latit.b[2], latit.b[1] , latit.b[0],
                        0x00};
    Wire.write(buffer, 16);
    digitalWrite(LED_BUILTIN, LOW);;
  }
}

