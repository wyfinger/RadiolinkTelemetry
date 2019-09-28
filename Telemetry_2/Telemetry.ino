/*
 * Radiolink R9D GPS telemetry
 * 2019, wyfinger@yahoo.com, MIT license
 * --
 * I use:
 *  - Arduino Nano and BN-280 GPS Module (https://www.aliexpress.com/item/-/33028598418.html);
 *  - Arduino Micro SD Storage Expansion (https://www.aliexpress.com/item/-/32869117358.html).
 */

#include <SPI.h>
#include <SD.h>           // ver 1.2.3
#include <SDConfigFile.h> // https://github.com/bneedhamia/sdconfigfile
#include <TinyGPS++.h>    // https://github.com/mikalhart/TinyGPSPlus
#include <SoftwareSerial.h>
#include <TimeLib.h>      // https://github.com/PaulStoffregen/Time
#include <Wire.h>

// Debug messages adds many bytes to code size and
// if turn on all debug messages program can have big
// size (over ~30 kb for Arduino Nano). If result size over
// 20-22 kb program can work uncorectly.
// Use no more than 1 flag at one point in time.
#define DEBUG_SETUP                 0    // print intro, test Serial
#define DEBUG_INIT_SD_CARD          0
#define DEBUG_READ_CONFIG           0
#define DEBUG_INIT_GPS              0
#define DEBUG_PREPARE_GPX_FILE      0
#define DEBUG_PUSH_POINT            0
#define DEBUG_REQUEST_TELEMETRY     0
#define DEBUG_CONFIG_GPS_MODULE     0    // work as serial bridge, experimental
#define DEBUG_VOLT                  0
#define DEBUG_BAND             115200

// GPS Module (Serial)
#define GPS_RX_PIN      3
#define GPS_TX_PIN      4
#define GPS_BAND    38400         // GPS module must be configurate to this speed,

// WARNING: I spent a lot of time determining what the problem was when messages
// for the RC receiver stopped coming. And I found that after calling ss.begin()
// the Wire.write() method stops working properly (reading messages works fine). 
// This bug repeats when SoftwareSerial runs at 9600 baud rate, so I set up the
// GPS receiver to 34800 baud speed.
// Test it!

// SD Storage Module (SPI)
// MOSI  11
// MISO  12
// CLK   13
// CS    SD_CS_PIN
#define SD_CS_PIN 10
//
#define VOLT_PIN  A3
// Radiolink Telemetry port (I2C) must be connected to A4, A5 pins (hardware I2C)

int gpsUtcOffset = 0;              // 'utc' param in 'gps.txt' file
byte gpsMinSat = 5;                // 'minsat' param in 'gps.txt' file
bool gpsFixed = false;
float voltKoef = 17.93077;
bool sdCardExists;

unsigned long tmrDist = 0;
unsigned long tmrRise = 0;
unsigned long tmrVolt = 0;
int32_t lastAlt = 0;
byte lastSat = 0;
double lastLat = 0;
double lastLng = 0;

char gpxFileName[9] = "0001.GPX"; 
unsigned int gpxPointCount = 0;
unsigned int tlmPointCount = 0;
byte lastGpxSec = 0;

bool packetSet = false;            // receive packets might be two different types, they should be sent one by one
int voltage = 0;
int rise = 0;
float distance = 0;

int32_t lastMillis = 0;

int yaw = 11;                     // values to show module status
int roll = 22;
int pitch = 33;

SoftwareSerial ss(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;
SDConfigFile cfg;

const char xmlHead[]        PROGMEM = "<?xml_head version=\"1.0\" encoding=\"UTF-8\"?>\n";
const char xmlGpx[]         PROGMEM = "<gpx xmlns=\"http://www.topografix.com/GPX/1/1\">\n";
const char xmlTimeA[]       PROGMEM = "\t\t\t\t<time>";
const char xmlTimeB[]       PROGMEM = "</time>\n";
const char xmlTrk[]         PROGMEM = "\t<trk>\n";
const char xmlNameA[]       PROGMEM = "\t\t<name>Flight track ";
const char xmlNameB[]       PROGMEM = "</name>\n";
const char xmlTrkseg[]      PROGMEM = "\t\t<trkseg>\n";
const char xmlTrkptA[]      PROGMEM = "\t\t\t<trkpt lat=\"";
const char xmlTrkptB[]      PROGMEM = "\" lon=\"";
const char xmlTrkptC[]      PROGMEM = "\">\n";
const char xmlTrkptD[]      PROGMEM = "\t\t\t</trkpt>\n";
const char xmlSatA[]        PROGMEM = "\t\t\t\t<sat>";
const char xmlSatB[]        PROGMEM = "</sat>\n";
const char xmlHdopA[]       PROGMEM = "\t\t\t\t<hdop>";
const char xmlHdopB[]       PROGMEM = "</hdop>\n";
const char xmlEleA[]        PROGMEM = "\t\t\t\t<ele>";
const char xmlEleB[]        PROGMEM = "</ele>\n";
const char xmlAgeA[]        PROGMEM = "\t\t\t\t<ageofdgpsdata>";
const char xmlAgeB[]        PROGMEM = "</ageofdgpsdata>\n";
const char xmlExtA[]        PROGMEM = "\t\t\t\t<extensions>\n";
const char xmlExtB[]        PROGMEM = "\t\t\t\t</extensions>\n";
const char xmlExtVoltA[]    PROGMEM = "\t\t\t\t\t<voltage>"; 
const char xmlExtVoltB[]    PROGMEM = "</voltage>\n";
const char xmlExtSpeedA[]   PROGMEM = "\t\t\t\t\t<speed>"; 
const char xmlExtSpeedB[]   PROGMEM = "</speed>\n";
const char xmlExtRiseA[]    PROGMEM = "\t\t\t\t\t<rise>"; 
const char xmlExtRiseB[]    PROGMEM = "</rise>\n";
const char xmlExtDistA[]    PROGMEM = "\t\t\t\t\t<distance>"; 
const char xmlExtDistB[]    PROGMEM = "</distance>\n";
const char xmlCork[]        PROGMEM = "\t\t</trkseg>\n\t</trk>\n</gpx>";




int getDight(float num, int dight) {

  int a = abs(dight);
  float b = dight <= 0 ? 10 : 0.1;
  float c = num;
  while (a-- > 0) {
    dight = dight - dight / abs(dight);
    c = c * b;
  }
  return abs((int)trunc(c) % 10);

}


char* getISO8601Time(char *res) {

  // ISO8601 time format is yyyy-mm-ddThh:mm:ssZ
  setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
  adjustTime(gpsUtcOffset * SECS_PER_HOUR);
 
  //time_t n = now();

  res[0]  = '0'+getDight(year(), 3);
  res[1]  = '0'+getDight(year(), 2);
  res[2]  = '0'+getDight(year(), 1);
  res[3]  = '0'+getDight(year(), 0);
  res[4]  = '-';
  res[5]  = '0'+getDight(month(), 1);
  res[6]  = '0'+getDight(month(), 0);
  res[7]  = '-';
  res[8]  = '0'+getDight(day(), 1);
  res[9]  = '0'+getDight(day(), 0);
  res[10] = 'T';
  res[11] = '0'+getDight(hour(), 1);
  res[12] = '0'+getDight(hour(), 0);
  res[13] = ':';
  res[14] = '0'+getDight(minute(), 1);
  res[15] = '0'+getDight(minute(), 0);
  res[16] = ':';
  res[17] = '0'+getDight(second(), 1);
  res[18] = '0'+getDight(second(), 0);
  res[19] = 'Z';
  res[20] = 0;
  return res;

}


void loop() {

  #if DEBUG_CONFIG_GPS_MODULE
    // work as serial port bridge 
    while (ss.available())
      Serial.write(ss.read());
    while (Serial.available())
      ss.write(Serial.read());
    return;  
  #endif 
  loopDist();
  loopGps();
  loopRise();
  loopVolt();

}


void loopDist() {

  // TinyGPSPlus::distanceBetween() method does not suit us
  // most arduino code for calculate distance between two points work
  // with big distance and give a big error with small position changes
  // see more https://jeonlab.wordpress.com/2013/10/29/gps-distance-measure-between-two-coordinates-using-arduino/
  
  if (millis() - tmrDist >= 800)  {     
    float lat = gps.location.lat();
    float lng = gps.location.lng();
    float delLat = fabs(lastLat-lat) * 111194.9;
    float delLong = 111194.9 * fabs(lastLng-lng)  * cos(radians((lastLat+lat)/2));
    float delta = sqrt(pow(delLat,2)+pow(delLong,2));
    // if distance more then 50m is it error
    // if distance lass than 5m is it small speed and big calc error
    if (delta > 5) {
      if (delta <= 50) {
        distance += delta;
      }  
      lastLat = lat;
      lastLng = lng;      
    }    
    tmrDist = millis();
  }

}


void loopGps() {

  // read data from GPS module
  while (ss.available())
    gps.encode(ss.read()); 

  // check of GPS initial fixed     
  if (gps.satellites.value() >= gpsMinSat) {
    if (!gpsFixed) {        
      #if DEBUG_INIT_GPS
        Serial.println(F("GPS fixed"));
      #endif
      prepareGpxFile();
      gpsFixed = true;
      lastGpxSec = gps.time.second();   // 2019-09-21 fix wrong time of first point
    }
  }

  // if satelites count changed show it
  #if DEBUG_INIT_GPS
    if ((!gpsFixed) && (gps.satellites.value() != lastSat)) {
      Serial.print(F("GPS find "));
      Serial.print(gps.satellites.value()); 
      Serial.print(F(" satelites, need ")); 
      Serial.println(gpsMinSat);
      lastSat = gps.satellites.value();
    }
  #endif  

  // if GPS fixed and time delta more than 1 second then write point to GPX file
  int delta = gps.time.second() - lastGpxSec;
  if (delta < 0) delta = 60 + delta;
  if (delta >= 1) {                              // GPX file points time period
    if(gpsFixed) pushGpxPoint();              
    lastGpxSec = gps.time.second();
  }

}


void loopRise() {

  // gps.altitude.value(gps.altitude.value()) saved in cm, rise speed must be saved in 0.1*m/s 
  if (millis() - tmrRise >= 1000)  {        
    rise = round( ((gps.altitude.value() - lastAlt) * 100.0) / (millis() - tmrRise) );    
    lastAlt = gps.altitude.value();
    tmrRise = millis();
  }

}


void loopVolt() {

  int voltRaw;
  if (millis() - tmrVolt >= 800)  { 
    voltRaw = analogRead(VOLT_PIN);
    voltage = voltRaw * voltKoef;   // this is a voltage measurement calibration coefficient
    tmrVolt = millis();  
  #if DEBUG_VOLT
    Serial.print(F("Voltage raw: "));
    Serial.print(voltRaw);
    Serial.print(", ");
    Serial.print(voltage);
    Serial.print("V, k=");
    Serial.println(voltKoef);
  #endif
  }
}


void prepareGpxFile() {    

  if (!sdCardExists) return;

  int fileNo = 1;

  while (SD.exists(gpxFileName)) {
    fileNo++;    
    gpxFileName[0] = '0'+getDight(fileNo, 3);
    gpxFileName[1] = '0'+getDight(fileNo, 2);
    gpxFileName[2] = '0'+getDight(fileNo, 1);
    gpxFileName[3] = '0'+getDight(fileNo, 0);
    if (fileNo > 9999) {    
      #if DEBUG_PREPARE_GPX_FILE
        Serial.println(F("Max file number achieved, work without file"));
      #endif
      sdCardExists = false;
      break;
    }
  }  

  #if DEBUG_PREPARE_GPX_FILE
    Serial.print(F("Create track file '"));
    Serial.print(gpxFileName);
    Serial.println("'");
  #endif  

  File gpxFile;
  gpxFile = SD.open(gpxFileName, FILE_WRITE);
  gpxFile.seek(0);

  char buf[50] = "";

  gpxFile.write(strcpy_P(buf, xmlHead));                // <?xml_head version="1.0" encoding="UTF-8"?>
  gpxFile.write(strcpy_P(buf, xmlGpx));                 // <gpx xmlns="http://www.topografix.com/GPX/1/1">
  gpxFile.write(strcpy_P(buf, xmlTimeA)+3);             // <time>2019-08-10T18:16:51Z</time>   +3 for delete first Tab symbols
  gpxFile.write(getISO8601Time(buf));
  gpxFile.write(strcpy_P(buf, xmlTimeB));
  gpxFile.write(strcpy_P(buf, xmlTrk));                 // <trk>
  gpxFile.write(strcpy_P(buf, xmlNameA));               // <name>Flight track 0002.GPX</name>
  gpxFile.write(gpxFileName); 
  gpxFile.write(strcpy_P(buf, xmlNameB));
  gpxFile.write(strcpy_P(buf, xmlTrkseg));              // <trkseg>

  // and write cork of GPX file
  gpxFile.write(strcpy_P(buf, xmlCork));                // </trkseg>      
                                                        // </trk>  
                                                        // </gpx>
  gpxFile.flush();
  gpxFile.close();

}


char* printNum(char* res, float num, unsigned int width, unsigned int digits) {

  unsigned int i;                       // there's nothing complicated here, 
  unsigned int hd = 0;                  // but will I be able to understand 
  unsigned int p = 0;                   // this code a year from now?
  unsigned int w = width==0 ? 8 : width;

  for (i = 0; i < w; i++) {
    res[i] = ' ';
    if (getDight(num, i) != 0) hd = i;
  }
  if (num < 0.0) {
    res[p++] = '-';
    num = -num;
  }  
  float rounding = 0.5;
  for (uint8_t i = 0; i < digits; ++i)
    rounding /= 10.0;
  num += rounding;
  for (i = 0; i <= hd; i++) {
    res[p++] = getDight(num, hd - i) + '0';
  }
  if ((digits > 0) && (p + 1 < w)) {
    res[p++] = '.';
    unsigned long int_part =  (unsigned long)num;
    double remainder = num - (double)int_part;
    while (digits-- > 0) {
      remainder *= 10.0;
       unsigned int toPrint = (unsigned int)(remainder);
       res[p++] = '0'+toPrint;
       remainder -= toPrint;
    }
  }
  if (width==0) res[p]=0;
  return res;

}


void pushGpxPoint() {

  byte bufSize = 59;
  char buf[bufSize] = "";

  #if DEBUG_PUSH_POINT
    if ((gpxPointCount % 20) == 0) {
       Serial.println(F("Volt  Sat  Latitude    Longitude   Alt    Speed  Rise  Dist"));
       Serial.println(F("V          deg         deg         m      km/h   m/s   m"));
       //                12.4  14   43.145379   131.906201  68.5   55.3   2.4   1454    
    }
    gpxPointCount++;
    for (byte i=0; i<bufSize; i++) buf[i]=' ';
    printNum(buf+0,  voltage/1000.0, 6, 1);           // voltage
    printNum(buf+6,  gps.satellites.value(), 5, 0);   // satellites count
    printNum(buf+11, gps.location.lat(), 12, 6);      // latitude
    printNum(buf+23, gps.location.lng(), 12, 6);      // longitude
    printNum(buf+35, gps.altitude.meters(), 5, 1);    // altitude
    printNum(buf+42, gps.speed.kmph(), 7, 1);         // speed
    printNum(buf+48, rise/10.0, 6, 1);                // rise
    printNum(buf+54, distance, 5, 0);                 // distance
    buf[bufSize]=0;
    Serial.println(buf);  
  #endif

  if (!sdCardExists) return;

  File gpxFile;
  gpxFile = SD.open(gpxFileName, O_WRITE);    // This is very important for SD library ver 1.2.3!
                                              // seek() function does not work right in this version 
                                              // if file was opened with FILE_WRITE flag,
                                              // see https://github.com/arduino-libraries/SD/issues/50
  // seek to back                                               
  gpxFile.seek(gpxFile.size()-strlen(xmlCork));

  gpxFile.write(strcpy_P(buf, xmlTrkptA));                      // <trkpt lat="59.934721667" lon="30.310183333">
  gpxFile.write(printNum(buf, gps.location.lat(), 0, 7));
  gpxFile.write(strcpy_P(buf, xmlTrkptB));
  gpxFile.write(printNum(buf, gps.location.lng(), 0, 7));  
  gpxFile.write(strcpy_P(buf, xmlTrkptC));
                           
  gpxFile.write(strcpy_P(buf, xmlTimeA));                       // <time>2019-08-10T18:16:51Z</time>
  gpxFile.write(getISO8601Time(buf));
  gpxFile.write(strcpy_P(buf, xmlTimeB));

  gpxFile.write(strcpy_P(buf, xmlAgeA));                        // <ageofdgpsdata>0.351</ageofdgpsdata>
  gpxFile.write(printNum(buf, gps.time.age()/1000.0, 0, 3));
  gpxFile.write(strcpy_P(buf, xmlAgeB));
 
  gpxFile.write(strcpy_P(buf, xmlSatA));                        // <sat>8</sat>
  gpxFile.write(printNum(buf, gps.satellites.value(), 0, 0));
  gpxFile.write(strcpy_P(buf, xmlSatB));
  
  gpxFile.write(strcpy_P(buf, xmlHdopA));                       // <hdop>8</sat>
  gpxFile.write(printNum(buf, gps.hdop.value(), 0, 0));
  gpxFile.write(strcpy_P(buf, xmlHdopB));

  gpxFile.write(strcpy_P(buf, xmlEleA));                        // <ele>45.4</ele>
  gpxFile.write(printNum(buf, gps.altitude.meters(), 0, 1));
  gpxFile.write(strcpy_P(buf, xmlEleB));

  gpxFile.write(strcpy_P(buf, xmlExtA));                        // <extensions>
  gpxFile.write(strcpy_P(buf, xmlExtVoltA));                    //   <voltage>12.2</voltage>
  gpxFile.write(printNum(buf,  voltage/1000.0, 0, 1));
  gpxFile.write(strcpy_P(buf, xmlExtVoltB));
  gpxFile.write(strcpy_P(buf, xmlExtSpeedA));                   //   <speed>45.2</speed>
  gpxFile.write(printNum(buf, gps.speed.kmph(), 0, 1));
  gpxFile.write(strcpy_P(buf, xmlExtSpeedB));
  gpxFile.write(strcpy_P(buf, xmlExtRiseA));                    //   <rise>45.2</rise>
  gpxFile.write(printNum(buf, rise/10.0, 0, 1));
  gpxFile.write(strcpy_P(buf, xmlExtRiseB));
  gpxFile.write(strcpy_P(buf, xmlExtDistA));                    //   <distance>56424</distance>
  gpxFile.write(printNum(buf, distance, 0, 1));
  gpxFile.write(strcpy_P(buf, xmlExtDistB));
  gpxFile.write(strcpy_P(buf, xmlExtB));                        // </extensions>

  gpxFile.write(strcpy_P(buf, xmlTrkptD));                      // </trkpt>

  gpxFile.write(strcpy_P(buf, xmlCork));                        // </trkseg>
                                                                // </trk> 
  gpxFile.flush();                                              // </gpx>
  gpxFile.close();

}


void readConfig() {

  const char *cfgFileName = "gps.txt"; 
  File cfgFile;
  if (sdCardExists) {
    if (!cfg.begin(cfgFileName, 80)) {
      #if DEBUG_READ_CONFIG
        Serial.print(F("Config file '"));
        Serial.print(cfgFileName);
        Serial.println(F("' does not exists, create blank"));
      #endif
      cfgFile = SD.open(cfgFileName, FILE_WRITE);
      if (!cfgFile) {
        #if DEBUG_READ_CONFIG
          Serial.println(F("Error on create blank config file"));
        #endif  
      } else {        
        cfgFile.print("utc=");  cfgFile.println(gpsUtcOffset);
        cfgFile.print("sat=");  cfgFile.println(gpsMinSat);
        cfgFile.print("volt=");  cfgFile.println(voltKoef);

        cfgFile.close();
      }
    } else {
      #if DEBUG_READ_CONFIG
        Serial.print(F("Read config file '"));
        Serial.print(cfgFileName);
        Serial.println(F("' contents, set settings:"));
      #endif  
      while (cfg.readNextSetting()) {
        if (cfg.nameIs("utc")) {
          gpsUtcOffset = cfg.getIntValue();
        } else if (cfg.nameIs("sat")) {
          gpsMinSat = cfg.getIntValue();
        } else if (cfg.nameIs("volt")) {
          String volt = cfg.getValue();
          voltKoef = volt.toFloat();
        }
      }
      cfg.end();
    }
  }
  #if DEBUG_READ_CONFIG
    Serial.print(F("UTC time offset is ")); Serial.println(gpsUtcOffset);
    Serial.print(F("Minimum sat count is ")); Serial.println(gpsMinSat);
  #endif

}


void requestTelem() {

  byte sat = gps.satellites.value();              // satelites count

  #if DEBUG_REQUEST_TELEMETRY
    if ((tlmPointCount % 20) == 0) {
      Serial.println(F("Sat  Alt  Speed  Rise  Volt   Yaw   Roll  Pitch  Dist"));
                     // 0                0     11744
                     // 0    0    0                   11    22    33     12545
    }
    tlmPointCount++;
    byte bufSize = 59;
    char buf[bufSize];
  #endif

  if (packetSet) {    

    int alt = gps.altitude.value() / 10;          // for radiolink 357 = 35.7m, gps.altitude.value() saved in cm
    int yaw = millis() % 0x0AFF;                  // this is a random values to show telemetry work status
    int roll = yaw % 0x00F0;
    int pitch = yaw % 0x00A5;
    int speed = gps.speed.mps() * 10;             // for radiolink 221 = 22.1m/s
    unsigned int dist = round(distance*10);

    byte buffer[16] = {0x89, 0xAB,
                        sat,
                        highByte(alt), lowByte(alt),
                        highByte(yaw), lowByte(yaw),
                        highByte(speed), lowByte(speed),
                        highByte(roll), lowByte(roll),
                        highByte(pitch), lowByte(pitch),
                        highByte(dist), lowByte(dist),
                        0x00};
    Wire.write(buffer, 16);
    packetSet = false;

    #if DEBUG_REQUEST_TELEMETRY
      for (byte i=0; i<bufSize; i++) buf[i]=' ';
      printNum(buf+0, sat, 5, 0);
      printNum(buf+5, alt, 6, 0);
      printNum(buf+10, speed, 7, 0);
      //printNum(buf+18, rise, 6, 0);
      //printNum(buf+24, voltage, 6, 0);
      printNum(buf+30, yaw, 6, 0);
      printNum(buf+36, roll, 6, 0);
      printNum(buf+42, pitch, 7, 0);
      printNum(buf+49, distance, 5, 0);
      buf[bufSize]=0;
      Serial.println(buf);  
    #endif
   
  } else {
    
    //calcRise();                           // for radiolink 123 = 12.3m/s

    union u32_tag  {
      byte  b[4];
      uint32_t ui32;
    } latit, longt;
    longt.ui32 = gps.location.lng()*10000000;
    latit.ui32 = gps.location.lat()*10000000; 

    byte buffer[16] = {0x89, 0xCD,
                        sat,
                        highByte(rise), lowByte(rise),
                        highByte(voltage), lowByte(voltage),
                        longt.b[3], longt.b[2], longt.b[1] , longt.b[0],
                        latit.b[3], latit.b[2], latit.b[1] , latit.b[0],
                        0x00};
    Wire.write(buffer, 16);
    packetSet = true;

    #if DEBUG_REQUEST_TELEMETRY
      for (byte i=0; i<bufSize; i++) buf[i]=' ';
      //printNum(buf+0, sat, 5, 0);
      //printNum(buf+5, alt, 6, 0);
      //printNum(buf+10, speed, 7, 0);
      printNum(buf+18, rise, 6, 0);
      printNum(buf+24, voltage, 6, 0);
      //printNum(buf+30, yaw, 6, 0);
      //printNum(buf+36, roll, 6, 0);
      //printNum(buf+42, pitch, 7, 0);
      //printNum(buf+49, distance, 5, 0);
      buf[bufSize]=0;
      Serial.println(buf);  
    #endif

  }

}


void setup() {
  #if DEBUG_SETUP || DEBUG_INIT_SD_CARD || DEBUG_READ_CONFIG || DEBUG_INIT_GPS || \
      DEBUG_PREPARE_GPX_FILE || DEBUG_PUSH_POINT || DEBUG_REQUEST_TELEMETRY || DEBUG_CONFIG_GPS_MODULE || DEBUG_VOLT
    Serial.begin(DEBUG_BAND);
    Serial.println(F("Radiolink GPS telemetry"));
  #endif  
  #if DEBUG_SETUP    
    Serial.println();    
    Serial.println();
    Serial.println(F("Radiolink GPS telemetry"));
    Serial.println(F("2019, wyfinger@yahoo.com, MIT license"));
    Serial.println(F("https://github.com/wyfinger/RadiolinkTelemetry"));
    Serial.println();
  #endif   

  setupTelem();
  setupVolt();
  setupGps();
  setupSdCard();

  readConfig();
}


void setupGps() {

  #if DEBUG_INIT_GPS
    Serial.println(F("Wait for GPS fix..."));
  #endif

  ss.begin(GPS_BAND);
  
}


void setupSdCard() {

  pinMode(SD_CS_PIN, OUTPUT);
  if (!SD.begin(SD_CS_PIN)) {
    #if DEBUG_INIT_SD_CARD
      Serial.println(F("SD card initialization failed!"));
    #endif
    sdCardExists = false;
   } else {
    #if DEBUG_INIT_SD_CARD
      Serial.println(F("SD card initialization done."));
    #endif
    sdCardExists = true;
  }

}


void setupTelem() {

  // init I2C for connect to radiolink recever telemetry port  
  Wire.begin(4);      // 4 is a address of I2C device  
  Wire.onRequest(requestTelem);

  tmrRise = 0;
  lastAlt = 0;
 
}


void setupVolt() {

  // battery voltage pin init
  pinMode(VOLT_PIN, INPUT);
  digitalWrite(VOLT_PIN, LOW);  

  tmrVolt = 0;

}


