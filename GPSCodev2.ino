#include "types.h"
#include <TinyGPS++.h>
#include <HardwareSerial.h>
// Libraries for AWS
#include <AWS_IOT.h>
#include "WiFi.h"
//#include "EEPROM.h"
//#include "QmuTactile.h"

//#include <Wire.h>
//#include "SSD1306.h"
//#include "oled_display.h"

// Network/AWS Setup
AWS_IOT hornbill;                  // AWS_IOT instance
const char* ssid ="Das Wohnheim";  // Netword SSID
const char* password ="Devon123";  // Network password
char HOST_ADDRESS[]="a398bg7bdnhsp8-ats.iot.us-east-1.amazonaws.com";  // AWS custom endpoint Address

char CLIENT_ID[]= "DHT11";
char TOPIC_NAME[]= "ESP32/DHT11";
int status = WL_IDLE_STATUS;


TinyGPSPlus gps;
HardwareSerial SerialGPS(1);
//SSD1306 display(OLED_ADDR, OLED_SDA, OLED_SCL);
//OledDisplay oledDisplay(&display);

GpsDataState_t gpsState = {};

#define TASK_OLED_RATE 200
#define TASK_SERIAL_RATE 100

uint32_t nextSerialTaskTs = 0;
uint32_t nextOledTaskTs = 0;

char GPSPayload[1024];

// GPS Setup
TinyGPSPlus tinyGPS;     // Create a TinyGPSPlus object

// Initialize GPS variable
int thour;
int tmin;
int tsec;
int dday;
int dmonth;
int dyear;
float latitude;
float longitude;
float alt;
float ccourse;
float sspeed;
int sats;
float extemp;


void setup() {

    Serial.begin(115200);
    SerialGPS.begin(9600, SERIAL_8N1, 16, 17);


    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("Connecting to WiFi..");
    }

    Serial.println("Connected to wifi");

    // Print ESP32 Local IP Address
    Serial.println(WiFi.localIP());

    if(hornbill.connect(HOST_ADDRESS,CLIENT_ID)== 0) // Connect to AWS using Host Address and Client ID
    {
        Serial.println("Connected to AWS");
        delay(1000);
    }
    else
    {
        Serial.println("AWS connection failed, Check the HOST Address");
        while(1);
    }

    thour = 0;
    tmin = 0;
    tsec = 0;
    dday = 0;
    dmonth = 0;
    dyear = 0;
    latitude = 0;
    longitude = 0;
    alt = 0;
    ccourse = 0;
    sspeed = 0;
    sats = 0;
    extemp = 0;


}


void loop()
{
    static int p0 = 0;

    while (SerialGPS.available() > 0)
    {
        gps.encode(SerialGPS.read());
    }

    dyear = gps.date.year();
    dmonth = gps.date.month();
    dday = gps.date.day();
    thour = gps.time.hour();
    tmin = gps.time.minute();
    tsec = gps.time.second();

    latitude = gps.location.lat();
    longitude = gps.location.lng();
    alt = gps.altitude.meters();
    ccourse = gps.course.deg();
    sspeed = gps.speed.mph();
    sats = gps.satellites.value();


    if (nextSerialTaskTs < millis()) {
        Serial.print("Date: ");
        if(dday < 10)
            Serial.print("0");
        Serial.print(dday);
        Serial.print("/");
        if(dmonth < 10)
            Serial.print("0");
        Serial.print(dmonth);
        Serial.print("/");
        Serial.print(dyear);

        Serial.print("\n");

        Serial.print("Time: ");
        if(thour < 10)
            Serial.print("0");
        Serial.print(thour);
        Serial.print(":");
        if(tmin < 10)
            Serial.print("0");
        Serial.print(tmin);
        Serial.print(":");
        if(tsec < 10)
            Serial.print("0");
        Serial.print(tsec);

        Serial.print("\n");


        /*
        Serial.print("LAT=");  Serial.println(gps.location.lat(), 6);
        Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
        Serial.print("ALT=");  Serial.println(gps.altitude.meters());
        Serial.print("Sats=");  Serial.println(gps.satellites.value());
        Serial.print("DST: ");
        Serial.println(gpsState.dist,1);
        */



    Serial.println("Ran GPS aquire code");

    sprintf(GPSPayload, "Latitude: %f; Longitude: %f; Altitude: %f\nSpeed: %f; Course: %f; Sats: %d\nDate: %d/%d/%d; Time: %d:%d:%d",
                      latitude, longitude, alt, sspeed, ccourse, sats, dday, dmonth, dyear, thour, tmin, tinyGPS.time.second());

    Serial.println(GPSPayload);  // Print to serial monitor

    if(hornbill.publish(TOPIC_NAME, GPSPayload) == 0)  // Publish the message
    {
        Serial.println("Published Message:");
        Serial.println(GPSPayload);
    }
    else
    {
        Serial.println("Publish Failed");
    }

    // Publish every five seconds
    //vTaskDelay(5000 / portTICK_RATE_MS);


        nextSerialTaskTs = millis() + TASK_SERIAL_RATE;
    }

    //oledDisplay.loop();

}
