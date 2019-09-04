#include <TinyGPS++.h>
/* 
   This sample sketch should be the first you try out when you are testing a TinyGPS++
   (TinyGPSPlus) installation.  In normal use, you feed TinyGPS++ objects characters from
   a serial NMEA GPS device, but this example uses static strings for simplicity.
*/


// The TinyGPS++ object
TinyGPSPlus gps;



void setup()
{
    Serial.begin(115200);
    
    
    while(!Serial.available());
    
    Serial.println(F("BasicExample.ino"));
    Serial.println(F("Basic demonstration of TinyGPS++ (no device needed)"));
    Serial.print(F("Testing TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
    Serial.println(F("by Mikal Hart"));
    Serial.println();

    Serial1.begin(9600);
  
    Serial.println();
    Serial.println(F("Done."));
}

bool gpsGet = 0;

void loop()
{
    while (Serial1.available())
    {
        if (gps.encode(Serial1.read()))
        {
            gpsGet = 1;
        }
        else
        {
            gpsGet = 0;
        }
    }

    displayInfo();
    
}

void displayInfo()
{
    
    if(!gpsGet)return;
    static unsigned long timer_s = millis();
    if(millis()-timer_s < 1000)return;
    timer_s = millis();
    
    Serial.print(F("Location: ")); 
    if (gps.location.isValid())
    {
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(","));
        Serial.print(gps.location.lng(), 6);
    }
    else
    {
        Serial.print(F("INVALID"));
    }

    Serial.print(F("  Date/Time: "));
    if (gps.date.isValid())
    {
        Serial.print(gps.date.month());
        Serial.print(F("/"));
        Serial.print(gps.date.day());
        Serial.print(F("/"));
        Serial.print(gps.date.year());
    }
    else
    {
        Serial.print(F("INVALID"));
    }

    Serial.print(F(" "));
    if (gps.time.isValid())
    {
        if (gps.time.hour() < 10) Serial.print(F("0"));
        Serial.print(gps.time.hour());
        Serial.print(F(":"));
        if (gps.time.minute() < 10) Serial.print(F("0"));
        Serial.print(gps.time.minute());
        Serial.print(F(":"));
        if (gps.time.second() < 10) Serial.print(F("0"));
        Serial.print(gps.time.second());
        Serial.print(F("."));
        if (gps.time.centisecond() < 10) Serial.print(F("0"));
        Serial.print(gps.time.centisecond());
    }
    else
    {
        Serial.print(F("INVALID"));
    }

    Serial.println();
}
