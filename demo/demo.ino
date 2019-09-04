// gps demo

#include <TinyGPS++.h>

#include <SPI.h>
#include <SD.h>

#include "mcp_can.h"

const int SPI_CS_PIN = 9;

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

#define PID_ENGIN_PRM       0x0C
#define PID_VEHICLE_SPEED   0x0D
#define PID_COOLANT_TEMP    0x05
#define PID_INTAKE_TEMP     0x0F
#define CAN_ID_PID          0x7DF

// SET TIME ZONE HERE
const int timeZone  = 8;             // EAST 8, Beijing Time


int obdVehicleSpeed = 0;
int obdEngineRPM    = 0;
int obdCoolantTemp  = 0;
int obdIntakeTemp   = 0;


File dataFile;

char fileNameKml[20];
char fileNameCsv[20];

TinyGPSPlus gps;

bool gpsGetOnce = 0;
bool gpsGet = 0;


void set_mask_filt()
{
    // set mask, set both the mask to 0x3ff
    CAN.init_Mask(0, 0, 0x7FC);
    CAN.init_Mask(1, 0, 0x7FC);

    // set filter, we can receive id from 0x04 ~ 0x09

    CAN.init_Filt(0, 0, 0x7E8);                 
    CAN.init_Filt(1, 0, 0x7E8);

    CAN.init_Filt(2, 0, 0x7E8);
    CAN.init_Filt(3, 0, 0x7E8);
    CAN.init_Filt(4, 0, 0x7E8); 
    CAN.init_Filt(5, 0, 0x7E8);
}


bool gpsProcess()
{
    while (Serial1.available())
    {
        if (gps.encode(Serial1.read()))
        {
            gpsGet = gps.location.isValid() && gps.date.isValid() && gps.time.isValid() && (gps.date.day() != 0);
            gpsGetOnce = (!gpsGetOnce && gpsGet) ? 1 : gpsGetOnce;
            return gpsGet;
        }
        else
        {
            gpsGet = 0;
            return 0;
        }
    }
    
    gpsGet = 0;
    
    return 0;
}


void getLocalTime(int *year, int *mon, int *day, int *hour, int *min, int *sec)
{
    int _year   = gps.date.year();
    int _month  = gps.date.month();
    int _day    = gps.date.day();
    int _hour   = gps.time.hour();
    int _minute = gps.time.minute();
    
    unsigned char dayPerMonth[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    dayPerMonth[1] = (((_year%100!=0) && (_year%4==0)) || ( _year % 400==0)) ? 29 : 28;
    
    _hour += timeZone;
    
    if(_hour > 23)
    { 
        _hour -= 24;
        _day++;
        
        if(_day > dayPerMonth[_month-1])
        {
            _day = 1;
            _month = (_month == 12) ? 1 : _month+1;
        }
    }
    else if(_hour < 0)
    {
        _hour += 24;
        _day--;
        
        if(_day < 1)
        {
            _month = (_month == 1) ? 12 : _month-1;
            _day = dayPerMonth[_month-1];
        }
    }
    
    *year = _year;
    *mon  = _month;
    *day  = _day;
    *hour = _hour;
    *min  = _minute;
    *sec  = gps.time.second();
}

void makeFileName(char *nameKml, char *nameCsv)
{
    int _year   = gps.date.year();
    int _month  = gps.date.month();
    int _day    = gps.date.day();
    int _hour   = gps.time.hour();
    int _minute = gps.time.minute();
    int _second = gps.time.second();
    
    getLocalTime(&_year, &_month, &_day, &_hour, &_minute, &_second);
    
    nameKml[0] = '0' + (int)(_month/10);
    nameKml[1] = '0' + (int)(_month%10);
    
    nameKml[2] = '0' + (int)(_day/10);
    nameKml[3] = '0' + (int)(_day%10);
    
    nameKml[4] = '0' + (int)(_hour/10);
    nameKml[5] = '0' + (int)(_hour%10);
    
    nameKml[6] = '0' + (int)(_minute/10);
    nameKml[7] = '0' + (int)(_minute%10);
    
    nameKml[8] = '.';
    
    for(int i=0; i<=8; i++)
    {
        nameCsv[i] = nameKml[i];
    }
    
    nameKml[9] = 'k';
    nameKml[10] = 'm';
    nameKml[11] = 'l';
    nameKml[12] = '\0';

    nameCsv[9] = 'c';
    nameCsv[10] = 's';
    nameCsv[11] = 'v';
    nameCsv[12] = '\0';
}


// 

// 0 : no
// 1 : yes
unsigned char flgDriving = 0;
unsigned long timerStop;

void checkStart()
{
    if(flgDriving)return;
    if(!gpsGet)return;
    
    if(gps.speed.isValid() && gps.speed.kmph() > 10)       // when speed > 10 km/s
    {
        // start
        
        unsigned char dta[8];
        
        // speed
        if(getPidFromCar(PID_VEHICLE_SPEED, dta))
        {
            obdVehicleSpeed = dta[3];
        }
        else
        {
            delay(1000);
            return;
        }
        
        if(obdVehicleSpeed < 5)
        {
            delay(1000);
            return;
        }
    
        Serial.println("START TO TRACK");
        flgDriving = 1;
        makeFileName(fileNameKml, fileNameCsv);
            
        dataFile = SD.open(fileNameKml, FILE_WRITE);
        
        if(dataFile)
        {
            Serial.print("Open ");
            Serial.print(fileNameKml);
            Serial.println(" OK!");
            makeHeaderKml(fileNameKml);
            dataFile.close();
        }
        else
        {
            Serial.println("file open fail");
            dataFile.close();
        }
        
        dataFile = SD.open(fileNameCsv, FILE_WRITE);
        
        if(dataFile)
        {
            Serial.print("Open ");
            Serial.print(fileNameCsv);
            Serial.println(" OK!");
            makeHeaderCsv(fileNameCsv);
            dataFile.close();
        }
        else
        {
            Serial.println("file open fail");
            dataFile.close();
        }

        timerStop = millis();
    }
}

void checkStop()
{
    if(!flgDriving)return;
    
    if(obdVehicleSpeed > 0)
    {
        timerStop = millis();
    }
    
    if(millis()-timerStop > 300000)
    {
        Serial.println("STOP TRACK");
        flgDriving = 0;
        makeFooterKml();
    }
}

void setup() 
{
    Serial.begin(115200);
    Serial1.begin(9600);
    
    //while(!Serial.available());
    
    pinMode(13, OUTPUT);
    
    if (!SD.begin(4)) 
    {
        Serial.println("Card failed, or not present");
        return;
    }
    
    Serial.println("card initialized.");
    
    while (CAN_OK != CAN.begin(CAN_500KBPS))    // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");
    set_mask_filt();
}


void loop()
{
    gpsProcess();
    obdProcess();
    
    saveGPS();
    saveCsv();

    checkStart();
    checkStop();
    
    blink();
}

void saveGPS()
{
    if(!flgDriving)return;
    if(!gpsGet)return;
    if(obdVehicleSpeed == 0)return;
    
    static unsigned long timer_s = millis();
    if(millis()-timer_s < 5000)return;
    timer_s = millis();
    
    dataFile = SD.open(fileNameKml, FILE_WRITE);
    
    if(dataFile)
    {
        dataFile.print("\t\t\t\t");
        dataFile.print(gps.location.lng(), 10);
        dataFile.print(',');
        dataFile.print(gps.location.lat(), 10);
        dataFile.println(', ');
        dataFile.close();
    }
}

void saveCsv()
{
    if(!flgDriving)return;
    
    static unsigned long timer_s = millis();
    if(millis()-timer_s < 1000)return;
    timer_s = millis();
    
    dataFile = SD.open(fileNameCsv, FILE_WRITE);
    
    
    int _year   = gps.date.year();
    int _month  = gps.date.month();
    int _day    = gps.date.day();
    int _hour   = gps.time.hour();
    int _minute = gps.time.minute();
    int _second = gps.time.second();
    
    getLocalTime(&_year, &_month, &_day, &_hour, &_minute, &_second);

    if(dataFile)
    {
        dataFile.print(_hour);
        dataFile.print(":");
        dataFile.print(_minute);
        dataFile.print(":");
        dataFile.print(_second);
        dataFile.print(",");
        dataFile.print(obdVehicleSpeed);
        dataFile.print(",");
        dataFile.print(obdEngineRPM);
        dataFile.print(",");
        dataFile.print(gps.location.lng(), 10);
        dataFile.print(",");
        dataFile.println(gps.location.lat(), 10);
        dataFile.close();     
    }
    
}

void blink()
{
    static int ledstatus = 0;
    
    if(gpsGetOnce == 0)
    {
        static unsigned long timer_s = millis();
        if(millis()-timer_s < 50)return;
        timer_s = millis();
        
        ledstatus = 1-ledstatus;
        digitalWrite(13, ledstatus);
    }
    else if(flgDriving == 0)
    {
        static unsigned long timer_s = millis();
        if(millis()-timer_s < 200)return;
        timer_s = millis();
        
        ledstatus = 1-ledstatus;
        digitalWrite(13, ledstatus);
    }
    else
    {
        static unsigned long timer_s = millis();
        if(millis()-timer_s < 1000)return;
        timer_s = millis();
        
        ledstatus = 1-ledstatus;
        digitalWrite(13, ledstatus);
    }
}

void makeHeaderCsv(char *name)
{
    dataFile.println("Time,Speed,RPM,Longitude,Latitude");
}

void makeHeaderKml(char *name)
{
    dataFile.println(F("<?xml version=\"1.0\" encoding=\"UTF-8\"?>"));
    dataFile.println(F("<kml xmlns=\"http://www.opengis.net/kml/2.2\" xmlns:gx=\"http://www.google.com/kml/ext/2.2\" xmlns:kml=\"http://www.opengis.net/kml/2.2\" xmlns:atom=\"http://www.w3.org/2005/Atom\">"));
    dataFile.println(F("<Document>"));
    dataFile.print(F("\t<name>"));
    dataFile.print(name);
    dataFile.println(F("</name>"));
    dataFile.println(F("\t<Style id=\"s_ylw-pushpin_hl\">"));
    dataFile.println(F("\t\t<IconStyle>"));
    dataFile.println(F("\t\t\t<scale>1.3</scale>"));
    dataFile.println(F("\t\t\t<Icon>"));
    dataFile.println(F("\t\t\t\t<href>http://maps.google.com/mapfiles/kml/pushpin/ylw-pushpin.png</href>"));
    dataFile.println(F("\t\t\t</Icon>"));
    dataFile.println(F("\t\t\t<hotSpot x=\"20\" y=\"2\" xunits=\"pixels\" yunits=\"pixels\"/>"));
    dataFile.println(F("\t\t</IconStyle>"));
    dataFile.println(F("\t\t<LineStyle>"));
    dataFile.println(F("\t\t\t<color>ff0000ff</color>"));
    dataFile.println(F("\t\t\t<width>2</width>"));
    dataFile.println(F("\t\t</LineStyle>"));
    dataFile.println(F("\t</Style>"));
    dataFile.println(F("\t<Style id=\"s_ylw-pushpin\">"));
    dataFile.println(F("\t\t<IconStyle>"));
    dataFile.println(F("\t\t\t<scale>1.1</scale>"));
    dataFile.println(F("\t\t\t<Icon>"));
    dataFile.println(F("\t\t\t\t<href>http://maps.google.com/mapfiles/kml/pushpin/ylw-pushpin.png</href>"));
    dataFile.println(F("\t\t\t</Icon>"));
    dataFile.println(F("\t\t\t<hotSpot x=\"20\" y=\"2\" xunits=\"pixels\" yunits=\"pixels\"/>"));
    dataFile.println(F("\t\t</IconStyle>"));
    dataFile.println(F("\t\t<LineStyle>"));
    dataFile.println(F("\t\t\t<color>ff0000ff</color>"));
    dataFile.println(F("\t\t\t<width>2</width>"));
    dataFile.println(F("\t\t</LineStyle>"));
    dataFile.println(F("\t</Style>"));
    dataFile.println(F("\t<StyleMap id=\"m_ylw-pushpin\">"));
    dataFile.println(F("\t\t<Pair>"));
    dataFile.println(F("\t\t\t<key>normal</key>"));
    dataFile.println(F("\t\t\t<styleUrl>#s_ylw-pushpin</styleUrl>"));
    dataFile.println(F("\t\t</Pair>"));
    dataFile.println(F("\t\t<Pair>"));
    dataFile.println(F("\t\t\t<key>highlight</key>"));
    dataFile.println(F("\t\t\t<styleUrl>#s_ylw-pushpin_hl</styleUrl>"));
    dataFile.println(F("\t\t</Pair>"));
    dataFile.println(F("\t</StyleMap>"));
    dataFile.println(F("\t<Placemark>"));
    dataFile.println(F("\t\t<name>test</name>"));
    dataFile.println(F("\t\t<styleUrl>#m_ylw-pushpin</styleUrl>"));
    dataFile.println(F("\t\t<LineString>"));
    dataFile.println(F("\t\t\t<tessellate>1</tessellate>"));
    dataFile.println(F("\t\t\t<coordinates>"));
}

void makeFooterKml()
{
    dataFile = SD.open(fileNameKml, FILE_WRITE);
    
    if(dataFile)
    {
        dataFile.println(F("\t\t\t</coordinates>"));
        dataFile.println(F("\t\t</LineString>"));
        dataFile.println(F("\t</Placemark>"));
        dataFile.println(F("</Document>"));
        dataFile.println(F("</kml>"));
        dataFile.close();
    }
}


// OBD

// return:
// 0 - timeout
// 1 - ok

unsigned char getPidFromCar(unsigned char __pid, unsigned char *dta)
{
    unsigned char tmp[8] = {0x02, 0x01, __pid, 0, 0, 0, 0, 0};
    CAN.sendMsgBuf(CAN_ID_PID, 0, 8, tmp);
    
    unsigned long timer_s = millis();
    
    while(1)
    {
        if(millis()-timer_s > 100)
        {
            obdVehicleSpeed = 0;
            obdEngineRPM    = 0;
            return 0;
        }
        
        unsigned char len = 0;
        
        if(CAN_MSGAVAIL == CAN.checkReceive())                   // check if get data
        {
            
            CAN.readMsgBuf(&len, dta);    // read data,  len: data length, buf: data buf
            if(dta[1] == 0x41 && dta[2] == __pid)
            {
                return 1;
            }
        }
    }
    
    return 0;
}

void obdProcess()
{
    if(!flgDriving)return;
    static unsigned long timer_s = millis();
    if(millis()-timer_s < 1000)return;
    timer_s = millis();
    
    unsigned char dta[8];
    
    // speed
    if(getPidFromCar(PID_VEHICLE_SPEED, dta))
    {
        obdVehicleSpeed = dta[3];
    }
    
    // rpm
    if(getPidFromCar(PID_ENGIN_PRM, dta))
    {
        obdEngineRPM = (256.0*(float)dta[3]+(float)dta[4])/4.0;
    }
}


// END FILE