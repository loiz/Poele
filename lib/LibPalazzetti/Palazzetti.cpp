#include <Palazzetti.h>
#include <Arduino.h>
#include <CircularBuffer.h>
#include "time.h"

Palazzetti::Palazzetti(HardwareSerial *serial)
{
    Serial.println("Init stove");
    _serial = serial;
    _serial->begin(38400);
}

bool Palazzetti::getTrame(char header,char * trame)
{
  CircularBuffer<char, 11> rtrame;
  unsigned long startTime = millis();
  if (trame == NULL)
      _serial->flush();
  while(1==1)
  {
    if (millis()-startTime>_timeout)
    {
        //Serial.printf("Trame Erreur : %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X \r\n",rtrame[0],rtrame[1],rtrame[2],rtrame[3],rtrame[4],rtrame[5],rtrame[6],rtrame[7],rtrame[8],rtrame[9],rtrame[10]); 
        return false;
    }
    if(_serial->available())
    {
        if (!rtrame.push(_serial->read()))
        {
        if ((rtrame[0]==header))
        {
            //Serial.printf("Trame Detect : %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X \r\n",rtrame[0],rtrame[1],rtrame[2],rtrame[3],rtrame[4],rtrame[5],rtrame[6],rtrame[7],rtrame[8],rtrame[9],rtrame[10]); 
            char checkSum=0;
            for(int i=0;i<10;i++)
            checkSum+=rtrame[i];
            if(rtrame[10] == checkSum)
            {
                if (trame!=NULL)
                {
                    for (int i=0;i<11;i++)
                        trame[i]=rtrame[i];

                    //Serial.printf("Trame : %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X \r\n",trame[0],trame[1],trame[2],trame[3],trame[4],trame[5],trame[6],trame[7],trame[8],trame[9],trame[10]); 

                }
                return true;
            }
        }
        }
    }
  }
  return false;
}

bool Palazzetti::write(char *trame)
{
    if (getTrame(0x00,NULL))
    {
        //Serial.print("Trame Envoyee : ");
        for(int i=0;i<11;i++)
        {
            _serial->print(trame[i]);
            //Serial.printf("%.2x ",trame[i]);
            //delay(1);
        }
        //Serial.println("");
    }
    return false;
}

char Palazzetti::readRegistry(int addresse)
{
    char trame[11]={0,0,0,0,0,0,0,0,0,0,0};
    trame[0] = 0x02;
    trame[1] = lowByte(addresse);
    trame[2] = highByte(addresse);

    char checksum = 0;
    for(int i=0;i<10;i++)
        checksum+=trame[i];
    trame[10] = checksum;
    
    for(int i=0;i<10;i++)
    {
        write(trame);
        if (getTrame(0x02,trame))
        {
            return trame[1];
        }
        delay(200);
    }
    
    return 255;
}

void Palazzetti::writeRegistry(int addresse,char value)
{
    char trame[11]={0,0,0,0,0,0,0,0,0,0,0};
    trame[0] = 0x01;
    trame[1] = lowByte(addresse);
    trame[2] = highByte(addresse);
    trame[3] = value;

    char checksum = 0;
    for(int i=0;i<10;i++)
        checksum+=trame[i];
    trame[10] = checksum;

    write(trame);
}

void Palazzetti::setPower(int level)
{
    writeRegistry(0x202A,level);
}

int Palazzetti::getPower()
{
    return readRegistry(0x202A);
}

void Palazzetti::powerOn()
{
    writeRegistry(0x2044,0x02);
    writeRegistry(0x2045,0x00);
}

void Palazzetti::powerOff()
{
    writeRegistry(0x2044,0x01);
    writeRegistry(0x2045,0x00);
}

void Palazzetti::setDate(struct tm timeinfo)
{
    Serial.println("Set date");
    //writeRegistry(0x204E,timeinfo.tm_sec);
    writeRegistry(0x204F,/*timeinfo.tm_min*/02);
    //writeRegistry(0x2050,timeinfo.tm_hour);
    //writeRegistry(0x2051,1);
    //writeRegistry(0x2051,timeinfo.tm_mday);
    //writeRegistry(0x2052,timeinfo.tm_mon);
    //writeRegistry(0x2053,timeinfo.tm_year);
}

void Palazzetti::getDate(struct tm *timeinfo)
{
    Serial.println("Get date");
    timeinfo->tm_sec = (int)readRegistry(0x204E);
    timeinfo->tm_min = (int)readRegistry(0x204F);
    timeinfo->tm_hour = (int)readRegistry(0x2050);
    timeinfo->tm_mday = (int)readRegistry(0x2052);
    timeinfo->tm_mon = (int)readRegistry(0x2053);
    timeinfo->tm_year = (int)readRegistry(0x2054)+2000;
}

int Palazzetti::getState()
{
    return readRegistry(0x201C);
}

long Palazzetti::getT1()
{
    int t1 = readRegistry(0x200E);
    int t2 = readRegistry(0x200F);
    return ((t1<<8) +(t2))/10;
}  

long Palazzetti::getT2()
{
    int t1 = readRegistry(0x2010);
    int t2 = readRegistry(0x2011);
    return ((t1<<8) +(t2))/10;
}  
  
long Palazzetti::getT5()
{
    int t1 = readRegistry(0x2012);
    int t2 = readRegistry(0x2013);
    return ((t1<<8) +(t2))/10;
}  
  
long Palazzetti::getExhausttemp()
{
    int t1 = readRegistry(0x200A);
    int t2 = readRegistry(0x200B);
    return ((t1<<8) +(t2))/10;
}  

int Palazzetti::getSetPoint()
{
    return (int)readRegistry(0x1C33);
}

void Palazzetti::setSetPoint(int setpoint)
{
    writeRegistry(0x1c33,setpoint);
}

