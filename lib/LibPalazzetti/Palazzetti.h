#ifndef Palazzetti_h
#define Palazzetti_h

#include "Arduino.h"
#pragma once
class Palazzetti
{
public:
  Palazzetti(HardwareSerial *serial);
  char readRegistry(int addresse);
  void writeRegistry(int addresse, char value);
  void setPower(int level);
  int getPower();
  void powerOn();
  void powerOff();
  void setDate(struct tm timeinfo);
  void getDate(struct tm *timeinfo);
  int getState();
  long getT1();  
  long getT2();  
  long getT5();  
  long getExhausttemp();  
  int getSetPoint();  
  void setSetPoint(int setpoint);  

private:
  HardwareSerial *_serial;
  bool getTrame(char header, char *trame);
  bool read(char *trame);
  bool write(char *trame);
  char readRegistry(int addresse,char *trame);
  unsigned long _timeout = 200;
};

#endif