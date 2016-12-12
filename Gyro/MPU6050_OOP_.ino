#include <Wire.h>
#include "Kalman.h"
#include "Gyroscope.h"

/*
a5 - SCL
a4 - sda
*/






const int SensorCount = 3;

//Сдвиговый регистр
//Пин подключен к ST_CP входу 74HC595
int latchPin = 8;
//Пин подключен к SH_CP входу 74HC595
int clockPin = 12;
//Пин подключен к DS входу 74HC595
int dataPin = 11;

Gyroscope Gyro[SensorCount];

void setup()
{
  
  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT); 
  pinMode(clockPin, OUTPUT);  
  Serial.begin(115200);
  Wire.begin();
  for(int i=1;i<=SensorCount;i++)
  {
    registerWrite(i, HIGH);
    Gyro[i].Init();
  }
}

void loop()
{
  while (Serial.available() == 0)
  {
    registerWrite(0, LOW);
  }
  int bitToSet = Serial.read() - '0';
    // Записываем HIGH в позицию соответствующую bitToSet
  if(bitToSet>0 && bitToSet<=SensorCount)
  {
    registerWrite(bitToSet, HIGH);
    Gyro[bitToSet-1].GetAxis();
  }
  
}



void registerWrite(int whichPin, int whichState) 
{
// инициализируем и обнуляем байт
  byte bitsToSend = 0;
 
  //Отключаем вывод на регистре
  digitalWrite(latchPin, LOW);
 
  // устанавливаем HIGH в соответствующем бите
  bitWrite(bitsToSend, whichPin, whichState);
 
  // проталкиваем байт в регистр
  shiftOut(dataPin, clockPin, MSBFIRST, bitsToSend);
 
    // "защелкиваем" регистр, чтобы байт появился на его выходах
  digitalWrite(latchPin, HIGH);
  
}

