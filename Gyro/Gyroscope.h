#ifndef Gyroscope_h
#define Gyroscope_h
#include <Wire.h>
#include <WProgram.h>
#include <Arduino.h>
// директивы #include и код помещается здесь

#endif


class Gyroscope
{
public:
  static const uint8_t IMUAddress = 0x69;
  void Init()
  {
    i2cWrite(0x6B,0x00); // Disable sleep mode  
    if(i2cRead(0x75,1)[0] != 0x68) 
    { // Read "WHO_AM_I" register
      Serial.print(F("MPU-6050 with address 0x"));
      Serial.print(IMUAddress,HEX);
      Serial.println(F(" is not connected"));
    }
    gyroXangle = 180; // Angle calculate using the gyro
    gyroYangle = 180;
    gyroZangle = 180;
    compAngleX = 180; // Calculate the angle using a Kalman filter
    compAngleY = 180;
    compAngleZ = 180;
    kalmanX.setAngle(180); // Set starting angle
    kalmanY.setAngle(180);
    kalmanZ.setAngle(180);
    timer = micros();
  }
  void GetAxis()
  {
    uint8_t* data = i2cRead(0x3B,14);
    accX = ((data[0] << 8) | data[1]);
    accY = ((data[2] << 8) | data[3]);
    accZ = ((data[4] << 8) | data[5]);  
    tempRaw = ((data[6] << 8) | data[7]);  
    gyroX = ((data[8] << 8) | data[9]);
    gyroY = ((data[10] << 8) | data[11]);
    gyroZ = ((data[12] << 8) | data[13]);

    /* Calculate the angls based on the different sensors and algorithm */
    

    accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
    accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;    
    accZangle = (atan2(accX,accY)+PI)*RAD_TO_DEG;
    gyroXrate = (double)gyroX/131.0;
    gyroYrate = -((double)gyroY/131.0);
    gyroZrate = -((double)gyroZ/131.0);
    gyroXangle += gyroXrate*((double)(micros()-timer)/1000000); // Считаем угол гироскопа без фильтра
    gyroYangle += gyroYrate*((double)(micros()-timer)/1000000);
    gyroZangle += gyroZrate*((double)(micros()-timer)/1000000);
 
    gyroXangle += kalmanX.getRate()*((double)(micros()-timer)/1000000); // Calculate gyro angle using the unbiased rate
    gyroYangle += kalmanY.getRate()*((double)(micros()-timer)/1000000);
    gyroZangle += kalmanZ.getRate()*((double)(micros()-timer)/1000000);
     
    compAngleX = (0.93*(compAngleX+(gyroXrate*(double)(micros()-timer)/1000000)))+(0.07*accXangle); // Считаем комплиментарный фильтр
    compAngleY = (0.93*(compAngleY+(gyroYrate*(double)(micros()-timer)/1000000)))+(0.07*accYangle);
    compAngleZ = (0.93*(compAngleZ+(gyroZrate*(double)(micros()-timer)/1000000)))+(0.07*accZangle);

    kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros()-timer)/1000000); // Считаем фильтр Кальмана
    kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros()-timer)/1000000);
    kalAngleZ = kalmanZ.getAngle(accZangle, gyroZrate, (double)(micros()-timer)/1000000);
    timer = micros();    
    /* Print Data */
    //Без Фильтрации  
    /*
    Serial.print("X");PrintValue((int)gyroXangle);
     Serial.print("Y");PrintValue((int)gyroXangle);
     Serial.print("Z");PrintValue((int)gyroXangle);
     */

    //Фильтр кальмана
    

    Serial.print("X");
    PrintValue((int)kalAngleX);
    Serial.print("Y");
    PrintValue((int)kalAngleY);
    Serial.print("Z");
    PrintValue((int)kalAngleZ);

    //Комплиментарный фильтр
    /*
    Serial.print("X");PrintValue((int)compAngleX);
     Serial.print("Y");PrintValue((int)compAngleY);
     Serial.print("Z");PrintValue((int)compAngleZ);
     */
    //Serial.print(temp);Serial.print("\t");

    Serial.print("\n");
  }

private:
  Kalman kalmanX;
  Kalman kalmanY;
  Kalman kalmanZ;
  int16_t accX;
  int16_t accY;
  int16_t accZ;
  int16_t tempRaw;
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;


  double accXangle; // Угол высчитываемый из акселерометра
  double accYangle;
  double accZangle;
  double gyroXangle; // Угол высчитываемый из гироскопа
  double gyroYangle;
  double gyroZangle;
  double compAngleX; // Угол для фильтра Кальмана
  double compAngleY;
  double compAngleZ;
  double kalAngleX; // Итоговый результат работы фильтра
  double kalAngleY;
  double kalAngleZ;
  double gyroXrate;
  double gyroYrate;
  double gyroZrate;
  uint32_t timer;

  void PrintValue (int input)
  {
    if(input<0)
    {
     Serial.print("N"); 
     input = -input;
    }
    else
    {
      Serial.print("P");
    }
    if(input<100)
    {
      Serial.print("0");
    }
    if(input<10)
    {
      Serial.print("0");
    }
    Serial.print(input);
  }

  void i2cWrite(uint8_t registerAddress, uint8_t data)
  {
    Wire.beginTransmission(IMUAddress);
    Wire.write(registerAddress);
    Wire.write(data);
    Wire.endTransmission(); // Send stop
  }
  uint8_t* i2cRead(uint8_t registerAddress, uint8_t nbytes)
  {
    uint8_t data[nbytes];  
    Wire.beginTransmission(IMUAddress);
    Wire.write(registerAddress);
    Wire.endTransmission(false); // Don't release the bus
    Wire.requestFrom(IMUAddress, nbytes); // Send a repeated start and then release the bus after reading
    for(uint8_t i = 0; i < nbytes; i++)
      data[i] = Wire.read();
    return data;
  }
};


