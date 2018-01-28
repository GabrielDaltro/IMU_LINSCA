#include "MPU9250.h"
#include "MPU9250_RegisterMap.h"
#define MPU_address 0x68
#define MAG_address 0x0C

void MPU9250::begin (int accScale, int gyScale)
{
    Wire.begin();
    Wire.setClock(400000);

    sendbyte(MPU9250_ACC_GY_ADDRESS,MPU9250_PWR_MGMT_1,0x80);
    sendbyte(MPU9250_ACC_GY_ADDRESS,MPU9250_PWR_MGMT_1,0x00);
    sendbyte(MPU9250_ACC_GY_ADDRESS,MPU9250_PWR_MGMT_1,0x00);
    delay(10);
    sendbyte(MPU9250_ACC_GY_ADDRESS,MPU9250_INT_PIN_CFG,0x02);
    delay(10);
    sendbyte(MPU9250_COMPASS_ADDRESS,MPU9250_CNTL1,0x06);
    delay(10);
    setAccScale(accScale);
    delay(10);
    setGyScale(gyScale);
    delay(10);
    sendbyte (MPU9250_ACC_GY_ADDRESS,MPU9250_CONFIGURATION,0);
    delay(10);
    //verifyRegister (MPU9250_ACC_GY_ADDRESS,MPU9250_GYRO_CONFIG);
   
    GyX_bias = 0;
    GyY_bias = 0;
    GyZ_bias = 0;
    GyBiasCalibration();   
}
void MPU9250::setGyScale (int scale)
{
    switch (scale)
    {
        case 250:  sendbyte(MPU9250_ACC_GY_ADDRESS,MPU9250_GYRO_CONFIG,0); GyScale = (float) 1.0/131.0; Serial.println ("Gy Scale 250");  break;
        case 500:  sendbyte(MPU9250_ACC_GY_ADDRESS,MPU9250_GYRO_CONFIG,8); GyScale = (float) 1.0/65.5;  Serial.println ("Gy Scale 500");  break;
        case 1000: sendbyte(MPU9250_ACC_GY_ADDRESS,MPU9250_GYRO_CONFIG,16);GyScale = (float) 1.0/32.8;  Serial.println ("Gy Scale 1000"); break;
        case 2000: sendbyte(MPU9250_ACC_GY_ADDRESS,MPU9250_GYRO_CONFIG,24);GyScale = (float) 1.0/16.4;  Serial.println ("Gy Scale 2000"); break;
        default:   sendbyte(MPU9250_ACC_GY_ADDRESS,MPU9250_GYRO_CONFIG,0); GyScale = (float) 1.0/131.0; Serial.println ("Gy Scale 250");  break;
    }
}
void MPU9250::setAccScale(int scale)
{
    switch (scale)
    {
        case 2:  sendbyte(MPU9250_ACC_GY_ADDRESS,MPU9250_ACCEL_CONFIG,0); AccScale = (float) 1.0/16384.0;break;
        case 4:  sendbyte(MPU9250_ACC_GY_ADDRESS,MPU9250_ACCEL_CONFIG,8); AccScale = (float) 1.0/8192.0; break;
        case 8:  sendbyte(MPU9250_ACC_GY_ADDRESS,MPU9250_ACCEL_CONFIG,16);AccScale = (float) 1.0/4096.0; break;
        case 16: sendbyte(MPU9250_ACC_GY_ADDRESS,MPU9250_ACCEL_CONFIG,24);AccScale = (float) 1.0/2048.0; break;
        default: sendbyte(MPU9250_ACC_GY_ADDRESS,MPU9250_ACCEL_CONFIG,0); AccScale = (float) 1.0/16384.0;break;
    }
}
float MPU9250::getAcX (void)
{
    return measures[0];
}
float MPU9250::getAcY (void)
{
    return measures[1];
}
float MPU9250::getAcZ (void)
{
    return measures[2];
}
float MPU9250::getTempCelsius(void)
{
    return measures[9];
}
float MPU9250::getGyX (void)
{
    return measures[3];
}
float MPU9250::getGyY (void)
{
    return measures[4];
}
float MPU9250::getGyZ (void)
{
    return measures[5];
}
float MPU9250::getMx  (void)
{
    return measures[6];
}
float MPU9250::getMy  (void)
{
    return measures[7];
}
float MPU9250::getMz  (void)
{
    return measures[8];
}
float* const MPU9250::getMeasures(void)
{
  return (float* const)measures;
}
void MPU9250::GyBiasCalibration (void)
{
    byte cont = 0;
    float soma[3] = {0,0,0};
    while (cont <  100)
    {
        doReadings();
        soma[0] += getGyX();
        soma[1] += getGyY();
        soma[2] += getGyZ();
        cont++;
    }
    GyX_bias = (float)soma[0]/cont;
    GyY_bias = (float)soma[1]/cont;
    GyZ_bias = (float)soma[2]/cont;
}
void MPU9250::doReadings(void)
{
    byte ST1;
    byte ST2;
    float aux;
     
    Wire.beginTransmission(MPU9250_ACC_GY_ADDRESS);
    Wire.write(MPU9250_ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU9250_ACC_GY_ADDRESS,14,false);
    measures[0]= -1*((int16_t)(Wire.read()<<8|Wire.read()))*AccScale;      //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    measures[1]= -1*((int16_t)(Wire.read()<<8|Wire.read()))*AccScale;  //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    measures[2]= -1*((int16_t)(Wire.read()<<8|Wire.read()))*AccScale;  //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    measures[9]=((int16_t)(Wire.read()<<8|Wire.read()))/333.87 + 21.0;  //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    measures[3]=((int16_t)(Wire.read()<<8|Wire.read()))*GyScale   - GyX_bias;  //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    measures[4]=((int16_t)(Wire.read()<<8|Wire.read()))*GyScale - GyY_bias;  //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    measures[5]=((int16_t)(Wire.read()<<8|Wire.read()))*GyScale - GyZ_bias;  //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    Wire.endTransmission(true);


   if ((measures[3] > -2) && (measures[3] < 2))
      measures[3] = 0;
   if ((measures[4] > -2) && (measures[4] < 2))
      measures[4] = 0;
   if ((measures[5] > -2) && (measures[5] < 2))
      measures[5] = 0;
          
    Wire.beginTransmission(MPU9250_COMPASS_ADDRESS);
    Wire.write(MPU9250_HXL);  // inicia leitura do registrador 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU9250_COMPASS_ADDRESS,7,false);
    Mg[0]=((int16_t)((Wire.read())|(Wire.read()<<8)))*compassScale;//1� byte Low 2� byte High
    Mg[1]=((int16_t)((Wire.read())|(Wire.read()<<8)))*compassScale;
    Mg[2]=((int16_t)((Wire.read())|(Wire.read()<<8)))*compassScale;
    ST2 = Wire.read();
    if(ST2 & 8) Serial.println("Overflow");
    Wire.endTransmission(true);
   
    measures[6] = 0;
    measures[7] = 0;
    measures[8] = 0;

    for(int i = 0 ; i < 3; i++) {
      for(int j = 0; j < 3; j++) {
        measures[i+6] += compassTransfMatrix[i][j] * (Mg[j] - compassBias[j]);
      }
    }

    aux = measures[6];
    measures[6] = measures[7];
    measures[7] = aux;
    measures[8] = -1*measures[8];
    
}
void MPU9250::exibeLeituras (void)
{
   // Serial.print ("AcX = ");
   // Serial.print (",");
    Serial.print(measures[0],4);
    Serial.print (",");
   // Serial.print ("AcY = ");
    Serial.print(measures[1],4);
    Serial.print (",");
    //Serial.print ("AcZ = ");
    Serial.print(measures[2],4);
    Serial.print (",");
    //Serial.print ("GyX = ");
    Serial.print(measures[3],4);
    Serial.print (",");
    //Serial.print ("GyY = ");
    Serial.print(measures[4],4);
    Serial.print (",");
    //Serial.print ("GyZ = ");
    Serial.print(measures[5],4);
    Serial.print (",");
    //Serial.print ("Mx = ");
    Serial.print(measures[6],4);
    Serial.print (",");
    //Serial.print ("My = ");
    Serial.print(measures[7],4);
    Serial.print (",");
    //Serial.print ("Mz = ");
    Serial.println(measures[8],4);
}
void MPU9250::sendbyte(byte device,byte address,byte value)
{
    Wire.beginTransmission(device);
    Wire.write(address);
    Wire.write(value);
    Wire.endTransmission(true);
}

void MPU9250::verifyRegister (byte device, byte reg)
{
     byte REGISTER = -50;    
     Wire.beginTransmission(device);
     Wire.write(reg);
     Wire.endTransmission(false);
     Wire.requestFrom(device,(byte)1,false);
    REGISTER = Wire.read(); 
    Wire.endTransmission(true); 
    Serial.print (reg);
    Serial.print(": ");
    Serial.println(REGISTER); 
    Serial.println();
}

