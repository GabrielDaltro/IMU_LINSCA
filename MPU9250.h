/**********************************************************************
 * Autor: Gabriel Daltro Duarte
 * Data: 23 de janeiro de 2018
 * Biblioteca testada no arduino Due
 * Versão: 1.0
 ********************************************************************/

#ifndef MPU9250_H
#define MPU9250_H

#include "MPU9250_RegisterMap.h"
#include <Arduino.h>
#include <Wire.h>

class MPU9250
 {
    public:
        void begin (int accScale, int gyScale);

        void doReadings(void);
        void exibeLeituras (void);

        float getAcX (void);
        float getAcY (void);
        float getAcZ (void);
        float getTempCelsius(void);
        float getGyX (void);
        float getGyY (void);
        float getGyZ (void);
        float getMx  (void);
        float getMy  (void);
        float getMz  (void);
        float* const getMeasures(void);

        void setCompassTransfMatrix (float mat[][3]);
        void setGyScale (int scale);
        void setAccScale(int scale);

    private:

        float Mg[3];
        float AccScale;
        float GyX_bias;
        float GyY_bias;
        float GyZ_bias;
        float GyScale;
        float Temp;
        float measures[10];
        float compassScale = 0.6;
        float compassTransfMatrix[3][3] = {{6.414,0.453,-0.066},{0.757,6.379,2.119},{-1.264,-0.905,4.469}};
        float compassBias[3] = {22.498,-21.015,-26.479};
        void GyBiasCalibration (void);

        void sendbyte(byte device,byte address,byte value);
        void verifyRegister (byte device, byte reg);

};



#endif // MPU9250_H
