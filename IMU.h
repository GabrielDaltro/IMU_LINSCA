/*****************************************************************************

1 - Euler X'Y'Z'
2 - Norte geogr�fico alinhado com Y
3 - X perpendicular ao horizonte apontando para cima
4 - Z = X x Y

******************************************************************************/

#ifndef IMU_H
#define IMU_H
#include <math.h>
#include <Arduino.h>

#define idealSampleTime  5000 // em micro segundos

#define DX 3
#define DY 3
#define ND -(1<<16)

class IMU
{
    public:
  
        IMU();
      
        void updateOrientation (const float raw[9]); // AcX, AcY,AcZ, GyX,GyY,GyZ, Mx, My, Mz
        void update_orientation_compass (const float raw[9], const float* ptrPitch, const float* ptrYaw);
        

        float getAccRoll (void);
        float getAccPitch(void);
        float getAccYaw (void);
        float getGyRoll (void);
        float getGyPitch (void);
        float getGyYaw (void);
        float getcompRoll (void);
        void setR_Gy (float roll, float pitch, float yaw);
        void setR_GyBegin(const float raw[9]);
        void printOrientation(void);

        void setT_Gy(float tgy); 

    private:
        float AccRoll;
        float AccPitch;
        float AccYaw;
        float GyRoll;
        float GyPitch;
        float GyYaw;
        float compRoll;
        float R_Gy[3][3]  = {{1,0,0},{0,1,0},{0,0,1}};
        float T_Gy; // período de amostragem
      
        void update_orientation_accel(const float* ptrAcx, const float* ptrAcy, const float* ptrAcz);
        void update_orientation_gy(const float* ptrGyx, const float* ptrGyy, const float* ptrGyz);
        

        
};
#endif // IMU_H
