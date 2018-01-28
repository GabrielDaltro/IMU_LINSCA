#include "MPU9250.h"
#include "KalmanFilter.h"
#include "IMU.h"

#define sampleTime  2000 // em micro segundos
#define  sendTime  5 // em milisegundos
int cont = 0;
/****************************************VARIÁVEIS DO FILTRO de KALMAN****************************************/
float R_1 = 1;
float R_2 = 1;
float R_MAG = 0.7;
float Q_value = 1;
float A[2][2] = {{1,0.005},{0, 1}};
float X_init_roll[2][1] = {{180},{0}};
float X_init_pitch[2][1] = {{0},{0}};
float X_init_yaw[2][1] = {{0},{0}};
float P_init[2][2] = {{0.1,0.1},{0.1,0.1}};
float H_1[2] = {1,0};
float H_2[2] = {1,0};
float X_roll[2][1];
float X_pich[2][1];
float X_yaw[2][1];
/****************************************DECLARAÇÃO DOS OBJETOS************************************************/
IMU imu;
MPU9250 mpu9250;

KalmanFilter FilterRoll  (A, X_init_roll,P_init,  R_MAG,  R_2, Q_value,  H_1,  H_2);
KalmanFilter FilterPitch (A, X_init_pitch,P_init,  R_1,  R_2, Q_value,  H_1,  H_2);
KalmanFilter FilterYaw   (A, X_init_yaw,P_init,  R_1,  R_2, Q_value,  H_1,  H_2);
/*************************************** VARIAVEIS PARA CONTROLE DE TEMPO*************************************/
unsigned long previousTime = 0;
unsigned long previousTime2 = 0;
unsigned long previousTime3 = 0;
unsigned long currentTime = 0;
unsigned long currentTime2 = 0;
unsigned long currentTime3 = 0;
/************************************************************************************************************/

void setup()
{
     Serial.begin(250000); 
     mpu9250.begin(2,500); 
      Serial.println ();
      Serial.println ();
      Serial.println ();
     mpu9250.doReadings();  
     imu.setR_GyBegin(mpu9250.getMeasures()); 
     
} 
void loop() 
{     
              currentTime = micros();
              if (currentTime - previousTime >= sampleTime)
              {
              /********************************************************************************/    
                   previousTime3 = micros(); 
                   mpu9250.doReadings();  
                   //mpu9250.exibeLeituras(); 
                   
                   // Calcula o pitch e o yaw do acelerômetro
                   // Calcula o roll, pitch e yaw do giroscópio                  
                   imu.updateOrientation(mpu9250.getMeasures()); 
                   imu.printOrientation();
                   // Faz a fusão do pitch e do yaw entre acelerômetro e giroscópio
                   FilterPitch.applyFilter((float)imu.getAccPitch(),(float)imu.getGyPitch());
                   FilterYaw.applyFilter((float)imu.getAccYaw(),(float)imu.getGyYaw());                 
                   FilterPitch.getX(X_pich);
                   FilterYaw.getX(X_yaw);

                   // Calculo do Roll usando o magnetômetro
                   imu.update_orientation_compass(mpu9250.getMeasures(),&X_pich[0][0],&X_yaw[0][0]);
                    
                   //Fusão entre os rolls do magnetômetro e giroscópio
                   //Serial.print ("GyRoll: "); Serial.println ((float)imu.getGyRoll());
                   //Serial.print ("compRoll: "); Serial.println ((float)imu.getcompRoll());
                   FilterRoll.applyFilter((float)imu.getcompRoll(),(float)imu.getGyRoll());
                   FilterRoll.getX(X_roll);
                   //Serial.print("RollFilter: ");
                   //Serial.println (X_roll[0][0]);

                   
                   //Atualiza a posição atual
                   cont++;
                   if (cont == 1000)
                   {
                      imu.setR_Gy (X_roll[0][0],X_pich[0][0],X_yaw[0][0]);
                   //imu.setR_Gy ((float)imu.getcompRoll(),(float)imu.getAccPitch(),(float)imu.getAccYaw());
                      cont = 0;                  
                   }


                   Serial.print ((X_roll[0][0]*180/3.14)-21);
                   Serial.print (",");
                   Serial.print (X_pich[0][0]*180/3.14);
                   Serial.print (",");
                   Serial.print (X_yaw[0][0]*180/3.14); 
                   Serial.print (",");
                   Serial.print(mpu9250.getGyZ());  
                   Serial.print (",   T:");
                   currentTime3 = micros();
                   Serial.println (currentTime3 - previousTime3);
                   
                  // if ((currentTime - previousTime) > (sampleTime + sampleTime/100)) {
                  //    Serial.print ("Muito Longo: ");
                  //    Serial.println (currentTime - previousTime);
                  // }
             /********************************************************************************/       
                previousTime =  micros();
              }
              
//              currentTime2 = millis();
//              if (currentTime2 - previousTime2 >= sendTime)
//              {
                /********************************************************************************/  
//                   Serial.print ((X_roll[0][0]*180/3.14)-21);
//                   Serial.print (",");
//                   Serial.print (X_pich[0][0]*180/3.14);
//                   Serial.print (",");
//                   Serial.print (X_yaw[0][0]*180/3.14); 
//                   Serial.print (",");
//                   Serial.print(mpu9250.getGyX());   
//                   //mpu9250.exibeLeituras(); 
//                   Serial.print (",");
//                   Serial.print(mpu9250.getGyY());   
//                   Serial.print (",");
//                   Serial.println(mpu9250.getGyZ());   
                   
                /********************************************************************************/
//                previousTime2 = millis();     
//              } 
        
}



















