#include "IMU.h"

IMU::IMU ()
{
  setT_Gy (idealSampleTime/1000000.0); // Inicilamente o tempo de amostragem é definido como 0.005 e deve ser atualizado a cada interação
}
void IMU::update_orientation_accel(const float* ptrAcx, const float* ptrAcy, const float* ptrAcz)
{
  float cos_theta = sqrt((*ptrAcx)*(*ptrAcx) + (*ptrAcy)*(*ptrAcy));
  
  AccPitch = atan2((-(*ptrAcz)),cos_theta);
  if(cos_theta > 0) 
  {
    AccRoll = ND;
    AccYaw = atan2(((*ptrAcy)/cos(AccPitch)),(-1*(*ptrAcx)/cos(AccPitch)));  
  }
  else 
  {
    AccRoll = 0;
    AccYaw = ND;  
  } 

  if (AccYaw < 0)
    AccYaw = AccYaw + 2*3.1416;

  if ((AccYaw >= 6.2483 ) && (AccYaw <= 6.2832))
    AccYaw = 0;
  else if (AccYaw <= 0.0349)
    AccYaw = 0;
    
}
void IMU::update_orientation_gy(const float* ptrGyx, const float* ptrGyy, const float* ptrGyz)
{
 
    float sum = 0 ;
  
    float sin_phi   = sin((*ptrGyx)*T_Gy*3.1416/180.0);
    float sin_theta = sin((*ptrGyy)*T_Gy*3.1416/180.0);
    float sin_psi   = sin((*ptrGyz)*T_Gy*3.1416/180.0);
    float cos_phi   = cos((*ptrGyx)*T_Gy*3.1416/180.0);
    float cos_theta = cos((*ptrGyy)*T_Gy*3.1416/180.0);
    float cos_psi   = cos((*ptrGyz)*T_Gy*3.1416/180.0);
    
    float xyz_euler[3][3] = {{cos_theta*cos_psi, -1*cos_theta*sin_psi, sin_theta},
                             {(sin_phi*sin_theta*cos_psi  + cos_phi*sin_psi),(-1*sin_phi*sin_theta*sin_psi + cos_phi*cos_psi), -1*sin_phi*cos_theta}, 
                             {(-1*cos_phi*sin_theta*cos_psi  + sin_phi*sin_psi),(cos_phi*sin_theta*sin_psi + sin_phi*cos_psi), cos_phi*cos_theta}};


//    for (int i = 0; i < 3; i++)
//    {
//      for (int j = 0; j < 3; j++)
//      {
//        Serial.print(xyz_euler[i][j]);
//        Serial.print(" ");
//      }
//      Serial.println ();
//    }
//    Serial.println();
//    for (int i = 0; i < 3; i++)
//    {
//      for (int j = 0; j < 3; j++)
//      {
//        Serial.print(R_Gy[i][j]);
//        Serial.print(" ");
//      }
//      Serial.println ();
//    }
//       Serial.println();
    float tmp[3][3] = {};
    for(int i = 0;i < DX; i++) {
      for(int j = 0; j < DY; j++) {
        for(int k = 0; k < 3; k++) {
              sum+=R_Gy[i][k] * xyz_euler[k][j];
        } 
        tmp[i][j]  = sum;
        sum = 0;
      }  
    }
   
    memcpy(R_Gy,tmp,DX*DY*sizeof(float));

//    for (int i = 0; i < 3; i++)
//    {
//      for (int j = 0; j < 3; j++)
//      {
//        Serial.print(R_Gy[i][j]);
//        Serial.print(" ");
//      }
//      Serial.println ();
//    }

    float cos_pitch = sqrt(R_Gy[0][0]*R_Gy[0][0] + R_Gy[0][1]*R_Gy[0][1]);
    GyPitch = atan2(R_Gy[0][2],cos_pitch);
    if(cos_theta > 0) 
    {
      GyRoll  = atan2((-1*R_Gy[1][2]/cos_pitch),(R_Gy[2][2]/cos_pitch));  
      GyYaw   = atan2((-1*R_Gy[0][1]/cos_pitch),(R_Gy[0][0]/cos_pitch));        
    }
    else 
    {
      GyRoll = 0;
      GyYaw = atan2(R_Gy[1][0],R_Gy[1][1]); 
    } 
//     Serial.println();
//    Serial.println("Roll,Pitch,Yaw(rad): ");
//    Serial.print (GyRoll);
//    Serial.print(",");
//    Serial.print (GyPitch);
//    Serial.print(",");
//    Serial.println(GyYaw);

    // converte para o intervalo 0 a 2pi
    if (GyRoll < 0)
      GyRoll = GyRoll + 2*3.1416;
    if (GyYaw < 0)
      GyYaw = GyYaw + 2*3.1416;

    //   
     if ((GyRoll >= 6.2483 ) && (GyRoll <= 6.2832))
      GyRoll = 0;
    else if (GyRoll <= 0.0349)
      GyRoll = 0;  

    if ((GyYaw >= 6.2483 ) && (GyYaw <= 6.2832))
      GyYaw = 0;
    else if (AccYaw <= 0.0349)
      GyYaw = 0;  

      
      
}
void IMU::updateOrientation (const float raw[9])
{
  update_orientation_accel(&(raw[0]),&(raw[1]),&(raw[2])); // 200 uS 
  update_orientation_gy (&raw[3],&raw[4],&raw[5]); // 215 uS
}
void IMU::update_orientation_compass (const float raw[9], const float* ptrPitch, const float* ptrYaw)
{
    float Hy;
    float Hz;
    float sin_theta = sin((*ptrPitch));
    float sin_psi   = sin((*ptrYaw));
    float cos_theta = cos((*ptrPitch));
    float cos_psi   = cos((*ptrYaw));

    Hy = (raw[6])*sin_psi + (raw[7])*cos_psi;
    Hz = (raw[6])*sin_theta*cos_psi + (raw[7])*sin_theta*sin_psi + (raw[8])*cos_theta;
    compRoll = atan2(Hy,Hz);
    if (compRoll < 0)
      compRoll = compRoll + 2*3.1416;
}
void IMU::setR_Gy (float roll, float pitch, float yaw)
{
    float sin_phi   = sin(roll);
    float sin_theta = sin(pitch);
    float sin_psi   = sin(yaw);
    float cos_phi   = cos(roll);
    float cos_theta = cos(pitch);
    float cos_psi   = cos(yaw);

    R_Gy[0][0] =    cos_theta*cos_psi;
    R_Gy[0][1] = -1*cos_theta*sin_psi;
    R_Gy[0][2] =    sin_theta;
    R_Gy[1][0] =    sin_phi*sin_theta*cos_psi  + cos_phi*sin_psi;
    R_Gy[1][1] = -1*sin_phi*sin_theta*sin_psi + cos_phi*cos_psi;
    R_Gy[1][2] = -1*sin_phi*cos_theta;
    R_Gy[2][0] = -1*cos_phi*sin_theta*cos_psi  + sin_phi*sin_psi;
    R_Gy[2][1] =    cos_phi*sin_theta*sin_psi + sin_phi*cos_psi;
    R_Gy[2][2] =    cos_phi*cos_theta;
}
void IMU::setT_Gy(float tgy)
{
  T_Gy = tgy;
}
void IMU::setR_GyBegin(const float raw[9])
{
  update_orientation_accel(&(raw[0]),&(raw[1]),&(raw[2])); // 200 uS 
  update_orientation_compass (raw, &AccPitch, &AccYaw);
  setR_Gy(compRoll,AccPitch,AccYaw);

  GyRoll = compRoll;
  GyPitch = AccPitch;
  GyYaw = AccYaw;

//  for (int i = 0; i < 3; i++)
//    {
//      for (int j = 0; j < 3; j++)
//      {
//        Serial.print(R_Gy[i][j]);
//        Serial.print(" ");
//      }
//      Serial.println ();
//    }
}
float IMU::getAccRoll (void)
{
  return AccRoll;
}
float IMU::getAccPitch(void)
{
  return AccPitch;
}
float IMU::getAccYaw (void)
{
  return AccYaw;
}
float IMU::getGyRoll (void)
{
  return GyRoll;
}
float IMU::getGyPitch (void)
{
  return GyPitch;
}
float IMU::getGyYaw (void)
{
  return GyYaw;
}
float IMU::getcompRoll (void)
{
  return compRoll;
}
void IMU::printOrientation(void)
{
   Serial.print(AccRoll*180/3.14);
   Serial.print (",");
   Serial.print(AccPitch*180/3.14);
   Serial.print (",");
   Serial.print(AccYaw*180/3.14);
   Serial.print (",");
   Serial.print(GyRoll*180/3.14);
   Serial.print (",");
   Serial.print(GyPitch*180/3.14);
   Serial.print (",");
   Serial.print(GyYaw*180/3.14);
   Serial.print (",");
   Serial.print(compRoll*180/3.14);
   Serial.print (",");   
}
