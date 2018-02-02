#include "KalmanFilter.h"

KalmanFilter::KalmanFilter (float a[][2], float X_init[][1], float P_init[][2], float R_1, float R_2, float Q_value, float H_1[2], float H_2[2] )
{
    matCopy_2x1(X_init,X); // X recebe X_init
    matCopy_2x2(P_init,P); // P recebe P_init
    matCopy_2x2(a,A);
    mat_2x2_transp (A,A_transp);
    calc_H (H_1,H_2,H,H_transp);
    Q[0][0] = Q_value; Q[0][1] = 0; Q[1][0] = 0; Q[1][1] = Q_value; //  Q = [Q_value 0; 0 Q_value];
    R[0][0] = R_1; R[0][1] = 0; R[1][0] = 0; R[1][1] = R_2; //  R = [R_1 0; 0 R_2];
    ident[0][0] = 1; ident[0][1] = 0; ident[1][0] = 0; ident[1][1] = 1;
}
KalmanFilter::KalmanFilter (float a[][2], float P_init[][2], float R_1, float R_2, float Q_value, float H_1[2], float H_2[2] ) 
{ // ESSE CONSTRUTOR FOI DECLARADO PARA SER USADO COM A FUNÇÃO setXinit()
    matCopy_2x2(P_init,P); // P recebe P_init
    matCopy_2x2(a,A);
    mat_2x2_transp (A,A_transp);
    calc_H (H_1,H_2,H,H_transp);
    Q[0][0] = Q_value; Q[0][1] = 0; Q[1][0] = 0; Q[1][1] = Q_value; //  Q = [Q_value 0; 0 Q_value];
    R[0][0] = R_1; R[0][1] = 0; R[1][0] = 0; R[1][1] = R_2; //  R = [R_1 0; 0 R_2];
    ident[0][0] = 1; ident[0][1] = 0; ident[1][0] = 0; ident[1][1] = 1;
}
void KalmanFilter::setXinit(float X_init[][1])
{
  matCopy_2x1(X_init,X); // X recebe X_init
}
 void KalmanFilter::setA(float a[][2])
 {
    matCopy_2x2(a,A);
    mat_2x2_transp (A,A_transp);  
 }
void KalmanFilter::applyFilter(float Z_1, float Z_2)
{
    float A_times_P[2][2];
    float A_times_P_times_Atransp[2][2];
    float P_pred_times_Htransp[2][2];
    float H_times_P_pred_times_Htransp[2][2];
    float H_times_P_pred_times_Htransp_R[2][2];
    float H_times_P_pred_times_Htransp_R_inv[2][2];
    float H_times_X_pred[2][1];
    float Z_less_H_times_X_pred[2][1];
    float K_times_Z_less_H_times_X_pred[2][1];
    float K_times_H[2][2];
    float I_less_K_times_H[2][2];

    #ifdef  USER_POS_ANG // caso as medições de posição sejam ângulos
      if (Z_1 > Z_2) // Z1 está proximo de 360 e Z2 passou de 360 e passou a ser próximo de 0
      {
        if (modulo(Z_1 - (Z_2 + 2*3.1416)) < 0.5236) // a diferença entre ele é menor que 30 graus?
        {
//          Serial.println();
//          Serial.println("************************");
//          Serial.print (Z_1); Serial.print (",");  Serial.println (Z_2);
//          Serial.println("************************");
          Z[0][0] = Z_1;
          Z[1][0] = Z_2 + 2*3.1416;
//          Serial.println();
//          Serial.println("************************");
//          Serial.print (Z[0][0]); Serial.print (",");  Serial.println (Z[1][0]);
//          Serial.println("************************");
        }
        else
        {
              Z[0][0] = Z_1;
              Z[1][0] = Z_2;
        }
      }
      else if (Z_2 > Z_1)
      {
        if (modulo(Z_2 - (Z_1 + 2*3.1416)) < 0.5236)  
        {
//          Serial.println();
//          Serial.println("************************");
//          Serial.print (Z_1); Serial.print (",");  Serial.println (Z_2);
//          Serial.println("************************");
          Z[0][0] = Z_1 + 2*3.1416;
          Z[1][0] = Z_2;
//          Serial.println("************************");
//          Serial.print (Z[0][0]); Serial.print (",");  Serial.println (Z[1][0]);
//          Serial.println("************************");
        }
        else
        {
              Z[0][0] = Z_1;
              Z[1][0] = Z_2;
        }
      }
      else // Z1 == Z2
      {
        if ((X[0][0] > 5.2360)&&(Z_1 >= 0)) // Z1 e Z2 são iguais, maiores ou iguais a zero e na interação passada, ambas eram maiores que 360
        {
//          Serial.println();
//          Serial.println("************************");
//          Serial.print (Z_1); Serial.print (",");  Serial.println (Z_2);
//          Serial.println("************************");
          Z[0][0] = Z_1 + 2*3.1416;
          Z[1][0] = Z_2 + 2*3.1416;
//          Serial.println("************************");
//          Serial.print (Z[0][0]); Serial.print (",");  Serial.println (Z[1][0]);
//          Serial.println("************************");
        }
        else 
        {
              Z[0][0] = Z_1;
              Z[1][0] = Z_2;
        }
      }
    #endif

    #ifndef USER_POS_ANG
        Z[0][0] = Z_1;
        Z[1][0] = Z_2;
    #endif
      

    //Prediction
    multMat_2x2_2x1(A,X,X_pred); // X_pred = A*X(k-1)
    multMat_2x2_2x2(A,P,A_times_P);
    multMat_2x2_2x2(A_times_P,A_transp,A_times_P_times_Atransp); // A*P*A'
    matSoma_2x2 (A_times_P_times_Atransp,Q,P_pred); // P_pred = A*P*A' + Q;


    //Corre��o - Calculo do ganho
    multMat_2x2_2x2(P_pred,H_transp,P_pred_times_Htransp); // P_pred*H'
    multMat_2x2_2x2(H,P_pred_times_Htransp,H_times_P_pred_times_Htransp); // H*P_pred*H'

    matSoma_2x2 (H_times_P_pred_times_Htransp,R,H_times_P_pred_times_Htransp_R); // ((H*P_pred*H')+ R)
    matInv_2x2 (H_times_P_pred_times_Htransp_R,H_times_P_pred_times_Htransp_R_inv); // ((H*P_pred*H')+ R)^-1
    multMat_2x2_2x2 (P_pred_times_Htransp,H_times_P_pred_times_Htransp_R_inv,K); //  K = (P_pred*H')*(((H*P_pred*H')+ R)^(-1));

    //Corre��o - Calculo do X e P
    multMat_2x2_2x1(H,X_pred,H_times_X_pred);
    matSub_2x1 (Z,H_times_X_pred,Z_less_H_times_X_pred);
    multMat_2x2_2x1 (K,Z_less_H_times_X_pred,K_times_Z_less_H_times_X_pred);
    matSoma_2x1 (X_pred,K_times_Z_less_H_times_X_pred,X);
    multMat_2x2_2x2 (K,H,K_times_H);
    matSub_2x2 (ident,K_times_H,I_less_K_times_H);
    multMat_2x2_2x2  (I_less_K_times_H,P_pred,P);

}

void KalmanFilter::calc_H (float H_1[2], float H_2[2], float H[][2],float H_transp[][2] ) //  H = [H_1' H_2']';
{


    H_transp[0][0] = H_1 [0];
    H_transp[1][0] = H_1 [1];
    H_transp[0][1] = H_2 [0];
    H_transp[1][1] = H_1 [1];


    mat_2x2_transp(H_transp,H);

}

void KalmanFilter::multMat_2x2_2x2 (float mat1[][2], float mat2[][2], float resp[][2])
{
    resp[0][0] = mat1[0][0]*mat2[0][0] + mat1[0][1]*mat2[1][0];
    resp[0][1] = mat1[0][0]*mat2[0][1] + mat1[0][1]*mat2[1][1];
    resp[1][0] = mat1[1][0]*mat2[0][0] + mat1[1][1]*mat2[1][0];
    resp[1][1] = mat1[1][0]*mat2[0][1] + mat1[1][1]*mat2[1][1];
}

void KalmanFilter::multMat_2x2_2x1 (float mat1[][2], float mat2[][1], float resp[][1])
{
    resp[0][0] = mat1[0][0]*mat2[0][0] + mat1[0][1]*mat2[1][0];
    resp[1][0] = mat1[1][0]*mat2[0][0] + mat1[1][1]*mat2[1][0];
}

void KalmanFilter::multMat_1x2_2x1 (float mat1[][2], float mat2[][1], float resp)
{
  resp = mat1[0][0]*mat2[0][0] + mat1[0][1]*mat2[1][0];
}

void KalmanFilter::mat_1x2_transp (float mat1[][2],float resp[][1] )
{
    resp[0][0] = mat1[0][0];
    resp[1][0] = mat1[0][1];
}

void KalmanFilter::mat_2x2_transp (float mat1[][2],float resp[][2] )
{
    resp[0][0] = mat1[0][0];
    resp[0][1] = mat1[1][0];
    resp[1][0] = mat1[0][1];
    resp[1][1] = mat1[1][1];
}

void KalmanFilter::matSoma_2x2 (float mat1[][2], float mat2[][2], float resp[][2])
{
    resp[0][0] = mat1[0][0] + mat2[0][0];
    resp[0][1] = mat1[0][1] + mat2[0][1];
    resp[1][0] = mat1[1][0] + mat2[1][0];
    resp[1][1] = mat1[1][1] + mat2[1][1];
}

void KalmanFilter::matSoma_2x1 (float mat1[][1], float mat2[][1], float resp[][1])
{
  resp[0][0] = mat1[0][0] + mat2[0][0] ;
  resp[1][0] = mat1[1][0] + mat2[1][0] ;
}

void KalmanFilter::matSub_2x2 (float mat1[][2], float mat2[][2], float resp[][2])
{
    resp[0][0] = mat1[0][0] - mat2[0][0];
    resp[0][1] = mat1[0][1] - mat2[0][1];
    resp[1][0] = mat1[1][0] - mat2[1][0];
    resp[1][1] = mat1[1][1] - mat2[1][1];
}

void KalmanFilter::matSub_2x1 (float mat1[][1], float mat2[][1], float resp[][1])
{
  resp[0][0] = mat1[0][0] - mat2[0][0] ;
  resp[1][0] = mat1[1][0] - mat2[1][0] ;
}

void KalmanFilter::matInv_2x2 (float mat1[][2],float resp[][2])
{
  float det_mat1 = mat1[0][0]*mat1[1][1] - mat1[1][0]*mat1[0][1];
  resp[0][0] = (float)mat1[1][1]/det_mat1;
  resp[0][1] = -1*mat1[0][1]/det_mat1;
  resp[1][0] = -1*mat1[1][0]/det_mat1;
  resp[1][1] = (float)mat1[0][0]/det_mat1;

}

void KalmanFilter::matCopy_2x1 (float mat1[][1], float mat2[][1])
{
    mat2[0][0] = mat1[0][0];
    mat2[1][0] = mat1[1][0];
}

void KalmanFilter::matCopy_2x2 (float mat1[][2], float mat2[][2])
{
    mat2[0][0] = mat1[0][0];
    mat2[1][0] = mat1[1][0];
    mat2[0][1] = mat1[0][1];
    mat2[1][1] = mat1[1][1];
}

//void KalmanFilter::dispMat_2x2 (float mat[][2])
//{
//    std::cout << "mat[0][0] = " << mat[0][0] << "\t mat[0][1] = " << mat[0][1] << "\nmat[1][0] = " << mat[1][0] << "\t mat[1][1] = " << mat[1][1] << "\n\n";
//
//}
//
//void KalmanFilter::dispMat_2x1 (float mat[][1]){
//    std::cout << "mat[0][0] = " << mat[0][0] << "\nmat[1][0] = " << mat[1][0] << "\n\n";
//}
//
//void KalmanFilter::dispMat_1x2 (float mat[2]){
//    std::cout << "mat[0] = " << mat[0] << "\tmat[1] = " << mat[1] << "\n\n";
//}
