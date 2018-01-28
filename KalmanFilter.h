#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#define USER_POS_ANG

#define TESTE 1

class KalmanFilter {
    public:
        KalmanFilter (float a[][2], float X_init[][1], float P_init[][2], float R_1, float R_2, float Q_value, float H_1[2], float H_2[2]  );
        KalmanFilter (float a[][2], float P_init[][2], float R_1, float R_2, float Q_value, float H_1[2], float H_2[2]  );
        void applyFilter(float Z_1, float Z_2);
        inline void getX (float X[][1]) {X[0][0] = this->X[0][0]; X[1][0] = this->X[1][0]; }

        void setXinit(float X_init[][1]);
        //void dispMat_2x2 (float mat[][2]);
        //void dispMat_2x1 (float mat[][1]);
        //void dispMat_1x2 (float mat[2]);
    private:
        float X[2][1]; // Vetor de estados
        float X_pred[2][1];
        float A[2][2]; // Matriz do sistemas
        float A_transp[2][2]; // transporta de A
        float P[2][2]; // Matriz de covari�ncia do erro
        float P_pred[2][2];
        float Q[2][2];
        float R[2][2];
        float H[2][2];
        float H_transp[2][2];
        float Z[2][1];
        float K[2][2];
        float ident[2][2];

        // Manipula��es de matrizes
       inline void multMat_2x2_2x2 (float mat1[][2], float mat2[][2], float resp[][2]);
       inline void multMat_2x2_2x1 (float mat1[][2], float mat2[][1], float resp[][1]);
       inline void multMat_1x2_2x1 (float mat1[][2], float mat2[][1], float resp);
       inline void mat_1x2_transp (float mat1[][2],float resp[][1] );
       inline void mat_2x2_transp (float mat1[][2],float resp[][2] );
       inline void matSoma_2x2 (float mat1[][2], float mat2[][2], float resp[][2]);
       inline void matSoma_2x1 (float mat1[][1], float mat2[][1], float resp[][1]);
       inline void matSub_2x2 (float mat1[][2], float mat2[][2], float resp[][2]);
       inline void matSub_2x1 (float mat1[][1], float mat2[][1], float resp[][1]);
       inline void matInv_2x2 (float mat1[][2],float resp[][2]);
       inline void matCopy_2x1(float mat1[][1], float mat2[][1]);
       inline void matCopy_2x2 (float mat1[][2], float mat2[][2]);


        //Calculo da matriz H
       inline void calc_H (float H_1[2], float H_2[2], float H[][2],float H_transp[][2] );

       // retorna o valor do módulo de um número
       float modulo (float num) 
       {
        if(num>0) return num; else return -1*num;
       }


};

#endif

