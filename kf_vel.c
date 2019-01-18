//飞行器速度估计值，为方便计算写成3阶矩阵的形式
float velocity[9] = {0, 0, 0,
                     0, 0, 0,
                     0, 0, 0};
//飞行器在地理系上的加速度值，为方便计算写成3阶矩阵的形式
float accel[9] = {0, 0, 0,
                 0, 0, 0,
                 0, 0, 0};
//飞行器的速度观测值，为方便计算写成3阶矩阵的形式
float Z[9] = {0, 0, 0,
           0, 0, 0,
           0, 0, 0};
//误差矩阵，为方便计算写成3阶矩阵的形式
float Y[9] = {0, 0, 0,
           0, 0, 0,
           0, 0, 0};
//状态转移矩阵
float F[9] = {1, 0, 0,
              0, 1, 0,
              0, 0, 1};
//状态转移矩阵的转置
float F_t[9] = {1, 0, 0,
              0, 1, 0,
              0, 0, 1};
//观测映射矩阵
float H[9] = {1, 0, 0,
               0, 1, 0,
               0, 0, 1};
//观测映射矩阵的转置
float H_t[9] = {1, 0, 0,
                0, 1, 0,
                0, 0, 1};
//过程矩阵，存放计算结果
float S[9] = {0, 0, 0,
                 0, 0, 0,
                 0, 0, 0};
//卡尔曼增益矩阵
float K[9] = {0, 0, 0,
                  0, 0, 0,
                  0, 0, 0};
//控制输入矩阵*deltaT deltaT=0.001S
float B[9] = {0.001, 0,  0,
                   0, 0.001, 0,
                   0, 0, 0.001};
//误差协方差矩阵
float P[9] = {0, 0, 0,
                   0, 0, 0,
                   0, 0, 0};
//单位矩阵
float I[9] =  {1, 0, 0,
                    0, 1, 0,
                    0, 0, 1};
//过程噪声协方差矩阵
float Q[9] = {0.005, 0, 0,
                    0, 0.005, 0,
                    0, 0, 0.001};
//测量噪声协方差矩阵
float R[9] = {100, 0,  0,
                    0, 100,  0,
                    0, 0, 3600};
 
//运行频率1KHz
//卡尔曼滤波，输入量为飞行器在地理坐标系下的三轴运动加速度、水平方向的GPS速度测量值
//以及垂直方向的速度测量值，这个由气压高度测量值微分得来（数据噪声很大）
void KalmanFilter(float accEf[3], float gpsVelBf[2], float baroVel)
{
        //用于存放计算结果的临时矩阵
        float m1[9], m2[9],  m3[9],  m4[9],  m5[9];
         
        //更新加速度矩阵
        accel[0] = accEf[0];
        accel[3] = accEf[1];        
        accel[6] = accEf[2];
        //更新速度观测矩阵
        Z[0] = gpsVelBf[0];
        Z[3] = gpsVelBf[1];
        Z[6] = baroVel;
         
        //1:状态预估计 Xk = Fk*Xk-1 + Bk*Uk
        Matrix3_Mul(F, velocity, m1);
        Matrix3_Mul(B, accel, m2);   
        Matrix3_Add(m1, m2, velocity);
        //2：误差协方差矩阵预更新 Pk = Fk*Pk-1*FkT + Qk
        Matrix3_Mul(F, P, m1);
        Matrix3_Mul(m1, F_t, m2);    
        Matrix3_Add(m2, Q, P);
         //3：计算误差矩阵 Yk = Zk - Hk*Xk
        Matrix3_Mul(H, velocity, m1);
        Matrix3_Sub(Z, m1, Y);
        //4：Sk = Hk*Pk*HkT + Rk
        Matrix3_Mul(H, P, m1);
        Matrix3_Mul(m1, H_t, m2);
        Matrix3_Add(m2, R, S);   
        //5：计算卡尔曼增益 Kk = Pk*HkT*Sk-1
        Matrix3_Det(S, m1);
        Matrix3_Mul(P, H_t, m2);
        Matrix3_Mul(m2, m1, K);
        //6：修正当前状态 Xk = Xk + Kk*Yk
        Matrix3_Mul(K, Y, m1);
        Matrix3_Add(velocity, m1, m2); 
        Matrix3_Copy(m2, velocity);
        //7：更新协方差矩阵 Pk = (I-Kk*Hk)*Pk*(I-Kk*Hk)T + Kk*Rk*KkT
        Matrix3_Mul(K, H, m1);
        Matrix3_Sub(I, m1, m2);
        Matrix3_Tran(m2, m3);
        Matrix3_Mul(m2, P, m4);
        Matrix3_Mul(m4, m3, m5);
        Matrix3_Mul(K, R, m1);
        Matrix3_Tran(K, m2);
        Matrix3_Mul(m1, m2, m3);
        Matrix3_Add(m5, m3, P);
}

