#include <vector>
#include <cmath>
#include <string>
#include<eigen>

#include"Myheadfile.h"
using namespace std;

//求均值函数
IMU_mean computeSegmentMean(const std::vector<IMUDATA>& seg)
{
    IMU_mean mean{};
    mean.f_mean.setZero();
    mean.w_mean.setZero();


    // 起止时间
    mean.t0 = seg.front().t;
    mean.t1 = seg.back().t;

    // 累加 acc 与 gyro
    for (const auto& d : seg) {
        mean.f_mean += Eigen::Vector3d(d.acc[0], d.acc[1], d.acc[2]);
        mean.w_mean += Eigen::Vector3d(d.gyro[0], d.gyro[1], d.gyro[2]);
    }

    // 求平均
    mean.f_mean /= seg.size();
    mean.w_mean /= seg.size();

    return mean;
}


//加速度计误差求解
void acc_Calibration(Calibra_data* calib, Acc_Error* acc)
{
    Eigen::MatrixXd L(3, 6);
    Eigen::MatrixXd A(4, 6);
    Eigen::MatrixXd M(3, 4);

    
    L << calib->x_up.f_mean, calib->x_down.f_mean, calib->y_up.f_mean, calib->y_down.f_mean, calib->z_up.f_mean, calib->z_down.f_mean ;  // 每个 vi 是 3x1，会按列依次拼接
    A << G, -G, 0, 0, 0, 0,
        0, 0, G, -G, 0, 0,
        0, 0, 0, 0, G, -G,
        1, 1, 1, 1, 1, 1;
	M = L * A.transpose() * (A * A.transpose()).inverse();
	acc->S_a << M(0,0)-1, M(1, 1) - 1, M(2, 2) - 1;
	acc->b_a = M.col(3);
    M.block<3, 3>(0, 0).diagonal().setZero();
    acc->M_a = M.block<3, 3>(0, 0);

}


//角增量积分
Eigen::Vector3d integration(const std::vector<IMUDATA>& seg)
{
   Eigen::Vector3d integral{ 0,0,0 };
   // 累加 acc 与 gyro
   for (const auto& d : seg) {
       integral += Eigen::Vector3d(d.dtheta[0], d.dtheta[1], d.dtheta[2]);
   }
   return integral;
}



//陀螺误差求解
void gyro_Calibration(integration_data* inte_data, Gyro_Error* Gyro)
{

	Gyro->b_g[0] = (inte_data->x_p[0] + inte_data->x_n[0]) / (2 * 40)- omgae_ie*sin(Fai);
	Gyro->b_g[1] = (inte_data->y_p[1] + inte_data->y_n[1]) / (2 * 45) - omgae_ie * sin(Fai);
	Gyro->b_g[2] = (inte_data->z_p[2] + inte_data->z_n[2]) / (2 * 42)- omgae_ie * sin(Fai);

    Gyro->S_g[0] = abs(inte_data->x_p[0] - inte_data->x_n[0]) / (4 * PAI) - 1;
	Gyro->S_g[1] = abs(inte_data->y_p[1] - inte_data->y_n[1]) / (4 * PAI) - 1;
	Gyro->S_g[2] = abs(inte_data->z_p[2] - inte_data->z_n[2]) / (4 * PAI) - 1;
}