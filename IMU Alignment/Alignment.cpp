#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <string>
#include<eigen>
#include "struct_const.h"
using namespace std;

std::vector<IMUDATA> parseRAWIMUSA(const std::string& file) {
    std::ifstream fin(file);
    std::string line;
    std::vector<IMUDATA> data;
    if (!fin.is_open()) return data;
    const double scale_dv = 2.0e-8;   // m/s per count (Δv增量)
    const double scale_dtheta = 1.0e-9;   // rad per count (Δθ增量)
    const double dt = 0.005;              // 200 Hz

    while (std::getline(fin, line)) {
        if (line.empty() || line[0] != '%') continue;

        // 找到分号，分号后是 IMU 主体
        size_t semicolon = line.find(';');
        if (semicolon == std::string::npos) continue;

        std::string imuPart = line.substr(semicolon + 1);

        // 去掉尾部 *CRC，避免影响解析
        size_t star = imuPart.find('*');
        if (star != std::string::npos) imuPart.resize(star);

        std::stringstream ss(imuPart);
        std::string token;

        // RAWIMUSXA 主体字段顺序：
        //  1 IMU Info, 2 IMU Type, 3 GPS Week, 4 GPS Seconds, 5 IMU Status(hex),
        //  6 ΔvZ, 7 -(ΔvY), 8 ΔvX, 9 ΔθZ, 10 -(ΔθY), 11 ΔθX
        int imuInfo = 0, imuType = 0;
        double week = 0.0, tow = 0.0;
        unsigned int status = 0;
        long dvZ_cnt = 0, n_dvY_cnt = 0, dvX_cnt = 0;
        long dthZ_cnt = 0, n_dthY_cnt = 0, dthX_cnt = 0;

        // 1 IMU Info
        if (!std::getline(ss, token, ',')) continue;
        // 文档中常为十六进制显示，但这里样例是两位十进制/十六进制均可；你若确定为16进制可改成 base=16
        imuInfo = std::stoi(token, nullptr, 10);

        // 2 IMU Type
        if (!std::getline(ss, token, ',')) continue;
        imuType = std::stoi(token, nullptr, 10);

        // 3 GPS Week
        if (!std::getline(ss, token, ',')) continue;
        week = std::stod(token);

        // 4 GPS Seconds of Week
        if (!std::getline(ss, token, ',')) continue;
        tow = std::stod(token);

        // 5 IMU Status (hex)
        if (!std::getline(ss, token, ',')) continue;
        status = std::stoul(token, nullptr, 16);

        // 6 ΔvZ  (counts)
        if (!std::getline(ss, token, ',')) continue;
        dvZ_cnt = std::stol(token);

        // 7 ΔvY  (counts) 
        if (!std::getline(ss, token, ',')) continue;
        n_dvY_cnt = std::stol(token);

        // 8 ΔvX  (counts)
        if (!std::getline(ss, token, ',')) continue;
        dvX_cnt = std::stol(token);

        // 9 ΔθZ  (counts)
        if (!std::getline(ss, token, ',')) continue;
        dthZ_cnt = std::stol(token);

        // 10 ΔθY  (counts)
        if (!std::getline(ss, token, ',')) continue;
        n_dthY_cnt = std::stol(token);

        // 11 ΔθX  (counts)
        if (!std::getline(ss, token, ',')) continue;
        dthX_cnt = std::stol(token);

        IMUDATA f{};
        f.t = tow;                // 如需：可组合 week + tow 成 GPST

        const double dvx = dvX_cnt * scale_dv;          // +X
        const double dvy = -(n_dvY_cnt)*scale_dv;     // Y
        const double dvz = dvZ_cnt * scale_dv;          // -Z

        const double dtx = dthX_cnt * scale_dtheta;     // +X
        const double dty = -(n_dthY_cnt)*scale_dtheta;// +Y
        const double dtz = dthZ_cnt * scale_dtheta;     // -Z


        //const double dvy = dvX_cnt * scale_dv;          // +X
        //const double dvx = (n_dvY_cnt) * scale_dv;     // Y
        //const double dvz = -dvZ_cnt * scale_dv;          // -Z

        //const double dty = dthX_cnt * scale_dtheta;     // +X
        //const double dtx = (n_dthY_cnt) * scale_dtheta;// +Y
        //const double dtz = -dthZ_cnt * scale_dtheta;     // -Z

        f.dv[0] = dvx; f.dv[1] = dvy; f.dv[2] = dvz;
        f.dtheta[0] = dtx; f.dtheta[1] = dty; f.dtheta[2] = dtz;

        f.acc[0] = dvx / dt;
        f.acc[1] = dvy / dt;
        f.acc[2] = dvz / dt;

        f.gyro[0] = dtx / dt;
        f.gyro[1] = dty / dt;
        f.gyro[2] = dtz / dt;

        data.push_back(f);
    }
    return data;
}

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

/*取北-东-地作为导航坐标系坐标系（NED），对准时IMU加速计感知的是g在b系下的分量，陀螺仪感知的是自转角速度
omgae_ie在b系的投影
*/

void IMU_Alignment(const IMU_mean& imu_mean, Attitude& attitude)
{
	// 提取均值数据
	const Eigen::Vector3d& f_b = imu_mean.f_mean; // 加速度计均值
	const Eigen::Vector3d& w_b = imu_mean.w_mean; // 陀螺仪均值
	//计算姿态矩阵
    // 定义向量
	Eigen::Vector3d g_n(0, 0, G);
    Eigen::Vector3d V_g = g_n / G;// 重力加速度在导航系下的表示

	Eigen::Vector3d omega_ie_n(omgae_ie * cos(Fai), 0, -omgae_ie * sin(Fai)); // 地球自转角速度在导航系下的表示 
	Eigen::Vector3d V_omgea = g_n.cross(omega_ie_n) / (g_n.cross(omega_ie_n)).norm(); 

	Eigen::Vector3d V_g_omgea = (g_n.cross(omega_ie_n)).cross(g_n) / ((g_n.cross(omega_ie_n)).cross(g_n)).norm();

    Eigen::Vector3d g_b(-f_b(0), -f_b(1), -f_b(2));
    Eigen::Vector3d W_g = g_b / g_b.norm();

    Eigen::Vector3d omega_ie_b(w_b(0), w_b(1), w_b(2)); // 地球自转角速度在导航系下的表示 
	Eigen::Vector3d W_omgea = g_b.cross(omega_ie_b) / (g_b.cross(omega_ie_b)).norm();
	Eigen::Vector3d W_g_omgea = (g_b.cross(omega_ie_b)).cross(g_b) / ((g_b.cross(omega_ie_b)).cross(g_b)).norm();

	// 构造姿态矩阵
	Eigen::Matrix3d C_bn;
    Eigen::Matrix3d V;
	V.col(0) = V_g;
	V.col(1) = V_omgea;
	V.col(2) = V_g_omgea;
	Eigen::Matrix3d W;
	W.col(0) = W_g;
	W.col(1) = W_omgea;
	W.col(2) = W_g_omgea;
	C_bn = V * W.transpose();
    
	/*cout << "C_bn:" << endl << C_bn << endl;
    cout << endl;*/
	 //提取姿态角
	attitude.roll = atan2(C_bn(2, 1), C_bn(2, 2)) ;
    attitude.pitch = atan(-C_bn(2, 0)/sqrt(C_bn(2, 1) * C_bn(2, 1) + C_bn(2, 2) * C_bn(2, 2)));
	attitude.yaw = atan2(C_bn(1, 0), C_bn(0, 0));
    /*cout << "roll(rad): " << attitude.roll  << endl;
    cout << "pitch(rad): " << attitude.pitch << endl;
    cout << "yaw(rad): " << attitude.yaw  << endl;
    cout << endl;
	cout << "roll(deg): " << attitude.roll * Deg << endl;
	cout << "pitch(deg): " << attitude.pitch * Deg << endl;
	cout << "yaw(deg): " << attitude.yaw * Deg << endl;*/

}

