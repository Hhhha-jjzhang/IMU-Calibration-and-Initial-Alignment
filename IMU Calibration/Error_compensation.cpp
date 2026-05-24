#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>
#include<eigen>
#include <vector>
#include <cmath>
#include <iomanip> // 添加此头文件以解决 std::setprecision 未定义的问题
#include"Myheadfile.h"
using namespace std;



static inline Eigen::Matrix3d Acc_BuildM(const Acc_Error& C) {
    Eigen::Matrix3d M1;
    M1 << 1.0 + C.S_a[0], 0,0,
        0, 1.0 + C.S_a[1], 0,
        0, 0, 1.0 + C.S_a[2];
    /*Eigen::Matrix3d M2 = (C.M_a).transpose();*/
    Eigen::Matrix3d M2 = C.M_a;
	Eigen::Matrix3d M = M1 + M2;
    return M;
}

static inline Eigen::Matrix3d Gyro_BuildM(const Gyro_Error& C) {
	Eigen::Matrix3d M;
	M << 1.0 + C.S_g[0], 0, 0,
		0, 1.0 + C.S_g[1], 0,
		0, 0, 1.0 + C.S_g[2];
	return M;
}

//计算并保存补偿后的IMU数据到CSV文件
void CompensateIMU(
    const std::vector<IMUDATA>& in,
    const Acc_Error& accCal,
    const Gyro_Error& gyoCal,
    std::vector<IMUDATA>& out,
    const std::string& out_csv,     // 输出 CSV 文件路径   
    double dt_for_increment    // 若>0，同时更新 dv/dtheta
) 
{
    out.clear();
    out.reserve(in.size());

    // 预构建矩阵与偏置
    const Eigen::Matrix3d Ma = Acc_BuildM(accCal);
    const Eigen::Matrix3d Mg = Gyro_BuildM(gyoCal);

    // 推荐用线性方程求解代替显式求逆，数值更稳
    const Eigen::PartialPivLU<Eigen::Matrix3d> Ma_lu(Ma);
    const Eigen::PartialPivLU<Eigen::Matrix3d> Mg_lu(Mg);

    const Eigen::Vector3d ba(accCal.b_a[0], accCal.b_a[1], accCal.b_a[2]);
    const Eigen::Vector3d bg(gyoCal.b_g[0], gyoCal.b_g[1], gyoCal.b_g[2]);

    std::ofstream fout(out_csv);
    if (!fout.is_open()) {
        std::cerr << "无法打开输出文件: " << out_csv << std::endl;
        return;
    }

    fout.setf(std::ios::fixed);
    fout << std::setprecision(10);
    fout << "t(s),acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z\n";


    for (const auto& s : in) {
        IMUDATA d = s; // 拷贝一份做就地改写

        // 原始量：\tilde{m}
        Eigen::Vector3d acc_raw(s.acc[0], s.acc[1], s.acc[2]);
        Eigen::Vector3d gyr_raw(s.gyro[0], s.gyro[1], s.gyro[2]);

        // m_c = M^{-1} (m_raw - b)  —— 用求解器实现
        Eigen::Vector3d acc_corr = Ma_lu.solve(acc_raw - ba);
        Eigen::Vector3d gyr_corr = Mg_lu.solve(gyr_raw - bg);

        d.acc[0] = acc_corr.x(); d.acc[1] = acc_corr.y(); d.acc[2] = acc_corr.z();
        d.gyro[0] = gyr_corr.x(); d.gyro[1] = gyr_corr.y(); d.gyro[2] = gyr_corr.z();

        // 如需同时更新增量（若你的上游用增量参与对准/导航）
        if (dt_for_increment > 0.0) {
            d.dv[0] = d.acc[0] * dt_for_increment;
            d.dv[1] = d.acc[1] * dt_for_increment;
            d.dv[2] = d.acc[2] * dt_for_increment;

            d.dtheta[0] = d.gyro[0] * dt_for_increment;
            d.dtheta[1] = d.gyro[1] * dt_for_increment;
            d.dtheta[2] = d.gyro[2] * dt_for_increment;
        }

        out.push_back(d);

        // === 写入 CSV 行 ===
        fout << d.t << ","
            << d.acc[0] << "," << d.acc[1] << "," << d.acc[2] << ","
            << d.gyro[0] << "," << d.gyro[1] << "," << d.gyro[2] << "\n";

    }
    fout.close();
    std::cout << "补偿结果已保存为 CSV 文件: " << out_csv << std::endl;
}


void SaveIMUToCSV(const std::vector<IMUDATA>& imu, const std::string& csv_path)
{
    std::ofstream fout(csv_path, std::ios::binary);
    if (!fout.is_open()) {
        std::cerr << "无法打开输出文件: " << csv_path << std::endl;
        return;
    }

    // 写 BOM，防止 Excel 打开乱码
    const unsigned char bom[3] = { 0xEF, 0xBB, 0xBF };
    fout.write(reinterpret_cast<const char*>(bom), 3);

    fout.setf(std::ios::fixed);
    fout << std::setprecision(10);

    // 表头
    fout << "t(s),acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z\n";

    // 数据
    for (const auto& d : imu) {
        fout << d.t << ","
            << d.acc[0] << "," << d.acc[1] << "," << d.acc[2] << ","
            << d.gyro[0] << "," << d.gyro[1] << "," << d.gyro[2] << "\n";
    }

    fout.close();
    std::cout << "IMU数据已保存到: " << csv_path << std::endl;
}