#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include "struct_const.h"   // 需包含 IMUDATA / IMU_mean / Attitude 的定义，以及 parseRAWIMUSA / IMU_Alignment 的声明

int main()
{
    //  1) 读取 IMU 数据（按需改路径，原始或补偿后的都可）
    const std::string in_path = "D://惯性导航原理//数据//对准//Calibration_30min.ASC";
    const std::string out_csv = "D://惯性导航原理//对准结果//align_per_epoch.csv";

    std::vector<IMUDATA> imu = parseRAWIMUSA(in_path);
    if (imu.empty()) {
        std::cerr << " 读取失败或文件为空: " << in_path << std::endl;
        return -1;
    }

    // 保险：按时间排序（若已按时序读取可省）
    std::sort(imu.begin(), imu.end(), [](const IMUDATA& a, const IMUDATA& b) { return a.t < b.t; });

    // ===== 2) 打开输出 CSV =====
    std::ofstream fout(out_csv, std::ios::binary);
    if (!fout.is_open()) {
        std::cerr << " 无法创建输出文件: " << out_csv << std::endl;
        return -1;
    }
    // 写 UTF-8 BOM（Excel 友好）
    const unsigned char bom[3] = { 0xEF,0xBB,0xBF };
    fout.write(reinterpret_cast<const char*>(bom), 3);
    fout.setf(std::ios::fixed);
    fout << std::setprecision(10);

    // 表头：时间、弧度、角度
    fout << "t(s),roll(rad),pitch(rad),yaw(rad),roll(deg),pitch(deg),yaw(deg)\n";

    // ===== 3) 历元逐条解算对准并写出 =====
    Attitude att{};
    IMU_mean mean{};
    mean.f_mean.setZero();
    mean.w_mean.setZero();

    for (size_t i = 0; i < imu.size(); ++i) {
        const auto& d = imu[i];

        // 用“单历元”构造 IMU_mean
        mean.t0 = mean.t1 = d.t;
        mean.f_mean = Eigen::Vector3d(d.acc[0], d.acc[1], d.acc[2]);
        mean.w_mean = Eigen::Vector3d(d.gyro[0], d.gyro[1], d.gyro[2]);

        // 对准
        IMU_Alignment(mean, att);

        // 写一行
        fout << d.t << ","
            << att.roll << "," << att.pitch << "," << att.yaw << ","
            << att.roll * Deg << "," << att.pitch * Deg << "," << att.yaw * Deg << "\n";

        // 简单进度提示（可选）
        if ((i + 1) % 5000 == 0) {
            std::cout << "已处理 " << (i + 1) << " / " << imu.size() << " 条..." << std::endl;
            fout.flush();
        }
    }

    fout.close();
    std::cout << "每历元对准结果已保存至: " << out_csv << std::endl;
    return 0;
}