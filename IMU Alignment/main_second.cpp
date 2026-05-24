
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <cmath>
#include "struct_const.h"   // 需要包含 IMUDATA/IMU_mean/Attitude 以及 computeSegmentMean/IMU_Alignment/parseRAWIMUSA 的声明

int main()
{
    // ===== 1) 读取 IMU 文件 =====
    const std::string in_path = "D://惯性导航原理//数据//对准//Calibration_30min.ASC";         // ←改成你的路径（补偿前或补偿后都行）
    const std::string out_csv = "D://惯性导航原理//对准结果//align_1Hz.csv";     // 输出对准结果（每秒一行）

    std::vector<IMUDATA> imu = parseRAWIMUSA(in_path);
    if (imu.empty()) {
        std::cerr << "读取失败或文件为空: " << in_path << std::endl;
        return -1;
    }
    std::sort(imu.begin(), imu.end(), [](const IMUDATA& a, const IMUDATA& b) { return a.t < b.t; });

    // ===== 2) 打开输出 CSV =====
    std::ofstream fout(out_csv, std::ios::binary);
    if (!fout.is_open()) {
        std::cerr << "无法创建输出文件: " << out_csv << std::endl;
        return -1;
    }
    // 写 UTF-8 BOM（Excel 友好）
    const unsigned char bom[3] = { 0xEF,0xBB,0xBF };
    fout.write(reinterpret_cast<const char*>(bom), 3);
    fout.setf(std::ios::fixed);
    fout << std::setprecision(10);
    fout << "t(s),roll(rad),pitch(rad),yaw(rad),roll(deg),pitch(deg),yaw(deg)\n";

    // ===== 3) 每秒分桶 → 均值 → 对准 =====
    const double t_start = std::floor(imu.front().t);
    const double t_end = std::floor(imu.back().t);

    std::size_t idx = 0;

    for (long s = static_cast<long>(t_start); s <= static_cast<long>(t_end); ++s) {
        const double bin_start = static_cast<double>(s);
        const double bin_end = bin_start + 1.0;

        // 收集该 1 秒内的样本
        std::vector<IMUDATA> seg;
        for (; idx < imu.size(); ++idx) {
            const double t = imu[idx].t;
            if (t < bin_start) continue;
            if (t >= bin_end)  break;
            seg.push_back(imu[idx]);
        }
        if (seg.empty()) continue;  // 该秒没有样本则跳过

        // 每秒均值
        IMU_mean mean = computeSegmentMean(seg);

        // 对准
        Attitude att{};
        IMU_Alignment(mean, att);

        // 以该秒中心时刻记时间戳
        const double t_center = 0.5 * (mean.t0 + mean.t1);

        // 写出结果
        fout << t_center << ","
            << att.roll << "," << att.pitch << "," << att.yaw << ","
            << att.roll * Deg << "," << att.pitch * Deg << "," << att.yaw * Deg << "\n";
    }

    fout.close();
    std::cout << "✅ 已保存每秒对准结果: " << out_csv << std::endl;
    return 0;
}
