#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <string>
#include<eigen>

#include"Myheadfile.h"
using namespace std;

std::vector<IMUDATA> parseRAWIMUSA(const std::string& file) {
    std::ifstream fin(file);
    std::string line;
    std::vector<IMUDATA> data;

    const double scale_dv = 1.5258789063E-06;  // m/s² * s
    const double scale_dtheta =1.0850694444E-07;        // rad
    const double dt = 0.01; // 100 Hz

    while (std::getline(fin, line)) {
        if (line.empty() || line[0] != '%') continue;

        size_t semicolon = line.find(';');
        if (semicolon == std::string::npos) continue;

        // 拆解IMU部分
        std::string imuPart = line.substr(semicolon + 1);
        std::replace(imuPart.begin(), imuPart.end(), '*', ','); // 去掉校验
        std::stringstream ss(imuPart);
        std::string token;

        IMUDATA f{};
        double week, tow;
        unsigned int status;
        double ax, ay, az, gx, gy, gz;

        // imuWk, imuTow, imuStatus, ΔvX, ΔvY, ΔvZ, ΔθX, ΔθY, ΔθZ
        std::getline(ss, token, ','); week = std::stod(token);
        std::getline(ss, token, ','); tow = std::stod(token);
        std::getline(ss, token, ','); status = std::stoul(token, nullptr, 16);
        std::getline(ss, token, ','); az = std::stod(token);
        std::getline(ss, token, ','); ay = std::stod(token);
        std::getline(ss, token, ','); ax = std::stod(token);
        std::getline(ss, token, ','); gz = std::stod(token);
        std::getline(ss, token, ','); gy = std::stod(token);
        std::getline(ss, token, ','); gx = std::stod(token);

        f.t = tow;
        f.dv[0] = ax * scale_dv;
        f.dv[1] = ay * scale_dv;
        f.dv[2] = az * scale_dv;
        f.dtheta[0] = gx * scale_dtheta;
        f.dtheta[1] = gy * scale_dtheta;
        f.dtheta[2] = gz * scale_dtheta;

        f.acc[0] = f.dv[0] / dt;
        f.acc[1] = -f.dv[1] / dt; 
        f.acc[2] = f.dv[2] / dt;

        f.gyro[0] = f.dtheta[0]/dt;
        f.gyro[1] = -f.dtheta[1]/dt;
        f.gyro[2] = f.dtheta[2]/dt;
        data.push_back(f);
    }

    return data;
}


