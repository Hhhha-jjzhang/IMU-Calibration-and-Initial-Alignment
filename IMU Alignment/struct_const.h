#pragma once
using namespace std;
#include<eigen>


#define PAI 3.1415926535898
#define Rad (PAI/180.0)                   // 角度转弧度的系数
#define Deg (180.0/PAI)                   // 弧度转角度的系数
#define G 9.7936174  // 当地重力加速度 m/s?
#define omgae_ie 7.292115E-5  // 地球自转角速度 rad/s
#define Fai 30.531651244*PAI/180.0  // 当地纬度的弧度值
#define h 28.2134// 椭球高
//定义IMU数据结构体
struct IMUDATA {
	double t;          // 秒
	double dv[3];      // 加速度增量 (m/s)
	double dtheta[3];  // 角增量 (度)

	double acc[3];     // 比力 (m/s?)
	double gyro[3];    // 角速度 (度/s)
};

//数据的起点与终点时间戳
struct Segment { size_t i0, i1; };
//均值数据统计量
struct IMU_mean
{
	double t0, t1;//取数据段的时间戳
	Eigen::Vector3d f_mean;    // m/s^2
	Eigen::Vector3d w_mean;    // rad/s
};

//标定六方向均值数据
struct Calibra_data
{
	IMU_mean x_down;
	IMU_mean x_up;
	IMU_mean y_down;
	IMU_mean y_up;
	IMU_mean z_down;
	IMU_mean z_up;
};


//标定六方向均值数据
struct integration_data
{
	Eigen::Vector3d x_p;
	Eigen::Vector3d x_n;
	Eigen::Vector3d y_p;
	Eigen::Vector3d y_n;
	Eigen::Vector3d z_p;
	Eigen::Vector3d z_n;
};

//姿态角
struct Attitude
{
	double roll;// 横滚角
	double pitch;// 俯仰角
	double yaw;// 航向角
};



//数据读取函数
std::vector<IMUDATA> parseRAWIMUSA(const std::string& file);
IMU_mean computeSegmentMean(const std::vector<IMUDATA>& seg);
void IMU_Alignment(const IMU_mean& imu_mean, Attitude& attitude);