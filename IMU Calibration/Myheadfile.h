#pragma once
using namespace std;
#include<eigen>


#define PAI 3.1415926535898
#define Rad (PAI/180.0)                   // 角度转弧度的系数
#define Deg (180.0/PAI)                   // 弧度转角度的系数
#define G 9.7936174  // 当地重力加速度 m/s²
#define omgae_ie 7.292115E-5  // 地球自转角速度 rad/s
#define Fai 30.531651244*PAI/180.0  // 当地纬度的弧度值
//定义IMU数据结构体
struct IMUDATA {
    double t;          // 秒
    double dv[3];      // 加速度增量 (m/s)
    double dtheta[3];  // 角增量 (度)

	double acc[3];     // 比力 (m/s²)
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




//加速计误差项
struct Acc_Error
{
	Eigen::Vector3d S_a;      //比例因子误差
	Eigen::Vector3d b_a;      //零偏误差
	Eigen::Matrix3d M_a;      //非正交误差
	Acc_Error()
	{
		S_a.setZero();
		b_a.setZero();
		M_a.setZero();
	}
};

//陀螺误差项
struct Gyro_Error
{
	Eigen::Vector3d S_g;      //比例因子误差
	Eigen::Vector3d b_g;      //零偏误差
	Gyro_Error()
	{
		S_g.setZero();
		b_g.setZero();
	}
};


//数据提取函数
std::vector<IMUDATA> parseRAWIMUSA(const std::string& file);
//数据分段函数
std::vector<IMUDATA> sliceIMUDataByTime(const std::vector<IMUDATA>& v, double t0, double t1);
//求均值函数
IMU_mean computeSegmentMean(const std::vector<IMUDATA>& seg);
//加速度计误差求解
void acc_Calibration(Calibra_data* calib, Acc_Error* acc);
//角增量积分
Eigen::Vector3d integration(const std::vector<IMUDATA>& seg);
//陀螺误差求解
void gyro_Calibration(integration_data* inte_data, Gyro_Error* Gyro);

static inline Eigen::Matrix3d Acc_BuildM(const Acc_Error& C);
static inline Eigen::Matrix3d Gyro_BuildM(const Gyro_Error& C);
void CompensateIMU(
	const std::vector<IMUDATA>& in,
	const Acc_Error& accCal,
	const Gyro_Error& gyoCal,
	std::vector<IMUDATA>& out,
	const std::string& out_csv,     // 输出 CSV 文件路径   
	double dt_for_increment = 0.0   // 若>0，同时更新 dv/dtheta
);

void SaveIMUToCSV(const std::vector<IMUDATA>& imu, const std::string& csv_path);