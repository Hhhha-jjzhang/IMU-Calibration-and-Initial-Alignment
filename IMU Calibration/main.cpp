#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>
#include <string>
#include<eigen>

#include"Myheadfile.h"
using namespace std;

int main() {
    //提取静置数据
    auto imu_data_x_down = parseRAWIMUSA("D://惯性导航原理//数据//标定//ASC//x_down_3min.ASC");
    auto imu_data_x_up = parseRAWIMUSA("D://惯性导航原理//数据//标定//ASC//x_up_3min.ASC");
    auto imu_data_y_down = parseRAWIMUSA("D://惯性导航原理//数据//标定//ASC//y_down_3min.ASC");
    auto imu_data_y_up = parseRAWIMUSA("D://惯性导航原理//数据//标定//ASC//y_up_3min.ASC");
    auto imu_data_z_down = parseRAWIMUSA("D://惯性导航原理//数据//标定//ASC//z_down_3min.ASC");
    auto imu_data_z_up = parseRAWIMUSA("D://惯性导航原理//数据//标定//ASC//z_up_3min.ASC");
    //提取正负旋转数据（p:+360，n:-360）
    auto imu_data_x_p= parseRAWIMUSA("D://惯性导航原理//数据//标定//ASC//x+360.ASC");
    auto imu_data_x_n = parseRAWIMUSA("D://惯性导航原理//数据//标定//ASC//x-360.ASC");
    auto imu_data_y_p = parseRAWIMUSA("D://惯性导航原理//数据//标定//ASC//y+360.ASC");
    auto imu_data_y_n = parseRAWIMUSA("D://惯性导航原理//数据//标定//ASC//y-360.ASC");
    auto imu_data_z_p = parseRAWIMUSA("D://惯性导航原理//数据//标定//ASC//z+360.ASC");
    auto imu_data_z_n = parseRAWIMUSA("D://惯性导航原理//数据//标定//ASC//z-360.ASC");

 
    std::vector<IMUDATA> x_down_sliced = sliceIMUDataByTime(imu_data_x_down, 558510.000023745, 558690.000023913);
    std::vector<IMUDATA> x_up_sliced = sliceIMUDataByTime(imu_data_x_up, 558135.000023396, 558315.000023563);
    std::vector<IMUDATA> y_down_sliced = sliceIMUDataByTime(imu_data_y_down, 557811.000023094, 557991.000023262);
    std::vector<IMUDATA> y_up_sliced = sliceIMUDataByTime(imu_data_y_up, 557453.000022761, 557633.000022928);
    std::vector<IMUDATA> z_down_sliced = sliceIMUDataByTime(imu_data_z_down, 559086.000024281, 559266.000024449);
    std::vector<IMUDATA> z_up_sliced = sliceIMUDataByTime(imu_data_z_up, 558766.000023983, 558946.000024151);

    //x轴截取40秒的数据
    std::vector<IMUDATA> x_p_sliced = sliceIMUDataByTime(imu_data_x_p, 558329.000023576, 558369.000023614);
    std::vector<IMUDATA> x_n_sliced = sliceIMUDataByTime(imu_data_x_n, 558383.000023627, 558423.000023664);
    //y轴截取45秒数据
    std::vector<IMUDATA> y_p_sliced = sliceIMUDataByTime(imu_data_y_p, 557661.000022954, 557706.000022996);
    std::vector<IMUDATA> y_n_sliced = sliceIMUDataByTime(imu_data_y_n, 557721.000023010, 557766.000023052);
	//z轴截取42秒数据
    std::vector<IMUDATA> z_p_sliced = sliceIMUDataByTime(imu_data_z_p, 558958.000024162, 559000.000024201);
    std::vector<IMUDATA> z_n_sliced = sliceIMUDataByTime(imu_data_z_n, 559012.000024213, 559054.000024252);///

	Calibra_data acc_calib_data;
    acc_calib_data.x_down = computeSegmentMean(x_down_sliced);
    acc_calib_data.x_up = computeSegmentMean(x_up_sliced);
    acc_calib_data.y_down = computeSegmentMean(y_down_sliced);
    acc_calib_data.y_up = computeSegmentMean(y_up_sliced);
    acc_calib_data.z_down = computeSegmentMean(z_down_sliced);
    acc_calib_data.z_up = computeSegmentMean(z_up_sliced);
	
	Acc_Error acc;
    acc_Calibration(&acc_calib_data, &acc);
    cout << "加速度计比例因子误差S_a:\n" << acc.S_a.transpose() << endl;
    cout << "加速度计零偏误差b_a:\n" << acc.b_a.transpose() << endl;
    cout << "加速度计非正交误差M_a:\n" << acc.M_a << endl;


	integration_data gyro_inte_data;
	gyro_inte_data.x_p = integration(x_p_sliced);
	gyro_inte_data.x_n = integration(x_n_sliced);
	gyro_inte_data.y_p = integration(y_p_sliced);
	gyro_inte_data.y_n = integration(y_n_sliced);
	gyro_inte_data.z_p = integration(z_p_sliced);
	gyro_inte_data.z_n = integration(z_n_sliced);

	Gyro_Error gyro;
	gyro_Calibration(&gyro_inte_data, &gyro);

	cout << "陀螺比例因子误差S_g:\n" << gyro.S_g << endl;
	cout << "陀螺零偏误差b_g:\n" << gyro.b_g << endl;


    //补偿误差部分：
    //x_down_sliced
	std::vector<IMUDATA> z_n_compensated;
    SaveIMUToCSV(z_n_sliced, "D://惯性导航原理//标定结果//补偿前//imu_z_n.csv");
    CompensateIMU(
        z_n_sliced,
        acc,
        gyro,
        z_n_compensated,
        "D://惯性导航原理//标定结果//补偿后//imu_compensated_z_n.csv",
        100     
    );


	return 0;
       



}