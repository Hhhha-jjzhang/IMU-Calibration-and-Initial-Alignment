#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <string>
#include<eigen>
#include "struct_const.h"
using namespace std;


int main()
{  //整段数据求平均值进行对准
	auto imu_data= parseRAWIMUSA("D://惯性导航原理//数据//对准//Calibration_30min.ASC");
	IMU_mean mean_data = computeSegmentMean(imu_data);
	Attitude attitude;
	IMU_Alignment(mean_data, attitude);

	//每秒求平均值进行对准

}