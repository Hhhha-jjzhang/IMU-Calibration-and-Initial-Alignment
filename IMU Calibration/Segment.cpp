#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <string>
#include<eigen>

#include"Myheadfile.h"
using namespace std;

//数据分段函数

std::vector<IMUDATA> sliceIMUDataByTime(const std::vector<IMUDATA>& v, double t0, double t1)
{
    std::vector<IMUDATA> out;
    if (v.empty() || t1 <= t0) {
        std::cerr << "[WARN] Invalid input range or empty data.\n";
        return out;
    }

    // 通过二分查找找到 [t0, t1) 区间的起始与结束位置
    auto lb = std::lower_bound(v.begin(), v.end(), t0,
        [](const IMUDATA& a, double t) { return a.t < t; });
    auto ub = std::lower_bound(v.begin(), v.end(), t1,
        [](const IMUDATA& a, double t) { return a.t < t; });

    //  计算索引范围
    size_t i0 = size_t(lb - v.begin());
    size_t i1 = size_t(ub - v.begin());

    if (i0 >= i1) {
        std::cerr << "[WARN] No data in range [" << t0 << ", " << t1 << ")\n";
        return out;
    }

    //复制区间数据
    out.reserve(i1 - i0);
    for (size_t i = i0; i < i1; ++i) {
        out.push_back(v[i]);
    }

    std::cout << "[INFO] Sliced data: " << (i1 - i0)
        << " samples, time [" << v[i0].t << ", " << v[i1 - 1].t << "]\n";

    return out;
}
