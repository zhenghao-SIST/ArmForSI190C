/*
 * File: utility.hpp
 * Author: Zhenghao Li
 * Email: lizhenghao@shanghaitech.edu.cn
 * Institute: SIST
 * Created: 2025-05-08
 * Last Modified: 2025-09-25
 */

#include <cmath>
#include <vector>
#include <stdexcept>

#ifndef _UTILITY_
#define _UTILITY_


struct axisRange{
    double left = 0;
    double right = M_PI * 2;
    void set(double a, double b);
};

bool inRange(double angle, double left, double right);

bool inRange(std::vector<double> &solutions, std::vector<axisRange> &range);

bool isZero(double value);

//double getAngleInRange(double angle1, double angle2);

double distance(double target, double angle);

double normalizeAngle(double angle);

double selectBest(std::vector<double> &angle, std::vector<double> &range, double current);
#endif
