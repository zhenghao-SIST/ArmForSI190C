/*
 * File: DH_server.cpp
 * Author: Zhenghao Li
 * Email: lizhenghao@shanghaitech.edu.cn
 * Institute: SIST
 * Created: 2025-04-29
 * Last Modified: 2025-06-11
 */
#include "joint_kinematics_node/DH_server.hpp"
#include <iostream>

DHServer::DHServer(): theta_delta(6, 0.0)
{
}

void DHServer::set_DH_params(int x, const std::vector<double> &&params)
{
    this->joint[x] << params[0], params[1], params[2], params[3];
}
void DHServer::set_theta(int index, double theta)
{
    theta_delta[index] = theta;
}
void DHServer::get_transform(Eigen::Matrix4d &T){
    Eigen::Matrix4d temp;
    double a, alpha, d, theta;
    for(size_t i = 0; i < 6; ++i){
        a = this->joint[i][0];
        alpha= this->joint[i][1];
        d = this->joint[i][2];
        theta = this->joint[i][3] + this->theta_delta[i];
        temp << cos(theta), -sin(theta) * cos(alpha),  sin(theta) * sin(alpha), a * cos(theta),
                sin(theta),  cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
                0,                              sin(alpha),                  cos(alpha),                 d,
                0,                                          0,                              0,                 1;

        T = T * temp;
    }
}

void DHServer::get_transform(Eigen::Matrix4d &T, std::vector<double> &t){
    Eigen::Matrix4d temp;
    double a, alpha, d, theta;
    for(size_t i = 0; i < 6; ++i){
        a = this->joint[i][0];
        alpha= this->joint[i][1];
        d = this->joint[i][2];
        theta = this->joint[i][3] + t[i];
        temp << cos(theta), -sin(theta) * cos(alpha),  sin(theta) * sin(alpha), a * cos(theta),
                sin(theta),  cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
                0,                              sin(alpha),                  cos(alpha),                 d,
                0,                                          0,                              0,                 1;

        T = T * temp;
    }
}

void DHServer::get_A03(Eigen::Matrix4d &T, std::vector<double> &t)
{
    Eigen::Matrix4d temp;
    double a, alpha, d, theta;
    for(size_t i = 0; i < 3; ++i){
        a = this->joint[i][0];
        alpha= this->joint[i][1];
        d = this->joint[i][2];
        theta = this->joint[i][3] + t[i];
        //std::cout << "1:" << theta << std::endl;
        temp << cos(theta), -sin(theta) * cos(alpha),  sin(theta) * sin(alpha), a * cos(theta),
                sin(theta),  cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
                0,                              sin(alpha),                  cos(alpha),                 d,
                0,                                          0,                              0,                 1;

        T = T * temp;
    }
}

void DHServer::get_A03(Eigen::Matrix4d &T)
{
    Eigen::Matrix4d temp;
    double a, alpha, d, theta;
    for(size_t i = 0; i < 3; ++i){
        a = this->joint[i][0];
        alpha= this->joint[i][1];
        d = this->joint[i][2];
        theta = this->joint[i][3] + this->theta_delta[i];
        //std::cout << "2:" << theta << std::endl;
        temp << cos(theta), -sin(theta) * cos(alpha),  sin(theta) * sin(alpha), a * cos(theta),
                sin(theta),  cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
                0,                              sin(alpha),                  cos(alpha),                 d,
                0,                                          0,                              0,                 1;

        T = T * temp;
    }
}



double DHServer::get_a(int joint)
{
    return this->joint[joint][0];
}

double DHServer::get_alpha(int joint)
{
    return this->joint[joint][1];
}

double DHServer::get_d(int joint)
{
    return this->joint[joint][2];
}

double DHServer::get_theta(int joint)
{
    return this->joint[joint][3] + this->theta_delta[joint];
}
double DHServer::get_delta(int joint)
{
    return this->theta_delta[joint];
}
double DHServer::get_d0(int joint)
{
    return this->joint[joint][3];
}

