/*
 * File: DH_server.hpp
 * Author: Zhenghao Li
 * Email: lizhenghao@shanghaitech.edu.cn
 * Institute: SIST
 * Created: 2025-04-29
 * Last Modified: 2025-06-11
 */

#include <Eigen/Core>

#ifndef _DH_SERVER_
#define _DH_SERVER_

class DHServer{
    Eigen::Vector4d joint[6];
    std::vector<double> theta_delta;
public:
    DHServer();
    double get_d(int joint);
    double get_a(int joint);
    double get_alpha(int joint);
    double get_theta(int joint);
    double get_delta(int joint);
    double get_d0(int joint);

    void set_DH_params(int x, const std::vector<double> &&params);
    void set_theta(int index, double theta);
    void get_transform(Eigen::Matrix4d &T);
    void get_transform(Eigen::Matrix4d &T, std::vector<double> &t);
    void get_A03(Eigen::Matrix4d &T, std::vector<double> &t);
    void get_A03(Eigen::Matrix4d &T);
};

#endif
