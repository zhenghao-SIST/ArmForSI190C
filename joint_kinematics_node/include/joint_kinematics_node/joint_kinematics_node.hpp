/*
 * File: joint_kinematics_node.hpp
 * Author: Zhenghao Li
 * Email: lizhenghao@shanghaitech.edu.cn
 * Institute: SIST
 * Created: 2025-04-29
 * Last Modified: 2025-09-12
 */

#ifndef JOINT_KINEMATICS_NODE_HPP_
#define JOINT_KINEMATICS_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sensor_msgs/msg/joint_state.hpp"
#include "joint_kinematics_node/DH_server.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "utility/utility.hpp"

class JointKinematicsNode: public rclcpp::Node
{
public:
  JointKinematicsNode();

private:
  void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void forwardKinematics();
  void inverseKinematics(Eigen::Vector3d &v, Eigen::Quaterniond &q);
  void timerCallback();
  std::vector<axisRange> range_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  DHServer dh_server;
};

#endif  // JOINT_KINEMATICS_NODE_HPP_

