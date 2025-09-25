/*
 * File: joint_kinematics_node.cpp
 * Author: Zhenghao Li
 * Email: lizhenghao@shanghaitech.edu.cn
 * Institute: SIST
 * Created: 2025-04-29
 * Last Modified: 2025-09-25
 */

#include "joint_kinematics_node/joint_kinematics_node.hpp"
#include <cmath>

JointKinematicsNode::JointKinematicsNode()
: Node("kinematics_node"), range_(6)
{
    this->declare_parameter("joint_1", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("joint_2", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("joint_3", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("joint_4", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("joint_5", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("joint_6", rclcpp::PARAMETER_DOUBLE_ARRAY);

    dh_server.set_DH_params(0, std::move(this->get_parameter("joint_1").as_double_array()));
    dh_server.set_DH_params(1, std::move(this->get_parameter("joint_2").as_double_array()));
    dh_server.set_DH_params(2, std::move(this->get_parameter("joint_3").as_double_array()));
    dh_server.set_DH_params(3, std::move(this->get_parameter("joint_4").as_double_array()));
    dh_server.set_DH_params(4, std::move(this->get_parameter("joint_5").as_double_array()));
    dh_server.set_DH_params(5, std::move(this->get_parameter("joint_6").as_double_array()));

    range_[0].set(M_PI / 2, 2 * M_PI);
    range_[1].set(0, M_PI);
    range_[2].set(M_PI, 2 * M_PI);
    range_[3].set(M_PI / 2, 2 * M_PI);
    range_[4].set(0,  M_PI);

    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&JointKinematicsNode::jointStateCallback, this, std::placeholders::_1)
    );

    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/gui_pose", 10,
      std::bind(&JointKinematicsNode::poseCallback, this, std::placeholders::_1)
    );

    pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    //timer_ = this->create_wall_timer(
    //        std::chrono::milliseconds(10),  // 每 10ms触发一次
    //        std::bind(&JointKinematicsNode::timerCallback, this));
}

void JointKinematicsNode::timerCallback()
{
    sensor_msgs::msg::JointState msg;
    msg.position =  {0,0,0,0,0,0};
    msg.velocity =  {0,0,0,0,0,0};
    msg.effort   =  {0,0,0,0,0,0};
    msg.header.stamp = rclcpp::Clock().now();
    msg.name = {
    "base_to_link0",
    "link0_to_link1",
    "link1_to_link2",
    "link2_to_link3",
    "link3_to_link4",
    "link4_to_link5"};

    pub_->publish(msg);

}

void JointKinematicsNode::poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    Eigen::Vector3d t;
    t << msg->position.x , msg->position.y, msg->position.z;
    Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    inverseKinematics(t, q);
}

void JointKinematicsNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    //RCLCPP_INFO(this->get_logger(), "Received JointState:");
    for (size_t i = 0; i < msg->name.size(); ++i) {
      /*
      RCLCPP_INFO(this->get_logger(), "Joint %s: position=%.2f velocity=%.2f effort=%.2f",
                  msg->name[i].c_str(),
                  msg->position.size() > i ? msg->position[i] : 0.0,
                  msg->velocity.size() > i ? msg->velocity[i] : 0.0,
                  msg->effort.size() > i ? msg->effort[i] : 0.0);
                  */
      dh_server.set_theta(i, msg->position[i]);
    }
    forwardKinematics();
}
void JointKinematicsNode::forwardKinematics()
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    Eigen::Vector4d zero;
    zero << 0, 0, 0, 1;
    dh_server.get_transform(T);
    Eigen::Vector4d end = T * zero;
    RCLCPP_INFO(this->get_logger(), "FK x: %.3f, y: %.3f, z: %.3f", end(0), end(1), end(2));

    Eigen::Matrix3d R = T.topLeftCorner(3, 3);
    Eigen::Quaterniond q(R);
    Eigen::Vector3d t = T.topRightCorner(3, 1);
    RCLCPP_INFO(this->get_logger(), "q x: %.3f, y: %.3f, z: %.3f w: %.3f", q.x(), q.y(), q.z(), q.w());

    inverseKinematics(t, q);
}

void JointKinematicsNode::inverseKinematics(Eigen::Vector3d &v, Eigen::Quaterniond &q)
{
    try{
        Eigen::Vector3d arm6;
        RCLCPP_INFO(this->get_logger(), "Solve IK");

        //Step 0
        arm6 << 0, 0, dh_server.get_d(5);
        Eigen::Vector3d j5 = v - q * arm6;
        RCLCPP_INFO(this->get_logger(), "Joint5 x: %.3f, y: %.3f, z: %.3f", j5.x(), j5.y(), j5.z());

        //Step 1
        double a2 = dh_server.get_a(1);
        double a3 = dh_server.get_a(2);
        double d4 = dh_server.get_d(3);
        double c = (j5.squaredNorm() - a2 * a2 - a3 * a3 - d4 * d4) / (2 * a2);
        double R = std::sqrt(a3 * a3 + d4 * d4);
        double c3 = c / R;
        double s3 = std::sqrt(1 -  c3 * c3);

        double beta = std::atan2(d4, a3);
        std::vector<double> j3{normalizeAngle(std::atan2( s3, c3) - beta - dh_server.get_d0(2)),
                               normalizeAngle(std::atan2(-s3, c3) - beta - dh_server.get_d0(2))};

        //for(auto t : j3){
        //    if(inRange(t, range_[2].left, range_[2].right))
        //        j3_candidate.push_back(t);
        //}

        //Step 2
        double theta1 = std::atan2(j5.y(), j5.x()) - dh_server.get_d0(0);
        std::vector<double> j1_candidate{theta1, theta1 + M_PI};

        //Step 3
        std::vector<std::vector<double>> arm_candidate;
        arm_candidate.reserve(4);
        // 2 * 2 =  four solutions
        for(size_t i = 0 ; i < 2; ++i){
            double theta3 = j3[i];
            double theta = theta3 + dh_server.get_d0(2);
            double v1 = j5.topRows(2).norm();
            double coff0 = a2 + a3 * cos(theta) - d4 * sin(theta),
                   coff1 =      a3 * sin(theta) + d4 * cos(theta);

            double ans0 = std::atan2(coff0 * j5[2] - coff1 * v1,
                                     coff1 * j5[2] + coff0 * v1);

            double ans1 = std::atan2(coff0 * j5[2] + coff1 * v1,
                                     coff1 * j5[2] - coff0 * v1);

            arm_candidate.push_back(std::vector<double>{j1_candidate[0], ans0, theta3});
            arm_candidate.push_back(std::vector<double>{j1_candidate[1], ans1, theta3});
        }

        std::vector<std::vector<double>> candidate;
        std::vector<int> flag(8, 0);
        candidate.reserve(8);
        //Get joint angles of the spherical wrist
        for(auto theta : arm_candidate){
            Eigen::Matrix4d A03, A;
            A03 = Eigen::Matrix4d::Identity();
            A = Eigen::Matrix4d::Identity();
            dh_server.get_A03(A03, theta);
            A.topLeftCorner(3, 3) = q.toRotationMatrix();
            A.topRightCorner(3, 1) = v;
            Eigen::Matrix4d A36 = A03.inverse() * A;

            //Step 4 theta4
            std::vector<double> tmp0(theta);
            std::vector<double> tmp1(theta);
            tmp0.push_back(std::atan2(A36(1, 2), A36(0, 2)));
            tmp1.push_back(std::atan2(-A36(1, 2), -A36(0, 2)));

            //Step theta5
            double temp2 = std::sqrt(A36(1, 2) * A36(1, 2) + A36(0, 2) * A36(0, 2));
            tmp0.push_back(std::atan2(-temp2, A36(2, 2)) -dh_server.get_d0(4));
            tmp1.push_back(std::atan2(temp2, A36(2, 2)) -dh_server.get_d0(4));

            //Step theta6
            tmp0.push_back(std::atan2(A36(2, 1), -A36(2, 0)));
            tmp1.push_back(std::atan2(-A36(2, 1), A36(2, 0)));

            candidate.push_back(tmp0);
            candidate.push_back(tmp1);
        }
        for(size_t i = 0; i < candidate.size(); ++i){
            if(inRange(candidate[i], range_)) flag[i] = true;
            //RCLCPP_INFO(this->get_logger(), "candidate %lu %.4f %.4f %.4f %.4f %.4f %.4f, %d",
            //        i, candidate[i][0], candidate[i][1], candidate[i][2],
            //        candidate[i][3], candidate[i][4], candidate[i][5], flag[i]);
        }


        RCLCPP_INFO(this->get_logger(), "full size %lu", candidate.size());
        std::vector<uint8_t> score(candidate.size(), 0);
        // How to select the best solution???
        for(size_t i = 0; i < candidate.size(); ++i){
            if(!flag[i])
                continue;
            RCLCPP_INFO(this->get_logger(), "candidate %lu %.4f %.4f %.4f %.4f %.4f %.4f",
                    i, candidate[i][0], candidate[i][1], candidate[i][2],
                    candidate[i][3], candidate[i][4], candidate[i][5]);
            
            Eigen::Matrix4d tempT = Eigen::Matrix4d::Identity();
            dh_server.get_transform(tempT, candidate[i]);
            Eigen::Vector3d solvedEnd = tempT.topRightCorner(3,1);
            double dis = (solvedEnd - v).norm();
            if(dis > 0.01){
                score[i] = 1 / dis;
            }
            else{
                double total_delta = 0.0;
                for(size_t j = 0; j < candidate[i].size();++j){
                    total_delta += fabs(normalizeAngle(candidate[i][j] - dh_server.get_delta(j)));
                }
                score[i] = 100 + 1 / total_delta;
            }
        }
        if(candidate.size() > 0){
            auto it = std::max_element(score.begin(), score.end());
            int index = std::distance(score.begin(), it);
            RCLCPP_INFO(this->get_logger(), "candidate %d is selected", index);

            sensor_msgs::msg::JointState msg;
            msg.position.assign(candidate[index].begin(), candidate[index].end());
            msg.velocity = {0,0,0,0,0,0};
            msg.effort   = {0,0,0,0,0,0};
            msg.header.stamp = rclcpp::Clock().now();
            msg.name = {
            "base_to_link0",
            "link0_to_link1",
            "link1_to_link2",
            "link2_to_link3",
            "link3_to_link4",
            "link4_to_link5"
            };
            //pub_->publish(msg);
        }else{
            RCLCPP_ERROR(this->get_logger(), "Not Reachable!");
        }

    }
    catch(const std::runtime_error &e){
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    }
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointKinematicsNode>());
  rclcpp::shutdown();
  return 0;
}
