/*
 * File: pose_publisher.cpp
 * Author: Zhenghao Li
 * Email: lizhenghao@shanghaitech.edu.cn
 * Institute: SIST
 * Created: 2025-06-12
 * Last Modified: 2025-06-12
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

class PosePublisher : public rclcpp::Node {
public:
    PosePublisher() : Node("pose_publisher"), current_index_(0) {
        publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/gui_pose", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(10), std::bind(&PosePublisher::publishPose, this));

        poses_ = {
            createPose(0.248, -0.004, -0.049, 0.826, 0.544, 0.113, 0.098),
            createPose(0.146, -0.2565, 0.005, 0.908, 0.201, 0.259, 0.261),
            createPose(-0.038, 0.167, 0.237, 0.079, 0.365, 0.368, 0.852)
        };
    }

private:
    void publishPose() {
        publisher_->publish(poses_[current_index_]);
        RCLCPP_INFO(this->get_logger(), "Publishing Pose [%lu]", current_index_);
        current_index_ = (current_index_ + 1) % poses_.size();
    }

    geometry_msgs::msg::Pose createPose(double x, double y, double z, double qx, double qy, double qz, double qw) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        pose.orientation.x = qx;
        pose.orientation.y = qy;
        pose.orientation.z = qz;
        pose.orientation.w = qw;
        return pose;
    }

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<geometry_msgs::msg::Pose> poses_;
    size_t current_index_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PosePublisher>());
    rclcpp::shutdown();
    return 0;
}
