/*
 * File: joint_comm_node.cpp
 * Author: Zhenghao Li
 * Email: lizhenghao@shanghaitech.edu.cn
 * Institute: SIST
 * Created: 2025-05-29
 * Last Modified: 2025-09-25
 */

#include "io_context/io_context.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "serial_driver/serial_driver.hpp"
#include <chrono>
#include <cstdint>
#include <memory>
#include <cmath>
#include <queue>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <sys/types.h>
#include <thread>

using namespace drivers::serial_driver;


double normalizeAngle(double angle)
{
    angle = std::fmod(angle, 2 * M_PI); // 先模到 (-2pi, 2pi)
    if (angle <= 0)
        angle += 2 * M_PI;  // 转到 (0, 2pi)
    return angle;    // 映射到 (0, 2pi]
}


class JointSerialComm : public rclcpp::Node {

  std::queue<std::vector<uint8_t>> commands;
  std::vector<uint8_t> current_command;
  std::thread t_;
  std::shared_ptr<drivers::common::IoContext> io_context_;
  rclcpp::TimerBase::SharedPtr timer_;
  int reducer_ratio[6];
  uint8_t motor_dir[6];
  double axis_dir[6];
  std::vector<double> rcv_pos;
  std::vector<double> snd_pos;
  std::vector<double> crt_pos;

  void parse(const std::vector<uint8_t> &arr){
      uint32_t i = 0;
      while(i < arr.size()){
          if(arr[i] != 0 || !current_command.empty())
              current_command.push_back(arr[i]);
          if(arr[i] == 0x6B){
              commands.push(current_command);
              current_command.clear();
          }
          ++i;
      }
  }

public:
  JointSerialComm()
  : Node("joint_serial_comm"), rcv_pos(6, 0), snd_pos(6, 0), crt_pos(6, 0)
  {
    std::string port = "/dev/ttyUSB0";
    int baud_rate = 115200;

    // 创建串口驱动实例
    io_context_ = std::make_shared<drivers::common::IoContext>(1);
    serial_port_ = std::make_unique<SerialDriver>(*io_context_);
    SerialPortConfig config(baud_rate, FlowControl::NONE, Parity::NONE, StopBits::ONE);
    serial_port_->init_port(port, config);
    serial_port_->port()->open();
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&JointSerialComm::joint_callback, this, std::placeholders::_1)
    );

    reducer_ratio[0] = 20;
    reducer_ratio[1] = 50;
    reducer_ratio[2] = 30;
    reducer_ratio[3] = 30;
    reducer_ratio[4] = 30;
    reducer_ratio[5] = 1;

    motor_dir[0] = 0;
    motor_dir[1] = 0;
    motor_dir[2] = 0;
    motor_dir[3] = 0;
    motor_dir[4] = 1;
    motor_dir[5] = 0;

    axis_dir[0] = -1;
    axis_dir[1] = 1;
    axis_dir[2] = -1;
    axis_dir[3] = -1;
    axis_dir[4] = 1;
    axis_dir[5] = 1;

    timer_ = this->create_wall_timer(std::chrono::milliseconds(200),
                                    std::bind(&JointSerialComm::require_position, this));
    RCLCPP_INFO(this->get_logger(), "JointSerialComm node started, serial initialized on %s", port.c_str());
    async_receive();

    home();
  }

private:
  std::unique_ptr<SerialDriver> serial_port_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;

  int angle_to_pulse(double angle_deg) {
    return static_cast<int>(angle_deg / (2 * M_PI) * 3200);  // 假设 1 度 = 100 脉冲
  }

  void async_receive(){
    auto port = serial_port_->port();
    port->async_receive([this](const std::vector<uint8_t> &data, const size_t &size){
      if(size > 0){
        parse(data);
        while(!commands.empty()){
            std::vector<uint8_t> cmd = commands.front();
            get_pos(cmd);
            commands.pop();
        }
        usleep(2000);
        RCLCPP_DEBUG(this->get_logger(), "async receive %lu", size);
      }
      async_receive();
    });
  }

  void require_position(){
    for(uint8_t i = 1; i <= 6; ++i){
        std::vector<uint8_t> frame = {i, 0x30, 0x6B};
        serial_port_->port()->send(frame);
        usleep(10000);
    }
  }

  void send_frame(uint8_t joint_id, int pulse, uint8_t dir = 0) {
    std::vector<uint8_t> frame = {0x01, 0xFD, 0x00, 0x13, 0x50, 0x0A,
                         0, 0, 0, 0, 0x01, 0x00, 0x6B};

    frame[0] = joint_id;
    frame[2] = dir;
    frame[6] = (pulse >> 24) & 0xFF;
    frame[7] = (pulse >> 16) & 0xFF;
    frame[8] = (pulse >> 8) & 0xFF;
    frame[9] = (pulse) & 0xFF;

    //std::string data(reinterpret_cast<char*>(frame), sizeof(frame));
    serial_port_->port()->send(frame);
  }

  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Get Command");
    for (size_t i = 0; i <= std::min(msg->position.size(), size_t(6)); ++i) {
    //for (size_t i = 6; i <= std::min(msg->position.size(), size_t(6)); ++i) {
      rcv_pos[i] = normalizeAngle(msg->position[i] * axis_dir[i]);
      if( abs(rcv_pos[i] - 0) < 1e-3  || abs(rcv_pos[i] - 2 *M_PI) < 1e-3){
          rcv_pos[i] = 0;
      }
      if(std::abs(snd_pos[i] - rcv_pos[i]) > 0.01){
        RCLCPP_INFO(this->get_logger(), "joint%lu:%f", i+1, rcv_pos[i]);
        if(rcv_pos[i] < 0){
          RCLCPP_ERROR(this->get_logger(), "joint%lu:%f direction can't be negative!", i+1, rcv_pos[i]);
          continue;
        }

        int pulse = angle_to_pulse(rcv_pos[i]* reducer_ratio[i]);
        send_frame(i + 1, pulse, motor_dir[i]);
        snd_pos[i] = rcv_pos[i];
        usleep(50000);
      }
    }
  }

  void home(){
    std::vector<uint8_t> homing_setting(20, 0);
    homing_setting[0] = 0x01;
    homing_setting[1] = 0x4C;
    homing_setting[2] = 0xAE;
    homing_setting[3] = 0x01;
    homing_setting[4] = 0x02;
    homing_setting[5] = 0x01;
    homing_setting[6] = (30 >> 8) & 0xff;
    homing_setting[7] = 30 & 0xff;
    homing_setting[8] = (10000 >> 24) & 0xff;
    homing_setting[9] = (10000 >> 16) & 0xff;
    homing_setting[10] = (10000 >> 8) & 0xff;
    homing_setting[11] = 10000 & 0xff;
    homing_setting[12] = (4000 >> 8) & 0xff;
    homing_setting[13] = 4000 & 0xff;
    homing_setting[14] = (400 >> 8) & 0xff;
    homing_setting[15] = 400 & 0xff;
    homing_setting[16] = (60 >> 8) & 0xff;
    homing_setting[17] = 60 & 0xff;
    homing_setting[18] = 0x01;
    homing_setting[19] = 0x6B;


    //std::string data(reinterpret_cast<char*>(frame), sizeof(frame));
    serial_port_->port()->send(homing_setting);
    usleep(100000);
  }

  void get_pos(std::vector<uint8_t> &cmd)
  {
    if(cmd.size() < 2)
      return;
    if(cmd[1] == 0x30 && cmd.size() == 8){

      int pos = (int)cmd[6] + ((int)cmd[5] << 8) + ((int)cmd[4] << 16) + ((int)cmd[3] << 24);
      int index = cmd[0] - 1;
      if(index >= 0 && cmd[0] <= 5)
        crt_pos[index] = double(pos) / 3200 / reducer_ratio[0] * 2 * M_PI;
    }
    if(cmd[1] == 0x00){
      RCLCPP_ERROR(this->get_logger(), "failed");
    }
    if(cmd[1] == 0xF5){
      RCLCPP_ERROR(this->get_logger(), "torque back");
    }
  }

  void read_loop(){
      while(rclcpp::ok()){
          //async_receive();
          std::vector<uint8_t> frame;
          frame.resize(8);
          //RCLCPP_INFO(this->get_logger(), "receive %lu", serial_port_->port()->receive(frame));
          parse(frame);
          while(!commands.empty()){
            std::vector<uint8_t> cmd = commands.front();
            get_pos(cmd);
            commands.pop();
          }
          usleep(2000);
      }

  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointSerialComm>());
  rclcpp::shutdown();
  return 0;
}
