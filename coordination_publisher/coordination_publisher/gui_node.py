#!/usr/bin/env python
#-*- coding: utf-8 -*-
# Author: Zhenghao Li
# Email: lizhenghao@shanghaitech.edu.cn
# Institute: SIST
# Date: 2025-06-10
import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton
)
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

class GuiPublisher(Node):
    def __init__(self):
        super().__init__('gui_publisher')
        self.publisher_ = self.create_publisher(Pose, 'gui_pose', 10)

class MainWindow(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.setWindowTitle('SI190C Pose commander')
        self.ros_node = ros_node

        self.inputs = {}
        self.default_values = {
            'q_x': '0.0',
            'q_y': '0.0',
            'q_z': '0.0',
            'q_w': '0.0',
            'x': '0.0',
            'y': '0.0',
            'z': '0.0',
        }

        layout = QVBoxLayout()

        for label in ['q_x', 'q_y', 'q_z', 'q_w', 'x', 'y', 'z']:
            h = QHBoxLayout()
            l = QLabel(label)
            e = QLineEdit()
            e.setText(self.default_values[label])
            self.inputs[label] = e
            h.addWidget(l)
            h.addWidget(e)
            layout.addLayout(h)

        self.button = QPushButton('发送')
        self.button.clicked.connect(self.publish_data)
        layout.addWidget(self.button)

        self.setLayout(layout)

    def publish_data(self):
        try:
            q_x = float(self.inputs['q_x'].text())
            q_y = float(self.inputs['q_y'].text())
            q_z = float(self.inputs['q_z'].text())
            q_w = float(self.inputs['q_w'].text())
            x = float(self.inputs['x'].text())
            y = float(self.inputs['y'].text())
            z = float(self.inputs['z'].text())

            msg = Pose()
            msg.position.x = x
            msg.position.y = y
            msg.position.z = z
            msg.orientation.x = q_x
            msg.orientation.y = q_y
            msg.orientation.z = q_z
            msg.orientation.w = q_w  # 默认简化为0

            self.ros_node.publisher_.publish(msg)
            print(f"已发送: {msg}")

        except ValueError:
            print("请输入有效数字！")

def main(args=None):
    rclpy.init(args=args)
    ros_node = GuiPublisher()

    app = QApplication(sys.argv)
    window = MainWindow(ros_node)
    window.show()
    app.exec_()

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

