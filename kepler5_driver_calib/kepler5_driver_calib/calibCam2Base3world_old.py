#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
import os
import json
import shutil
from sensor_msgs.msg import JointState, Image
from OB_Camera_Ros2 import CameraSubscriber
from eyetohand4world import Camera2RobotOnBase
from pathlib import Path

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]

class AcquireAndCalib(Node):
    def __init__(self):
        super().__init__('arm_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            '/manipulation/arm_state',
            self.arm_state_callback,
            10)
        self.angle_xyz = None
        self.received_new_pose = False

    def arm_state_callback(self, msg):
        self.angle_xyz = msg.velocity[6:]
        self.received_new_pose = True
        print("angle-xyz:", self.angle_xyz)

    def get_latest_arm_pose(self):
        self.received_new_pose = False
        return self.angle_xyz

def getImage(color_image, photo_path):
    cv2.imwrite(photo_path, color_image)
    print(f"照片已保存到: {photo_path}")


def getArmEndPose(joint_path, arm_pose_value):
    # 将 arm_pose_value 从 array 转换为 list
    data = {'pose': list(arm_pose_value)}
    with open(joint_path, 'w') as file:
        json.dump(data, file, indent=4)
    print(f"关节位置已保存到: {joint_path}")

def main(args=None):
    rclpy.init(args=args)

    photo_dir = str(ROOT / 'calimageworld00_wt')
    if os.path.exists(photo_dir):
        print("文件夹已存在，正在删除...")
        shutil.rmtree(photo_dir)
        print("文件夹已删除")
    os.makedirs(photo_dir)
    print("文件夹创建完成")

    camera_subscriber = CameraSubscriber()
    acquirecalib = AcquireAndCalib()

    executor = rclpy.executors.MultiThreadedExecutor()

    while True:
        rclpy.spin_once(camera_subscriber, executor=executor, timeout_sec=0.01)
        intr = camera_subscriber.get_intrinsics()
        if intr is not None:
            print("intr is not None")
            intrinsic_cam = [
                intr['fx'], intr['fy'], intr['cx'], intr['cy'], intr['width'], intr['height']
            ]
            break
        else:
            print("intr is None")

    photo_count = 0
    try:
        while rclpy.ok():
            print("准备获取用户输入...")
            user_input = input("是否保存数据(y/n),退出程序(q):")

            if user_input == 'y':
                # 等待直到收到新的图像帧和新的关节状态
                camera_subscriber.received_new_image = False
                acquirecalib.received_new_pose = False

                while not (camera_subscriber.received_new_image and acquirecalib.received_new_pose):
                    rclpy.spin_once(camera_subscriber, executor=executor, timeout_sec=0.01)
                    rclpy.spin_once(acquirecalib, executor=executor, timeout_sec=0.01)

                latest_image = camera_subscriber.get_latest_image()
                arm_pose_value = acquirecalib.get_latest_arm_pose()

                if latest_image is not None and arm_pose_value is not None:
                    photo_path = os.path.join(photo_dir, f"cal_image_{photo_count}.jpg")
                    joint_path = os.path.join(photo_dir, f"cal_image_{photo_count}.json")
                    getImage(latest_image, photo_path)
                    getArmEndPose(joint_path, arm_pose_value)
                    photo_count += 1
                else:
                    print("没有获取图像或末端位置，重新输入")
            elif user_input == 'n':
                continue
            elif user_input == 'q':
                break

        print("intrinsic_cam", intrinsic_cam)
        cam_robot = Camera2RobotOnBase(intrinsic_cam)
        t_camera2base = cam_robot.camera2robotbase()
        with open(str(ROOT / "camera2robotbase00_wt.txt"), 'w') as f:
            f.write(str(t_camera2base))
    except KeyboardInterrupt:
        pass
    finally:
        camera_subscriber.destroy_node()
        acquirecalib.destroy_node()
 
        rclpy.shutdown()

if __name__ == '__main__':
    main()
