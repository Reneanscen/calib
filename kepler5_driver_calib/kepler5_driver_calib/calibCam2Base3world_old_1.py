#! /usr/bin/python3
# -*- coding: utf-8 -*-

'''
参考资料
https://blog.csdn.net/qq_42699973/article/details/139225786
https://blog.csdn.net/weixin_50161877/article/details/137137558
https://zhuanlan.zhihu.com/p/632052753
'''

import rclpy
from rclpy.node import Node
import numpy as np
# from Zg_Camera import ImageSubscriber

import cv2
from cv_bridge import CvBridge
import threading
import os
import json
import shutil
from sensor_msgs.msg import JointState

from OB_Camera_Ros2 import CameraSubscriber
from eyetohand4world import Camera2RobotOnBase
from pathlib import Path  # path将str转换成path对象，使字符串路径易于操作
FILE = Path(__file__).resolve()  # __file__:当前路径，.resolve():获取绝对路径
ROOT = FILE.parents[0]  # #.parents():路径的父目录

class AcquireAndCalib(Node):
    def __init__(self):
        super().__init__('arm_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            '/manipulation/arm_state',
            self.arm_state_callback,
            1)
        self.subscription  # prevent unused variable warning
        self.angle_xyz=None
    def arm_state_callback(self, msg):
        # 在这里，可以使用self.extra_arg和msg
        # rospy.loginfo("Received position: %s, extra_arg: %s", msg.position)
        # self.get_logger().info('Received arm state:')
        # for i, name in enumerate(msg.name):
        #     self.get_logger().info(f'  {name}: Position: {msg.position[i]}, Velocity: {msg.velocity[i]}, Effort: {msg.effort[i]}')

        self.angle_xyz=msg.velocity[6:]
        print("angle-xyz:",self.angle_xyz)
        
    def arm_end_pose(self):
        return self.angle_xyz

def getImage(color_image,photo_path):
    cv2.imwrite(photo_path,color_image)
    print(f"照片已保存到: {photo_path}")

def getArmEndPose(joint_path, arm_pose_value):
    # 将 arm_pose_value 从 array 转换为 list
    data = {'pose': list(arm_pose_value)}
    with open(joint_path, 'w') as file:
        json.dump(data, file, indent=4)
    print(f"关节位置已保存到: {joint_path}")


def main(args=None):
    rclpy.init(args=args)
    # 创建保存照片的文件夹
    photo_dir = str(ROOT)+ '/' +'calimageworld0010'
    if os.path.exists(photo_dir):
        print("文件夹已存在，正在删除...")
        shutil.rmtree(photo_dir)
        print("文件夹已删除")
        os.makedirs(photo_dir)
        print("文件夹重新创建完成")
    elif not os.path.exists(photo_dir):
        print("文件夹不存在，创建中...")
        os.makedirs(photo_dir)
        print("文件夹创建完成")
    camera_subscriber = CameraSubscriber()
    # acquirecalib=AcquireAndCalib()
    executor = rclpy.executors.MultiThreadedExecutor()
    while True:
        rclpy.spin_once(camera_subscriber, executor=executor, timeout_sec=0.01)
        intr=camera_subscriber.get_intrinsics()
        if intr is not None:
            print("intr is not None")
            intrinsic_cam=[intr['fx'],intr['fy'],intr['cx'],intr['cy'],intr['width'],intr['height']]
            break
        else:
            print("intr is  None")
    
    photo_count=0
    try:
        while rclpy.ok():

            rclpy.spin_once(camera_subscriber, executor=executor, timeout_sec=0.01)
             # 获取订阅者1的图像
            image_ = camera_subscriber.color_image
            if image_ is not None:
                cv2.imshow("Image_rgb", image_)
                key = cv2.waitKey(1)
                if key & 0xFF == ord('s'):
                    photo_path = os.path.join(photo_dir, f"cal_image_{photo_count}.jpg")
                    joint_path = os.path.join(photo_dir, f"cal_image_{photo_count}.json")
                    getImage(image_, photo_path)
                    
                    #    arm_pose_value=acquirecalib.arm_end_pose()
                    #    getArmEndPose(joint_path,arm_pose_value)

                    photo_count += 1
                elif key & 0xFF == ord('q') or key == 27:  # 按下ESC键退出
                    cv2.destroyAllWindows()
                    break
            # rclpy.spin_once(acquirecalib, executor=executor, timeout_sec=0.01)
           
            # user_input=input("是否保存数据(y/n),退出程序(q):")
            # if user_input=='y':
            #     # rclpy.spin_once(camera_subscriber, executor=executor, timeout_sec=1)

            #     # 获取订阅者1的图像
            #     image_ = camera_subscriber.color_image
              
            #     if image_ is not None :
            #         photo_path = os.path.join(photo_dir, f"cal_image_{photo_count}.jpg")
            #         joint_path = os.path.join(photo_dir, f"cal_image_{photo_count}.json")
            #         getImage(image_, photo_path)

            #         # arm_pose_value=acquirecalib.angle_xyz
            #         # getArmEndPose(joint_path,arm_pose_value)

            #         photo_count += 1
            #     else:
            #         print("没有获取图像，重新输入")
            # elif user_input=='n':
            #     pass
            # elif user_input=='q':
            #     break


           
        print("intrinsic_cam",intrinsic_cam)
        cam_robot=Camera2RobotOnBase(intrinsic_cam)
        t_camera2base=cam_robot.camera2robotbase()
        f=open(str(ROOT)+ '/' +"camera2robotbase.txt",'w')
        f.write(str(t_camera2base))
        f.close()
    except KeyboardInterrupt:
        pass

    finally:
        # 销毁节点
        camera_subscriber.destroy_node()
        # acquirecalib.destroy_node()
        rclpy.shutdown()
        # cv2.destroyAllWindows()  # 确保关闭所有OpenCV窗口

if __name__ == '__main__':
    main()