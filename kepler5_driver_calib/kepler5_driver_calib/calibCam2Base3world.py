#! /usr/bin/python3
# -*- coding: utf-8 -*-

'''
参考资料
https://blog.csdn.net/qq_42699973/article/details/139225786
https://blog.csdn.net/weixin_50161877/article/details/137137558
https://zhuanlan.zhihu.com/p/632052753
'''

import rospy
import numpy as np
from Zg_Camera import ImageSubscriber
import cv2
from cv_bridge import CvBridge
import threading
import os
import json
import shutil
from sensor_msgs.msg import JointState
from eyetohand4 import Camera2RobotOnBase
from std_msgs.msg import Int32  # 使用 Int32 类型
from enum import Enum

from pathlib import Path  # path将str转换成path对象，使字符串路径易于操作
FILE = Path(__file__).resolve()  # __file__:当前路径，.resolve():获取绝对路径
ROOT = FILE.parents[0]  # #.parents():路径的父目录

class AcquireAndCalib:
    def __init__(self):
        self.subscriber = rospy.Subscriber("/manipulation/arm_state", JointState, self.arm_state_callback)
        self.angle_xyz=[0,0,0,0,0,0]
    
    def arm_state_callback(self, msg):
        # 在这里，可以使用self.extra_arg和msg
        # rospy.loginfo("Received position: %s, extra_arg: %s", msg.position)
        
        self.angle_xyz=msg.velocity[6:]

        print("angle-xyz:",self.angle_xyz)
        
    def arm_end_pose(self):
        return self.angle_xyz

    def is_fixed_between_arrays(self,arr1, arr2, threshold=0.1):
        if len(arr1) != len(arr2):
            print("数组长度不同，无法比较")
            return False  # 如果数组长度不同，无法比较
        # 2. 统计不相同的元素对数
        difference_count = 0
        for i in range(len(arr1)):
            if abs(arr1[i] - arr2[i]) > threshold:
                difference_count += 1
            # 如果不相同的元素对数达到 3 个，直接返回 False
            if difference_count >= 3:
                return True
        return False

def getImage(color_image,photo_path):
    cv2.imwrite(photo_path,color_image)
    print(f"照片已保存到: {photo_path}")

def getArmEndPose(joint_path,arm_pose_value):
    # 创建数据
    data = {
        'pose': arm_pose_value,
    }

    # 写入JSON文件
    with open(joint_path, 'w') as file:
        json.dump(data, file, indent=4)

def calib_camera_base():
    # 创建保存照片的文件夹
    photo_dir = str(ROOT)+ '/' +'calimageworld'
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
  
    # 创建图像订阅者对象
    subscriber_rgbd = ImageSubscriber()
    
    fx=subscriber_rgbd.intr['fx']
    fy=subscriber_rgbd.intr['fy']
    cx=subscriber_rgbd.intr['cx']
    cy=subscriber_rgbd.intr['cy']
    cw=subscriber_rgbd.intr['width']
    ch=subscriber_rgbd.intr['height']
   
    intrinsic_cam=[fx,fy,cx,cy,cw,ch]
    acquirecalib=AcquireAndCalib()

    try:
        photo_count = 0
        init_arr=[0,0,0,0,0,0]
        while not rospy.is_shutdown():
           # 获取订阅者1的图像
           image_ = subscriber_rgbd.get_image()
           if image_ is not None:
               arm_pose_value=acquirecalib.arm_end_pose()
               is_save=acquirecalib.is_fixed_between_arrays(init_arr, arm_pose_value, threshold=5)
               init_arr=arm_pose_value

               print("is_save",is_save)
               if  is_save:
                   photo_path = os.path.join(photo_dir, f"cal_image_{photo_count}.jpg")
                   joint_path = os.path.join(photo_dir, f"cal_image_{photo_count}.json")
                   getImage(image_, photo_path)
                   
                   getArmEndPose(joint_path,arm_pose_value)
                   print("pose_value",arm_pose_value)
                   photo_count += 1
               if photo_count>14: break

        cam_robot=Camera2RobotOnBase(intrinsic_cam)
        t_camera2base=cam_robot.camera2robotbase()
        f=open(str(ROOT)+ '/' +"camera2robotbase.txt",'w')
        f.write(str(t_camera2base))
        f.close()
    finally:
        rospy.loginfo("Calibration completed.")
        pass
if __name__ == '__main__':
    rospy.init_node("cal_master", anonymous=True)

    # 创建显示图像的线程
    display_thread = threading.Thread(target=calib_camera_base)
    display_thread.start()

    rospy.spin()


