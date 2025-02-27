#! /usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from sensor_msgs.msg import Image,CameraInfo
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d

from cv_bridge import CvBridge
import cv2
import threading


class camera_info:
    def __init__(self):
        self.cam_intr_K=None
        self.cam_intr_w=None
        self.cam_intr_h=None
        rospy.Subscriber('/feynman_camera/default/rgb/camera_info',CameraInfo,self.callback_intrinsics)
        rospy.sleep(1)
        
    def callback_intrinsics(self,data):
        #print(data.K)
        self.cam_intr_K=data.K
        self.cam_intr_w=data.width
        self.cam_intr_h=data.height
        
    def get_intr(self):
        
        return self.cam_intr_K

class ImageSubscriber:
    def __init__(self):
        #self.topic = topic
        self.color_image  = None
        self.depth_image = None
        #self.o3d_cloud = o3d.geometry.PointCloud()
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        # self.intr= {"fx":333.4180603027344,
        # "fy":333.4180603027344,
        # "cx":328.2615051269531,
        # "cy":201.4126739501953,
        # "width":640,
        # "height":400}#相机内参
        # self.intr= {"fx":302.1266174316406,
        #          "fy":302.2621765136719,
        #          "cx":319.5,
        #          "cy":199.5,
        #          "width":640,
        #          "height":400}#相机内参

        self.cam_info=camera_info()
        self.inf_K=self.cam_info.cam_intr_K
        self.inf_w=self.cam_info.cam_intr_w
        self.inf_h=self.cam_info.cam_intr_h

        self.intr= {"fx":self.inf_K[0],
                "fy":self.inf_K[4],
                "cx":self.inf_K[2],
                "cy":self.inf_K[5],
                "width":self.inf_w,
                "height":self.inf_h}#相机内参
        
       
        #print(self.intr)
          # 获取多种颜色
        hex = ('FF3838', 'FF9D97', 'FF701F', 'FFB21D', 'CFD231', '48F90A', '92CC17', '3DDB86', '1A9334', '00D4BB',
               '2C99A8', '00C2FF', '344593', '6473FF', '0018EC', '8438FF', '520085', 'CB38FF', 'FF95C8', 'FF37C7')
        self.palette = [tuple(int(('#' + c)[1 + i:1 + i + 2], 16) for i in (0, 2, 4)) for c in hex]  # hax2rgb



        # 创建订阅者，订阅指定的topic，数据类型为Image
        rospy.Subscriber('/feynman_camera/default/rgb/image_rect_color', Image, self.callback_rgb)
        rospy.Subscriber('/feynman_camera/default/depthalignrgb/image_raw', Image, self.callback_d)
        

    def callback_rgb(self, data):
        # 使用cv_bridge将ROS Image消息转换为OpenCV图像
        with self.lock:
            self.color_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def get_image(self):
        # 获取最新的图像
        with self.lock:
            return self.color_image 
            
    def callback_d(self, data):
        # 使用cv_bridge将ROS Image消息转换为OpenCV图像
        with self.lock:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
            

    def get_image_d(self):
        # 获取最新的图像
        with self.lock:
            return self.depth_image
            

    def pixel_to_camera(self,pixel_coords, depth):
    
        # 相机内参矩阵
        fx=self.intr['fx']
        fy=self.intr['fy']
        cx=self.intr['cx']
        cy=self.intr['cy']
        # 像素坐标
        u = pixel_coords[0]
        v = pixel_coords[1]

        # 深度相机坐标系下的坐标
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth

        # print("深度相机坐标系下的坐标：")
        # print("X:", x)
        # print("Y:", y)
        # print("Z:", z)
        p_cam=[x,y,z]
        return p_cam
            

    def get_points(self):
            #点云配准
        color_image=self.color_image
        depth_image=self.depth_image
        intrcam=self.intr #相机内参
    
        #转化为open3d格式
        if color_image is not None:
            o3d_color = o3d.geometry.Image(color_image)  
            o3d_depth = o3d.geometry.Image(depth_image)
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d_color, o3d_depth,
                                                                                depth_scale=1000,
                                                                                depth_trunc=5,
                                                                                convert_rgb_to_intensity=False)
            cam_intr = o3d.camera.PinholeCameraIntrinsic(intrcam['width'], intrcam['height'],
                                                            intrcam['fx'], intrcam['fy'],
                                                            intrcam['cx'], intrcam['cy'])

            # 原始带颜色点云
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, cam_intr)  
            #o3d.visualization.draw_geometries([pcd])

            return pcd


def display_images():
    # 创建两个图像订阅者对象
    subscriber_rgbd = ImageSubscriber()
    #subscriber_depth = ImageSubscriber()
    # vis=o3d.visualization.Visualizer()
    # vis.create_window(window_name='pcd',width=800,height=600)
    # pointscloud=o3d.geometry.PointCloud()
    while not rospy.is_shutdown():
        # 获取订阅者1的图像
        image_ = subscriber_rgbd.get_image()
        if image_ is not None:
            cv2.imshow("Image_rgb", image_)

        # 获取订阅者2的图像
        image_depth = subscriber_rgbd.get_image_d()
       

        if image_depth is not None:
            # x=400
            # y=100
            # depth_z=image_depth[y,x]
            # pixel_coords=(x,y)
            # print(pixel_coords)
            # p_cam=subscriber_rgbd.pixel_to_camera(pixel_coords, depth_z)
            # print("----------z----",p_cam[:])
            # pcd=subscriber_rgbd.get_points()
            #-------------open3d点云显示----------------#
    
            # pointscloud= pcd
            # vis.update_geometry(pointscloud)  # 更新窗口中显示的内容
            # vis.add_geometry(pointscloud)     # 添加点云到可视化窗口
   
 
            # vis.poll_events()
            # vis.update_renderer()    # 更新显示窗口
            # vis.clear_geometries()   # 清空显示内容
            #----------------------#
            #vis.run()
            #----------------------#
            cv2.imshow("Image_depth", image_depth)

        key=cv2.waitKey(1)
        if key == ord('q'):
            cv2.destroyAllWindows()
            #vis.destroy_window()
            break

if __name__ == '__main__':
    rospy.init_node("yolo_master", anonymous=True)

    # 创建显示图像的线程
    display_thread = threading.Thread(target=display_images)
    display_thread.start()

    rospy.spin()


    
    

   
   
    
