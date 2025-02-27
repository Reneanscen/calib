import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import open3d as o3d
class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()
        
        self.color_image = None  # 初始化变量来存储彩色图像
        self.depth_image = None  # 初始化变量来存储深度图像
        self.cam_intr_K = None
        self.cam_intr_w = None
        self.cam_intr_h = None
        self.cam_intr_D = None  # 增加畸变系数

        # self.latest_image = None
        self.received_new_image = False
        # self.latest_depth = None
        self.received_new_depth = False
        self.subscription_rgb = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.callback_rgb,
            10)
        self.subscription_rgb  # 防止未使用变量警告

        self.subscription_depth = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.callback_d,
            10)
        self.subscription_depth  # 防止未使用变量警告

        self.subscription_intrinsics = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.callback_intrinsics,
            10)
        self.subscription_intrinsics  # 防止未使用变量警告

        # self.get_logger().info('Subscriber Initialized')

    def callback_rgb(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.color_image = cv_image  # 存储图像而不是直接显示
        self.received_new_image = True

    def get_latest_image(self):
        self.received_new_image = False
        return self.color_image

    def callback_d(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        self.received_new_depth = True
        # cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        # cv_depth_normalized = cv2.normalize(cv_depth, None, 0, 255, cv2.NORM_MINMAX)
        # self.depth_image = cv_depth_normalized.astype(np.uint8)  # 存储处理后的图像
    def get_latest_depth(self):
        self.received_new_depth = False
        return self.depth_image

    def callback_intrinsics(self, msg):

        # self.get_logger().info('Received Camera Intrinsics')
        try:
            self.cam_intr_D = msg.d
            self.cam_intr_K = msg.k
            self.cam_intr_w = msg.width
            self.cam_intr_h = msg.height
        except Exception as e:
            self.get_logger().error('Failed to extract camera intrinsics: ' + str(e))
    
    def get_intrinsics(self):
        try:
            if self.cam_intr_K is not None:
                intr= {"fx":self.cam_intr_K[0],
                        "fy":self.cam_intr_K[4],
                        "cx":self.cam_intr_K[2],
                        "cy":self.cam_intr_K[5],
                        "width":self.cam_intr_w,
                        "height":self.cam_intr_h}#相机内参
                return intr
        except Exception as e:
            self.get_logger().error('Failed to extract camera intrinsics: ' + str(e))
    
    def pixel_to_camera(self,pixel_coords, depth):
        intr=self.get_intrinsics()
        # 相机内参矩阵
        fx=intr['fx']
        fy=intr['fy']
        cx=intr['cx']
        cy=intr['cy']
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


    def get_points(self,color_image,depth_image,intrcam):
        #点云配准
        # color_image=self.color_image
        # depth_image=self.depth_image
        # intrcam=self.get_intrinsics() #相机内参
    
        #转化为open3d格式
        if color_image is not None and depth_image is not None:
            # 将 BGR 图像转换为 RGB 图像
            color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
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
            # o3d.visualization.draw_geometries([pcd])

            return pcd
def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    executor = rclpy.executors.MultiThreadedExecutor()
    photo_count = 0
    intrcam=None
    try:
        while rclpy.ok():
            rclpy.spin_once(camera_subscriber, executor=executor, timeout_sec=0.01)
            intrcam=camera_subscriber.get_intrinsics()
            if intrcam is None:
                continue
            # print(",ooo",camera_subscriber.get_intrinsics())
            # 检查图像是否已接收并处理
            if camera_subscriber.color_image is not None:
                cv2.imshow("RGB Image", camera_subscriber.color_image)
            if camera_subscriber.depth_image is not None:
                cv2.imshow("Depth Image", camera_subscriber.depth_image)
            key=cv2.waitKey(1)
            if key == ord('q'):
                cv2.destroyAllWindows()
                break
           
            pcd=camera_subscriber.get_points(camera_subscriber.color_image,camera_subscriber.depth_image,intrcam)
            o3d.visualization.draw_geometries([pcd])
    except KeyboardInterrupt:
        pass

    finally:
        # 销毁节点
        camera_subscriber.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()  # 确保关闭所有OpenCV窗口

if __name__ == '__main__':
    main()
