# 文件说明：棋盘格手眼标定（眼在手外）
 
'''
参考：https://blog.csdn.net/weixin_55004283/article/details/135432910
在执行手眼标定时，需要将标定板固定在机械臂末端，并同时将相机固定在另一侧。
接着控制机械臂末端位于不同的位置，记录下此时机械臂相对于基座的位姿，并使用相机拍摄标定板上的棋盘格图像。
将图像放入./images文件夹中，并将位姿信息输入到chessboard_handeye_calibration.py文件的pose_vectors变量中。
最后运行chessboard_handeye_calibration.py，即可得到相机相对于机械臂基座的位姿矩阵。
https://blog.csdn.net/handsome_wang5/article/details/129227308
'''
 
import cv2
import numpy as np
import transforms3d
import glob
from math import *
import json
from scipy.spatial.transform import Rotation as R  

import eyetohandother as eyetohand

from pathlib import Path  # path将str转换成path对象，使字符串路径易于操作
FILE = Path(__file__).resolve()  # __file__:当前路径，.resolve():获取绝对路径
ROOT = FILE.parents[0]  # #.parents():路径的父目录

#用于根据欧拉角计算旋转矩阵
def myRPY2R_robot(x, y, z):
    x = np.radians(x)
    y = np.radians(y)
    z = np.radians(z)
    Rx = np.array([[1, 0, 0], [0, cos(x), -sin(x)], [0, sin(x), cos(x)]])
    Ry = np.array([[cos(y), 0, sin(y)], [0, 1, 0], [-sin(y), 0, cos(y)]])
    Rz = np.array([[cos(z), -sin(z), 0], [sin(z), cos(z), 0], [0, 0, 1]])
    R = Rz@Ry@Rx
    return R

class Camera2RobotOnBase:
    def __init__(self,intrinsic_cam):
         # 定义棋盘格参数
        self.set_square_size = 15.0  # 格子的边长为15mm
        self.set_pattern_size = (11, 8)   # 在这个例子中，假设标定板有11个内角点和8个内角点
        
        # 导入相机内参和畸变参数
        # 焦距 fx, fy, 光心 cx, cy
        # 畸变系数 k1, k2 
        #intrinsic_cam=[fx,fy,cx,cy,cw,ch]
        self.fx, self.fy, self.cx, self.cy = intrinsic_cam[:4]
        self.k1, self.k2 = 0, 0
        self.K = np.array([[self.fx, 0, self.cx], 
                    [0, self.fy, self.cy], 
                    [0, 0, 1]], dtype=np.float64)   # K为相机内参矩阵
        self.dist_coeffs = np.array([self.k1, self.k2, 0, 0], dtype=np.float64)   # 畸变系数

    def pose_vectors_to_base2end_transforms(self,pose_vectors):
        # 提取旋转矩阵和平移向量
        R_base2ends = []
        t_base2ends = []

        # 迭代的到每个位姿的旋转矩阵和平移向量
        for pose_vector in pose_vectors:
            # 提取旋转矩阵和平移向量
            R_end2base = self.euler_to_rotation_matrix(pose_vector[3], pose_vector[4], pose_vector[5])
            t_end2base = np.array([pose_vector[0],pose_vector[1],pose_vector[2]])
            # print("R_end2base",R_end2base)
            # R_end2base_2=myRPY2R_robot(pose_vector[3], pose_vector[4], pose_vector[5])
            # print("R_end2base2",R_end2base_2)

            # 将旋转矩阵和平移向量组合成齐次位姿矩阵
            pose_matrix = np.eye(4)
            pose_matrix[:3, :3] = R_end2base
            pose_matrix[:3, 3] = t_end2base
    
            # 求位姿矩阵的逆矩阵
            R_base2end=np.linalg.inv(R_end2base)
            t_base2end=-R_base2end@t_end2base
    
            # 将旋转矩阵和平移向量保存到列表
            R_base2ends.append(R_base2end)
            t_base2ends.append(t_base2end)
            
        return R_base2ends, t_base2ends
 
    def euler_to_rotation_matrix(self,rx, ry, rz, unit='deg'):  # rx, ry, rz是欧拉角，单位是度
        '''
        将欧拉角转换为旋转矩阵：R = Rz * Ry * Rx
        :param rx: x轴旋转角度
        :param ry: y轴旋转角度
        :param rz: z轴旋转角度
        :param unit: 角度单位，'deg'表示角度，'rad'表示弧度
        :return: 旋转矩阵
        '''
        if unit =='deg':
            # 把角度转换为弧度
            rx = np.radians(rx)
            ry = np.radians(ry)
            rz = np.radians(rz)
        
        # 计算旋转矩阵Rz 、 Ry 、 Rx
        Rx = transforms3d.axangles.axangle2mat([1, 0, 0], rx)
        Ry = transforms3d.axangles.axangle2mat([0, 1, 0], ry)
        Rz = transforms3d.axangles.axangle2mat([0, 0, 1], rz)
    
        # 计算旋转矩阵R = Rz * Ry * Rx
        #旋转顺序XYZ
        rotation_matrix = np.dot(Rz, np.dot(Ry, Rx))
        # rotation_matrix = np.dot(np.dot(Rz,Ry), Rx)

        return rotation_matrix
    
    def camera2robotbase(self):
        square_size=self.set_square_size 
        pattern_size=self.set_pattern_size
        # 所有图像的路径
        #images = glob.glob('./images/*.png')
        images = glob.glob(str(ROOT)+ '/' +'calimage0010/*.jpg')
        # images = sorted(images)
        import re
        # images = sorted(images, key=lambda x: int(re.search(r'cal_image_(\d+)', x).group(1)))
        print("images",images)
        
        # 准备位姿数据
        obj_points = []  # 用于保存世界坐标系中的三维点
        img_points = []  # 用于保存图像平面上的二维点

        pose_vectors1=[]
        pose_boards=[]
        
        # 创建棋盘格3D坐标
        objp = np.zeros((np.prod(pattern_size), 3), dtype=np.float64)
        objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * square_size
        #objp = objp[::-1]
        R_board2cameras = []  # 用于保存旋转矩阵
        t_board2cameras = []  # 用于保存平移向量
        board2cameras_matrixs= []
        # 迭代处理图像
        det_success_num = 0  # 用于保存检测成功的图像数量
        for image in images:
            img = cv2.imread(image)   # 读取图像
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)   # RGB图像转换为灰度图像
            # 棋盘格检测
            ret, corners = cv2.findChessboardCorners(gray, pattern_size)
            if ret:
                det_success_num += 1
                # 如果成功检测到棋盘格，添加图像平面上的二维点和世界坐标系中的三维点到列表
                obj_points.append(objp)
                img_points.append(corners)

                path_before_jpg = image.split('.jpg')[0]
                print("path_before_jpg",path_before_jpg)
                ret, rvec, t_board2camera= cv2.solvePnP(objp, corners, self.K, self.dist_coeffs) 
                R_board2camera, _ = cv2.Rodrigues(rvec)   # 输出：R为旋转矩阵和旋转向量的关系  输入：rvec为旋转向量

                # 将标定板相对于相机坐标系的旋转矩阵和平移向量保存到列表
                # 创建Rotation对象  
                rot = R.from_matrix(R_board2camera)  
                t_=t_board2camera.tolist()
                #print("t_",t_[0])
                # 获取ZYX顺序的欧拉角（以度为单位）  
                euler_angles_deg = rot.as_euler('xyz', degrees=True)  
                cal_cam_pose=[*t_[0],*t_[1],*t_[2],euler_angles_deg[0],euler_angles_deg[1],euler_angles_deg[2]]
                print("cal_cam_pose",cal_cam_pose)
                # print("euler_angles_deg=========",euler_angles_deg)
                # euler_angles = transforms3d.euler.mat2euler(R_board2camera, axes='sxyz')
                # # 如果你需要将弧度转换为角度
                # euler_angles_degrees = np.degrees(euler_angles)
                # print("euler_angles_degrees============",euler_angles_degrees)
                pose_boards.append(cal_cam_pose)
                R_board2cameras.append(R_board2camera)
                t_board2cameras.append(t_board2camera)

                board2cameras_matrix = np.eye(4)
                board2cameras_matrix[:3, :3] = R_board2camera
                board2cameras_matrix[:3, 3] = t_board2camera.reshape(3)
                board2cameras_matrixs.append(board2cameras_matrix)


                with open(path_before_jpg+'.json', 'r') as file:
                    data_loaded = json.load(file)

                    print('end_base_pose',data_loaded['pose'])
                    pose_vectors1.append(data_loaded['pose'])
            
                # 绘制并显示角点
            #    cv2.drawChessboardCorners(img, pattern_size, corners, ret)
            #    cv2.imshow('img', img)
            #    cv2.waitKey(100)
                # print("det_success_num",det_success_num)
        
        #cv2.destroyAllWindows()

        pose_vectors_=np.array(pose_vectors1, dtype=np.float64)
        # 求解手眼标定
        # R_base2end：机械臂基座相对于机械臂末端的旋转矩阵
        # t_base2end：机械臂基座相对于机械臂末端的平移向量
        R_base2ends, t_base2ends = self.pose_vectors_to_base2end_transforms(pose_vectors_)
        # eyetohand.cal_camera_base(pose_vectors1,pose_boards)

        # 选择一种手眼标定的方法，例如 Tsai 法
        # cv2.CALIB_HAND_EYE_TSAI：使用 Tsai-Lenz 算法进行标定。
        # cv2.CALIB_HAND_EYE_PARK：使用 Park-Martin 方法。
        # cv2.CALIB_HAND_EYE_HORAUD：使用 Horaud 方法。
        # cv2.CALIB_HAND_EYE_DANIILIDIS：使用 Daniilidis 方法。
        
        method = cv2.CALIB_HAND_EYE_TSAI
        # R_camera2base：相机相对于机械臂基座的旋转矩阵
        # t_camera2base：相机相对于机械臂基座的平移向量
        R_camera2base, t_camera2base = cv2.calibrateHandEye(R_base2ends, t_base2ends, 
                                                            R_board2cameras, t_board2cameras,method)
        
        # 将旋转矩阵和平移向量组合成齐次位姿矩阵
        T_camera2base = np.eye(4)
        T_camera2base[:3, :3] = R_camera2base
        T_camera2base[:3, 3] = t_camera2base.reshape(3)
        
        # 输出相机相对于机械臂基座的旋转矩阵和平移向量
        print("Camera to base rotation matrix:")
        print(R_camera2base)
        print("Camera to base translation vector:") 
        print(t_camera2base)

        # 输出相机相对于机械臂基座的位姿矩阵
        print("Camera to base pose matrix :")
        np.set_printoptions(suppress=True)  # suppress参数用于禁用科学计数法
        print("方法一RT",T_camera2base)

        # 定义平移矩阵
        # translation_matrix = np.array([
        #     [1, 0, 0, 0],
        #     [0, 1, 0, 0],
        #     [0, 0, 1, 68],
        #     [0, 0, 0, 1]
        # ])

        # -90度转换为弧度
        theta = np.pi / 2

        # 构建绕X轴旋转的矩阵
        rotation_matrix = np.array([
            [1, 0, 0, 0],
            [0, np.cos(theta), -np.sin(theta), -68],
            [0, np.sin(theta), np.cos(theta), 0],
            [0, 0, 0, 1]
        ])

        # 先进行平移，再进行旋转
        # transformed_matrix = np.dot(translation_matrix, T_camera2base)
        T_camera2robotbase = np.dot(rotation_matrix, T_camera2base)
        print("T_camera2robotbase",T_camera2robotbase)
        # print("transformed_matrix",transformed_matrix)
      
        rot_matrix = np.array(T_camera2robotbase[:3,:3])
        # 创建Rotation对象  
        rot = R.from_matrix(rot_matrix)  
        
        # 获取ZYX顺序的欧拉角（以度为单位）  
        euler_angles_deg = rot.as_euler('xyz', degrees=True)  

        # 打印欧拉角  
        print("cv欧拉角xyz(度):", euler_angles_deg)
        T_camera2robotbase=np.array2string(T_camera2robotbase,separator=',')
        print("机器人中心坐标系RT",T_camera2robotbase)
        f=open(str(ROOT)+ '/' +"camera2robotbase0001.txt",'w')
        f.write(str(T_camera2robotbase))
        f.close()

        return T_camera2robotbase

if __name__ == "__main__":

    # intrinsic_cam=[593.7489013671875,593.6917724609375, 639.5, 359.5,1280,720]
    # intrinsic_cam=[599.8084106445312,599.8926391601562, 639.5, 359.5,1280,720]
    intrinsic_cam=[460.3976745605469,460.1457214355469, 320.7341613769531, 242.30792236328125,640,480]

    cam_robot=Camera2RobotOnBase(intrinsic_cam)
    t_camera2base=cam_robot.camera2robotbase()
 
    #print("t_camera2base=\n",t_camera2base)
