import numpy as np
from scipy.spatial.transform import Rotation as R  

def rotation_matrix_z(theta):
    """返回绕 Z 轴旋转 theta（弧度）的旋转矩阵"""
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, 0],
        [np.sin(theta), np.cos(theta), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def rotation_matrix_x(theta):
    """返回绕 X 轴旋转 theta（弧度）的旋转矩阵"""
    return np.array([
        [1, 0, 0, 0],
        [0, np.cos(theta), -np.sin(theta), 0],
        [0, np.sin(theta), np.cos(theta), 0],
        [0, 0, 0, 1]
    ])

def rotation_matrix_y(theta):
    """返回绕 Y 轴旋转 theta（弧度）的旋转矩阵"""
    return np.array([
        [np.cos(theta), 0, np.sin(theta), 0],
        [0, 1, 0, 0],
        [-np.sin(theta), 0, np.cos(theta), 0],
        [0, 0, 0, 1]
    ])

def translation_matrix(dx, dy, dz):
    """返回平移矩阵"""
    return np.array([
        [1, 0, 0, dx],
        [0, 1, 0, dy],
        [0, 0, 1, dz],
        [0, 0, 0, 1]
    ])

def forward_kinematics(theta1, theta2):
    """
    计算从原点到末端执行器的齐次变换矩阵
    :param theta1: 第一个关节的旋转角度（度）
    :param theta2: 第二个关节的旋转角度（度）
    :return: 4x4 齐次变换矩阵
    """
    # 将角度转换为弧度
    theta1 = np.radians(theta1)
    theta2 = np.radians(theta2)

    # 第一个关节的平移和旋转
    T1_translation = translation_matrix(0, 0, 99.15)
    T1_rotation = rotation_matrix_z(theta1)
    T1 = np.dot(T1_translation, T1_rotation)

    # 第二个关节的平移和旋转
    T2_translation = translation_matrix(0, 0, 30.2)
    T2_rotation_x = rotation_matrix_x(np.radians(90))  # 绕 X 轴旋转 +90°
    T2_rotation_y = rotation_matrix_y(theta2)          # 绕 Y 轴旋转 theta2

    # 组合第二个关节的变换矩阵
    T2 = np.dot(np.dot(T2_translation, T2_rotation_x), T2_rotation_y)

    # 计算总的齐次变换矩阵
    T = np.dot(T1, T2)

    return T
# 设置输出选项为不使用科学计数法，并限制小数点后6位
np.set_printoptions(suppress=True, precision=6)
# 计算并打印总的齐次变换矩阵
T_nect2robots= forward_kinematics(0, -10)
print("从原点到末端执行器的齐次变换矩阵为：")
print(T_nect2robots)
T_camera2robotbase=np.array([[ -0.07291181, -0.52170181,  0.85000652,141.28466375],
                                [ -0.99676563,  0.06699779, -0.0443798 ,-10.13281035],
                                [ -0.03379554, -0.8504931 , -0.52489936,203.08081513],
                                [  0.0       ,  0.0        ,  0.0       ,  1.0        ]], dtype=np.float64)

T_camera2robotbase10=np.array( [[ -0.05727836, -0.65517558,  0.75330216,141.12416553],
 [ -0.99828035,  0.02816067, -0.0514132 ,  1.8356761 ],
 [  0.01247118, -0.75495161, -0.6556619 ,174.5785687 ],
 [  0.0       ,  0.0        ,  0.0       ,  1.0        ]], dtype=np.float64)
    
T_camera2nect= np.dot(np.linalg.inv(T_nect2robots),T_camera2robotbase10)
rot_matrix = np.array(T_camera2nect[:3,:3])
# 创建Rotation对象  
rot = R.from_matrix(rot_matrix)  

# 获取ZYX顺序的欧拉角（以度为单位）  
euler_angles_deg = rot.as_euler('xyz', degrees=True)  

# 打印欧拉角  
print("cv欧拉角xyz(度):", euler_angles_deg)
print("T_camera2nect",T_camera2nect)
