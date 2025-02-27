# kepler5_driver_calib功能包手眼标定
1、概述
在机器人身上安装好标定板。利用奥比相机提供rgb图、深度图的rostopic提取标定板上的相机坐标系值。
（1）订阅奥比相机的节点如下：
/camera/color/image_raw #订阅肇观相机rgb图
/camera/depth/image_raw #订阅肇观相机深度图

2、功能实现与对外接口含义说明
<!-- 2.1 标定数据采集 -->
启动相机节点： ros2 launch orbbec_camera gemini_330_series.launch.py
启动采集数据节点：python3 /home/kepler/ros2_ws/src/kepler5_driver_calib/kepler5_driver_calib/calibCam2Base3world_old.py 
机械臂启动运行，
按键盘“y”键，采集不同位置图片和相应机械臂末端姿态9张，保存在calimageworld20文件中，（注意：机械臂末端姿态数据由张小龙提供）；
按键盘“q”键，退出程序,标定出结果camera2robotbase20.txt。
<!-- 2.2 启动标定
（1）打开ros2 topic echo /camera/color/camera_info，获取相机内参；
（2）内参手动输入eyetohand4.py文件中，示例：修改文件最后的代码行 intrinsic_cam=[624.778564453125,624.3175659179688, 639.5, 359.5,1280,720]
（3）运行：python3 eyetohand4.py
（4）输出标定结果：camera2robotbase_cv.txt
2.3 标定结束 -->