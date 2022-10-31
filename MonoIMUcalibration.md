# d435i/d455/l515单目+IMU标定
by fcl 2021.07.28
## 1. IMU标定
1. 修改launch文件
找到realsense-ros包，复制其中的rs_camera.launch，并重命名为rs_camera_l515_imu_calib.launch，并对里面的内容做如下更改
这样做的目的是将accel和gyro的数据合并得到imu话题，如不这样做发布的topic中只有加速计和陀螺仪分开的topic，没有合并的camera/imu topic，并且让accel和gyro都设置成true
`<arg name="unite_imu_method"          default="linear_interpolation"/>`
2. 运行启动文件
`roslaunch realsense2_camera rs_camera_l515_imu_calib.launch`
3. 编写启动文件
进入/software/l515calib/imu_calib_ws/src/imu_utils-master/launch，打开终端运行
`gedit l515_imu_calibration.launch`
在里面写入
```
<launch>

    <node pkg="imu_utils" type="imu_an" name="imu_an" output="screen">
    	<!--TOPIC名称和上面一致-->
        <param name="imu_topic" type="string" value= "/camera/imu"/>
        <!--imu_name 无所谓-->
        <param name="imu_name" type="string" value= "l515"/>
        <!--标定结果存放路径-->
        <param name="data_save_path" type="string" value= "$(find imu_utils)/data/"/>
        <!--数据录制时间-min-->
        <param name="max_time_min" type="int" value= "60"/>
        <!--采样频率-->
        <param name="max_cluster" type="int" value= "100"/>
    </node>

</launch>
```
4. 录制imu数据包
realsense静止放置，放置时间要稍大于l515_imu_calibration.launch中的录制时间，即大于50分钟`rosbag record -O l515imucalib /camera/imu`
其中l515imucalib是bag包的名字，可以更改，录的包在当前终端目录下
/camera/imu是发布的IMU topic，可以通`rostopic list`命令查看
5. 校准程序
包录制好之后，运行校准程序，首先激活imu_util工作空间的setup.bash
`source /home/fcl/software/l515calib/imu_calib_ws/devel/setup.bash`
然后
`roslaunch imu_utils l515_imu_calibration.launch `回放数据包
打开新的终端，cd到存放imu_calibration.bag的路径
```
cd ~/data/dataset/IMU标定录制的大于50min的包
rosbag play -r 200 l515imucalib.bag
```
标定结束后在/home/fcl/software/l515calib/imu_calib_ws/src/imu_utils-master/data中生成许多文件，其中d455_imu_param.yaml就是我们想要的结果

## 2. 相机标定
1. 标定板
下载打印标定板https://github.com/ethz-asl/kalibr/wiki/downloads
下载Aprilgrid 6*6 0.8*0.8m(unscaled)，然后缩放到40%，用A4纸就可以打印出来
原始pdf的格子参数是：
```
6*6的格子
大格子边长：5.5cm
小格子边长：1.65cm
小格子与大格子边长比例：0.3
```
调整后的格子参数是：
```
大格子边长：2.188，一定要自己测量大格子边长，即tagSize
小格子边长：0.66cm
小格子与大格子边长比例：0.3
```

新建april_6x6_A4.yaml文件，我的文件的具体内容/home/fcl/software/d455calib/multicalib_yaml_data/april_6x6_A4.yaml展示如下：
```
target_type: 'aprilgrid' #gridtype
tagCols: 6               #number of apriltags
tagRows: 6               #number of apriltags
tagSize: 0.022           #size of apriltag, edge to edge [m]
tagSpacing: 0.3          #ratio of space between tags to tagSize
```

3. 开始录制.bag文件
第一步：将realsense对准标定板放置
`roslaunch realsense2_camera rs_camera_l515_camera_calib.launch` 
运行`rviz`，fixed frame选择camera_link，之后在里面add rgb的/camera/color/image_raw

第二步：修改相机帧数（官方推荐是4Hz，尽管实际频率不完全准确，但是不影响结果)kalibr在处理标定数据的时候要求频率不能太高，一般为4Hz，我们可以使用如下命令来更改topic的频率，实际上是将原来的topic以新的频率转成新的topic
`rosrun topic_tools throttle messages /camera/color/image_raw 4.0 /color`
查看频率
`rostopic hz /color`
第三步：之后对准标定板，尝试移动realsense，同时要确保标定板一直在三个图像当中。
录制过程参考https://www.youtube.com/watch?v=puNXsnrYWTY&app=desktop
需要科学上网观看
总结下来就是偏航角左右摆动3次，俯仰角摆动3次，滚转角摆动3次，上下移动3次，左右移动3次，前后移动3次，然后自由移动一段时间，摆动幅度要大一点，让视角变化大一点，但是移动要缓慢一点，同时要保证标定板在3个相机视野内部，整个标定时间要在90s以上更好
录制ROS数据包
`rosbag record -O l515cameracalib /color`
后面的topic是转换频率后的topic

4. 使用Kalibr标定
第一步：先激活环境变量
`source software/kalibr_ws/devel/setup.bash `
第二步：运行标定指令
kalibr_calibrate_cameras --target software/l515calib/camera_calib_data/april_6x6_A4.yaml --bag --models pinhole-radtan --topics /color --bag-from-to  --show-extraction

`kalibr_calibrate_cameras --target /home/fcl/software/l515calib/camera_calib_data/april_6x6_A4.yaml --bag  /home/fcl/software/l515calib/camera_calib_data/l515cameracalib.bag --models pinhole-radtan  --topics /color --bag-from-to a b --show-extraction`
其中
april_6x6_A4.yaml是标定板的配置文件
注意如果选择棋格盘，注意targetCols和targetRows表示的是内侧角点的数量，不是格子数量。
multicameras_calibration.bag 是录制的数据包
models pinhole-radtan pinhole-radtan pinhole-radtan表示三个摄像头的相机模型和畸变模型
--topics /infra_left /infra_right /color表示三个摄像头对应的拍摄的数据话题
-–bag-from-to a b表示处理bag中a-b秒的数据
–show-extraction表示显示检测特征点的过程，这些参数可以相应的调整
可以使用rosbag info 来参看录制的包的信息
```
fcl@fcl-ThinkPad-T480:~/software/l515calib/camera_calib_data$ rosbag info l515cameracalib.bag 
path:        l515cameracalib.bag
version:     2.0
duration:    3:05s (185s)
start:       Jul 28 2021 10:30:14.45 (1627439414.45)
end:         Jul 28 2021 10:33:19.49 (1627439599.49)
size:        2.0 GB
messages:    765
compression: none [765/765 chunks]
types:       sensor_msgs/Image [060021388200f6f0f447d0fcd9c64743]
topics:      /color   765 msgs    : sensor_msgs/Image
```

第三步：排查错误1
出现以下报错：cannot import name NavigationToolbar2Wx，如下图所示：
解决办法：发现 matplotlib 中没有NavigationToolbar2Wx 而是换成了NavigationToolbar2WxAgg 所以修改源码，将PlotCollection.py中的NavigationToolbar2Wx换成NavigationToolbar2WxAgg
catkin_make一下
第四步：排查错误2
还有一个报错
```
ImportError: No module named igraph
fcl@fcl-ThinkPad-T480:~$ sudo apt-get install python-igraph
```
第五步：排查错误3
报错找不到焦距，手动输入焦距，400
在终端输入
```
fcl@fcl-ThinkPad-T480:~$ source ~/software/kalibr_ws/devel/setup.bash 
fcl@fcl-ThinkPad-T480:~$ export KALIBR_MANUAL_FOCAL_LENGTH_INIT=1并不是每次都需要
fcl@fcl-ThinkPad-T480:~$ kalibr_calibrate_cameras --target software/l515calib/camera_calib_data/april_6x6_A4.yaml --bag software/l515calib/camera_calib_data/l515cameracalib.bag --models pinhole-radtan --topics /color --bag-from-to 2 112 --show-extraction
```
最终产生3个文件

## 3. 相机和IMU的标定
1. 复制上面双目标定结果产生的camchain-softwarel515calibcamera_calib_datal515cameracalib.yaml文件为chain.yaml

2. 新建一个文件imu.yaml，参考上面imu标定步骤得到的d455_imu_param.yaml，选取其中的
```
   avg-axis:
      gyr_n: 1.8120078954292167e-03
      gyr_w: 1.9468192818602672e-05
   avg-axis:
      acc_n: 1.8783286176747332e-02
      acc_w: 8.8271097542303530e-04
```
最后得到的imu.yaml如下
```
#Accelerometers
accelerometer_noise_density: 2.3177792566996982e+03  #Noise density (continuous-time)
accelerometer_random_walk:   6.3118277723523931e+01   #Bias random walk

#Gyroscopes
gyroscope_noise_density:     2.0702462760023993e-03   #Noise density (continuous-time)
gyroscope_random_walk:       2.3093829433059453e-05  #Bias random walk

rostopic:                    /camera/imu      #the IMU ROS topic
update_rate:                 200.0      #Hz (for discretization of the values above)
```
3. 同样需要用到april_6x6_A4.yaml
4. 复制realsense-ros包中rs_camera.launch，重命名为rs_camera_imucamcalib.launch，更改内容如下
时间对齐`<arg name="enable_sync"               default="true"/>`
合并加速计和陀螺仪的topic`<arg name="unite_imu_method"          default="linear_interpolation"/>`
5. 启动realsense
`roslaunch realsense2_camera rs_camera_imucamcalib.launch`
6. 查看默认的频率
设置camera是4hz，imu是200hz，45s就行了
```
fcl@fcl-ThinkPad-T480:~$ rostopic hz /camera/imu
subscribed to [/camera/imu]
average rate: 199.887
	min: 0.004s max: 0.010s std dev: 0.00042s window: 189
average rate: 199.886
	min: 0.000s max: 0.010s std dev: 0.00056s window: 389
average rate: 199.884
	min: 0.000s max: 0.010s std dev: 0.00059s window: 589
^Caverage rate: 199.973
	min: 0.000s max: 0.010s std dev: 0.00058s window: 653
fcl@fcl-ThinkPad-T480:~$ rostopic hz /camera/color/image_raw
subscribed to [/camera/color/image_raw]
average rate: 30.058
	min: 0.031s max: 0.035s std dev: 0.00064s window: 28
average rate: 30.035
	min: 0.029s max: 0.046s std dev: 0.00195s window: 58
average rate: 30.023
	min: 0.029s max: 0.046s std dev: 0.00164s window: 88
^Caverage rate: 30.046
	min: 0.027s max: 0.046s std dev: 0.00206s window: 102
fcl@fcl-ThinkPad-T480:~$ 
```
7. 打开rviz，add imu topic和color topic，同时调整realsense位置，要确保一直包含标定板全部内容
8. 调整imu和color的发布频率以及以新的topic名发布它们，其中图像的发布频率改为20Hz，imu发布频率改为200Hz
```
rosrun topic_tools throttle messages /camera/color/image_raw 20.0 /color
```
这种调整频率的方式只是理想结果，通过rostopic hz topic名可以查看实际的频率，可以发现实际频率和设置的频率并不一定相同
9. 和上面一样开始采集数据包
`rosbag record -O l515cameraimucalib.bag /camera/imu /color`
查看bag包内容
```
fcl@fcl-ThinkPad-T480:~/software/l515calib/camera_imu_calib_data$ rosbag info l515cameraimucalib.bag 
path:        l515cameraimucalib.bag
version:     2.0
duration:    1:54s (114s)
start:       Jul 26 2021 10:19:56.93 (1627265996.93)
end:         Jul 26 2021 10:21:51.51 (1627266111.51)
size:        6.0 GB
messages:    25229
compression: none [2331/2331 chunks]
types:       sensor_msgs/Image [060021388200f6f0f447d0fcd9c64743]
             sensor_msgs/Imu   [6a62c6daae103f4ff57a132d6f95cec2]
topics:      /camera/imu   22899 msgs    : sensor_msgs/Imu  
             /color         2330 msgs    : sensor_msgs/Image

```
10. 开始标定
相应参数需要相应更改，target.yaml对应april_6x6_A4.yaml文件
```
source software/kalibr_ws/devel/setup.bash

kalibr_calibrate_imu_camera --bag /home/fcl/software/d455calib/imu_stereo_data/imu_stereo.bag --cam /home/fcl/software/d455calib/imu_stereo_data/chain.yaml --imu /home/fcl/software/d455calib/imu_stereo_data/imu.yaml --target /home/fcl/software/d455calib/multicalib_yaml_data/april_6x6_A4.yaml --bag-from-to 3 125 --show-extraction
```
最终得到的结果为是得打yaml，txt，和pdf文件
标定结果的好坏可以看results-imucam-homezjimu_stereo.txt中的重投影误差Reprojection error，两个相机都在0.15以下说明标定的结果比较好
