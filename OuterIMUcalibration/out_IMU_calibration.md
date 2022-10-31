# Ubuntu配置外置IMU HI226
by fcl 2021.08.03
## 1. 查看设备
插上设备
`fcl@fcl-ThinkPad-T480:~$ ls /dev/ttyUSB*`
或者`cd /dev`再`ls`
可以看到ttyUSB0就是我们的IMU

## 2. 非官方ROS驱动sdk驱动设备
### 2.1 驱动设备
在github上找到一个可以驱动HI226的sdk
https://github.com/TzuChiehHung/hipnuc_imu
放在/home/fcl/software/outIMUcalib/out_IMU_ws/src下
然后在out_IMU_ws下`catkin_make`编译
用法
`source software/outIMUcalib/out_IMU_ws/devel/setup.bash`
`sudo chmod 666 /dev/ttyUSB0`
`roslaunch hipnuc_imu imu.launch`
查看topic
`rostopic list`
可以看到/imu
查看频率
`rostopic hz /imu`
可以看到100hz
### 2.2 标定设备
标定过程和realsense的IMU calibration一样
`source software/outIMUcalib/out_IMU_ws/devel/setup.bash`
`sudo chmod 666 /dev/ttyUSB0`
`roslaunch hipnuc_imu imu.launch`
`rosbag record -O outimucalib /imu`
`source software/l515calib/imu_calib_ws/devel/setup.bash`
`roslaunch imu_utils out_imu_calibration.launch`
`rosbag play -r 100 outimucalib.bag`

## 3 张昊发我的官方文档
### 3.1 驱动设备
#### 3.1.1 WIN下修改频率
examples里面有一个Ubuntu，它的Readme.md里面有介绍修改波特率来适配高频率输出的介绍，
简单来说就是，可以在其所在目录下的main.c修改波特率来更改频率，超过100hz就需要改了

/home/fcl/software/outIMUcalib/out_IMU_data里有官方文档这个文件夹，里面有hi226umcn.pdf的文档和products-master软件，这个pdf讲述了如何去修改波特率和输出hz，这个master文件夹里有官方给的ROS的启动源码
/home/fcl/software/outIMUcalib/out_IMU_data/官方文档/products-master/examples/ROS，在这个路径下，文档Readme.md，讲述了如何创建工作空间和启动imu节点，但是首先要去windows里将输出hz改成200，windows的设置如下：
官网https://hipnuc.com/HI226_229.html
1、下载driver
hipnuc-products-products-master\hipnuc-products-products-master\usb_uart_drivers\win\CP2104\CP210xVCPInstaller_x64.exe
2. 运行监控
然后打开F:\Chromedownload\监控软件\CHCenter_win\CHCenter.exe
点击设备设置
首先AT+BAUD=921600，send
其次AT+ODR=200，send
![avatar](设置串口波特率和输出赫兹.png)
#### 3.1.2 ubuntu下创建工作空间-编译-启动节点
1. ws创建和编译
创建新的工作空间out_imu_200hz_ws
创建src
将/home/fcl/software/outIMUcalib/out_IMU_data/官方文档/products-master/examples/ROS/serial_imu_ws/src下的两个文件放入src文件夹，serial_imu和imu_launch，并回到out_imu_200hz_ws路径下，打开终端，catkin_make

2. 启动节点
`fcl@fcl-ThinkPad-T480:~$ roscore`
`fcl@fcl-ThinkPad-T480:~$ sudo chmod 777 /dev/ttyUSB0`
`fcl@fcl-ThinkPad-T480:~$ source software/outIMUcalib/out_imu_200hz_ws/devel/setup.bash`
`fcl@fcl-ThinkPad-T480:~$ rosrun serial_imu serial_imu`
可以看到输出的hz
`rostopic list`
可以看到输出的IMU的topic叫IMU_data

### 3.2 标定设备
200hz输出频率修改好之后标定外置IMU
#### 3.2.1 录制IMU信息
按照上面的方法启动IMU节点后，确保是200hz，开始录制
`fcl@fcl-ThinkPad-T480:~$ rosbag record -O hi226imucalib /IMU_data`
#### 3.2.2 录制完毕后开始标定
1. 修改标定文件
打开/home/fcl/software/outIMUcalib/imu_calib_ws/src/imu_utils-master/launch/out_imu_calibration.launch文件，修改成如下内容
```
<launch>

    <node pkg="imu_utils" type="imu_an" name="imu_an" output="screen">
    	<!--TOPIC名称和上面一致-->
        <param name="imu_topic" type="string" value= "/IMU_data"/>
        <!--imu_name 无所谓-->
        <param name="imu_name" type="string" value= "HI226"/>
        <!--标定结果存放路径-->
        <param name="data_save_path" type="string" value= "$(find imu_utils)/data/"/>
        <!--数据录制时间-min-->
        <param name="max_time_min" type="int" value= "60"/>
        <!--采样频率，即是IMU频率，采样频率可以使用rostopic hz /camera/imu查看，设置为200，波特率没改，最多是100，为后面的rosbag play播放频率-->
        <param name="max_cluster" type="int" value= "200"/>
    </node>
    
</launch>

```
2. 然后运行标定的节点
```
fcl@fcl-ThinkPad-T480:~$ source software/outIMUcalib/imu_calib_ws/devel/setup.bash 
fcl@fcl-ThinkPad-T480:~$ roslaunch imu_utils out_imu_calibration.launch
```
3. 重播IMU的bag包
`fcl@fcl-ThinkPad-T480:~$ rosbag play -r 200 hi226imucalib.bag`
标定结束后可以得到.yaml文件
/home/fcl/software/outIMUcalib/imu_calib_ws/src/imu_utils-master/data/HI226_imu_param.yaml把它复制到/home/fcl/software/outIMUcalib/out_IMU_data/路径下
这就是外置IMU单独标定的结果
```
%YAML:1.0
---
type: IMU
name: HI226
Gyr:
   unit: " rad/s"
   avg-axis:
      gyr_n: 1.3605048143002939e-03
      gyr_w: 1.0753178549792385e-05
   x-axis:
      gyr_n: 7.9608186362475648e-04
      gyr_w: 6.5933908502289326e-06
   y-axis:
      gyr_n: 2.4623504770671513e-03
      gyr_w: 1.9831800234293994e-05
   z-axis:
      gyr_n: 8.2308210220897343e-04
      gyr_w: 5.8343445648542239e-06
Acc:
   unit: " m/s^2"
   avg-axis:
      acc_n: 2.7333046656311814e-02
      acc_w: 3.6715055966953723e-04
   x-axis:
      acc_n: 2.3517717692426342e-02
      acc_w: 1.7602241420464871e-04
   y-axis:
      acc_n: 2.5914290095789345e-02
      acc_w: 5.8202044758073250e-04
   z-axis:
      acc_n: 3.2567132180719757e-02
      acc_w: 3.4340881722323040e-04
```