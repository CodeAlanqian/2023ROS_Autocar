#Racecar

ROS racecar  
************************安装************************  

配置小车串口udev：  
cd  ~/racecar/src/racecar/udev  
sudo  bash racecar_init.sh 
sudo reboot  
**********************建立地图**********************  
先安装电脑用户名和主机名配置主从机  
a) 运行车  
roslaunch racecar Run_car.launch  
b) 3.3运行gmapping  
roslaunch racecar Run_gmapping.launch  
c) 3.4运行键盘控制  
rosrun racecar racecar_teleop.py  
或者手柄遥控  
roslaunch racecar teleop_joy.launch    
d) 3.6 建立地图  
键盘控制建立地图,按键如下：  
U	I 	O  
J	K	L  
M	, 	.  
加减速为W，S.  
手柄控制：  
L侧方向键:上下为加速减速，左右为转向幅度大小调节  
R侧摇杆：控制小车运动  
e) 保存地图（地图直接保存在小车上）  
在racecar文件夹下执行：bash save_map.sh  
地图保存在racecar/map/mymap.pgm  


************************导航************************  

a) 运行车  
roslaunch racecar Run_car.launch  
b) 运行AMCL  
roslaunch racecar amcl_nav.launch  
c) 4.5 开始导航   
在RVIZ中设定初始坐标，设定目标位置，开始导航  
*********************软件接口***********************  
1.启动底盘  
	启动底盘需要启动rosserial_python节点。  
	设置参考racecar/launch/Run_car.launch  
2.发布地盘控制指令：  
	通过发布Twist消息控制底盘。  
	线速度：twist.linear.x,这里的线速度范围为500～2500（对应PWM脉冲为0.5ms～2.5ms）,1500为静止，1500-2500为正向速度，500-1500为反向速度。  
	角速度：twist.angular.x，这里角度范围为0～180度,90度为中间值，90-180度左转，0-90度右转。  
3.里程计数据：  
	里程计采用激光雷达和IMU数据融合的里程计。  
	需要先启动IMU节点和雷达节点，参考racecar/launch/Run_car.launch  
	然后启动rf2o节点，用rf2o生成激光里程计，参考racecar/launch/includes/rf2o.launch.xml  
	再启动robot_localization用EKF融合里程计信息，参考racecar/launch/Run_gmapping.launch  
	


						
	
