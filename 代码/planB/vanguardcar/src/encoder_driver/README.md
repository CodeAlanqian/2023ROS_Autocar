## Encoder ##
#### 包内有两个代码文件 .\src\Encoder_vel.py .\src\imu_encoder_mix.cpp
##### Encoder_vel.py 
负责串口的打开与数据读取发布,主要参数为：port baud k
racecar\launch\Run_car.launch中已经写入了该脚本，port 与 baud都不需要修改（新车需要绑定串口）
k是滑移系数，需要在使用前进行标定，即让车行走1m，观察里程计话题(/encoder_imu_odom)的输出量为a，则k=1/a

##### imu_encoder_mix.cpp
IMU 与编码器的融合节点，订阅的话题为/imu_data /encoder ，输出话题为/encoder_imu_odom， 如果里程计不工作先排查两个输入的话题是否正常发值。
encoder_driver\launch\wheel_odom.launch 该文件可以单独启动轮式里程计
