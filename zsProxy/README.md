### 参考

[ROS WIKI](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)

### 前言

* 因为windows平台的ROS for LV 没有显式支持action，所以不能直接给move_base节点发送move_base/cancel (actionlib_msgs/GoalID)，故在ROS端添加一个Proxy来代理解决

* 该代理的功能还包括ROSARIA和move_base的reconfigure，所以要从命令行运行重新配置命令

* 还修正一个ROSARIA发送的pose_msg内容为相对坐标的问题，改为发送相对于zsWorld坐标系的绝对坐标（在rosaria包中完成）

### 工作

0. ROSARIA坐标修正，并制作一个reconfiguration_server（在rosaria包中完成）

1. 接收上级节点发送的取消service_call，转发给move_base取消action_msg

2. 接收来自上级节点的move_base reconfigure_msg，运行命令行进行重新配置

3. 接收来自上级节点的rosaria reconfigure_msg，运行命令行进行重新配置

4. 接收来及上级节点的重新定位的msg，运行命令行重新配置tf，主要是zsWorld与odom的坐标系转换关系

5. 确认ROS for LV与ROS的连接关系

6. 用rosaria_client进行测试