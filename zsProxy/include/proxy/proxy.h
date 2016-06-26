// Author: Li Zhensheng
// 两个功能
// 1）接受服务请求，向move_base发送取消
// 2）接收pose消息，重新配置zs_world_frame坐标系的位置
#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include <nav_msgs/Odometry.h>
#include "std_srvs/Empty.h"
#include <iomanip> // for std :: setprecision and std :: fixed
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <termios.h>
#include <unistd.h>
#include <signal.h>

namespace zs_proxy{
    typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionClient;

    class Proxy{
        public:
        Proxy();// TODO:
        
        private:
        bool cancelMoveBaseCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
        bool reconfigParameterCB(const geometry_msgs::Pose& param);
    }
}