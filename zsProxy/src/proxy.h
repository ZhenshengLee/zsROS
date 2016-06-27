// Author: Li Zhensheng
// 两个功能
// 1）接受服务请求，向move_base发送取消
// 2）接收pose消息，重新配置zs_world_frame坐标系的位置
#include <string>
#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include <nav_msgs/Odometry.h>
#include "std_srvs/Empty.h"
#include <iomanip> // for std :: setprecision and std :: fixed
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "tf/tf.h"
#include <termios.h>
#include <unistd.h>
#include <signal.h>
#include <sstream>

namespace zs_proxy{
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseActionClient;

    class Proxy{
        public:
            Proxy();
        // TODO:
        private:
            bool cancelGoalCB(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);
            void reconfigParameterCB(const geometry_msgs::Pose param);
            // void doubletostring(const double data);
            MoveBaseActionClient* ac_;
            ros::Subscriber zs_pose_sub_;
            ros::ServiceServer cancel_goal_srv_;
            double start_pose_x_;
            double start_pose_y_;
            double start_pose_th_;
            std::string start_pose_x_str_;
            std::string start_pose_y_str_;
            std::string start_pose_th_str_;
    };
};