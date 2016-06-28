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

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseActionClient;

class ProxyNode{
    public:
        ProxyNode(ros::NodeHandle n);
    // TODO:
    private:
        bool cancelGoalCB(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);
        void reconfigParameterCB(const geometry_msgs::Pose& param);
        // void doubletostring(const double data);
        MoveBaseActionClient* ac_;
        ros::Subscriber zs_pose_sub_;
        ros::ServiceServer cancel_goal_srv_;
        ros::NodeHandle nh_;
        double start_pose_x_;
        double start_pose_y_;
        double start_pose_th_;
        std::string start_pose_x_str_;
        std::string start_pose_y_str_;
        std::string start_pose_th_str_;
};

ProxyNode::ProxyNode(ros::NodeHandle n):
	// TODO:列表初始化
	nh_(n),
	ac_(NULL),
	start_pose_x_(0.0),
	start_pose_y_(0.0),
	start_pose_th_(0.0),
	start_pose_x_str_(""),
	start_pose_y_str_(""),
	start_pose_th_str_("")
{
	// ROS_INFO("zs_proxy constructing...");
	ac_ = new MoveBaseActionClient("move_base", false);
	
	cancel_goal_srv_ = nh_.advertiseService("cancel_goal", &ProxyNode::cancelGoalCB, this);
	zs_pose_sub_ = nh_.subscribe<geometry_msgs::Pose>("zs_pose", 1, (boost::function <void(const geometry_msgs::Pose)>)boost::bind(&ProxyNode::reconfigParameterCB, this, _1 ));
	// zs_pose_sub_ = nh_.subscribe("zs_pose", 1, &ProxyNode::reconfigParameterCB);
}
bool ProxyNode::cancelGoalCB(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp){
	ROS_INFO("cancelGoalCB");
	ac_->cancelGoal();
	return true;
}
void ProxyNode::reconfigParameterCB(const geometry_msgs::Pose& param){
	// 将double转为string
	ROS_INFO("reconfigParameterCB...");
	std::stringstream ss;
	ss << param.position.x;
	ss >> start_pose_x_str_;
	ss << param.position.y;
	ss >> start_pose_y_str_;
	ss << tf::getYaw(param.orientation);
	ss >> start_pose_th_str_;
	std::system(("rosrun dynamic_reconfigure dynparam set RosAria zsstart_pose_x " + start_pose_x_str_).c_str());
	std::system(("rosrun dynamic_reconfigure dynparam set RosAria zsstart_pose_y " + start_pose_y_str_).c_str());
	std::system(("rosrun dynamic_reconfigure dynparam set RosAria zsstart_pose_th "+ start_pose_th_str_).c_str());
}

int main(int argc, char** argv){
    ros::init(argc, argv, "zsProxy");
	ros::NodeHandle n(std::string("~"));
    ProxyNode proxy(n);
    // ROS_INFO("zs: zs_proxy ini...");
    ros::spin();

    return 0;
}
