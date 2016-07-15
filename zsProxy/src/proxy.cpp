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
#include <std_msgs/Bool.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseActionClient;

class ProxyNode{
    public:
        ProxyNode(ros::NodeHandle n);
    // TODO:
    private:
        bool cancelGoalCB(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);
        void reconfigParameterCB(const geometry_msgs::Pose& param);
		void sendGoalCB(const geometry_msgs::Pose& goal);
		void canelGoalMsgCB(const std_msgs::Bool trigger);

        MoveBaseActionClient* ac_;
        ros::Subscriber zs_pose_sub_;
		ros::Subscriber zs_goal_sub_;
        ros::ServiceServer cancel_goal_srv_;
        ros::NodeHandle nh_;
        double start_pose_x_;
        double start_pose_y_;
        double start_pose_th_;
        std::string start_pose_x_str_;
        std::string start_pose_y_str_;
        std::string start_pose_th_str_;
		move_base_msgs::MoveBaseGoal goal_;
		ros::Subcriber cancel_goal_sub_;
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
	zs_goal_sub_ = nh_.subscribe<geometry_msgs::Pose>("zs_goal", 1, (boost::function <void(const geometry_msgs::Pose)>)boost::bind(&ProxyNode::sendGoalCB, this, _1 ));
	cancel_goal_sub_ = nh_.subscribe<std_msgs::Bool>("zs_cancel", 1, (boost::function <void(const geometry_msgs::Pose)>)boost::bind(&ProxyNode::cancelGoalMsgCB, this, _1 ));
}
bool ProxyNode::cancelGoalCB(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp){
	ROS_INFO("cancelGoalCB");
	ac_->cancelGoal();
	return true;
}
void canelGoalMsgCB(const std_msgs::Bool trigger){
	if(trigger){
		ROS_INFO(cancelGoalMsgCB);
		ac_->cancelGoal();
	}
}
void ProxyNode::sendGoalCB(const geometry_msgs::Pose& goal){
	goal_.target_pose.header.frame_id = "zsworld_frame";//odom
  	goal_.target_pose.header.stamp = ros::Time::now();
	goal_.target_pose.pose = goal;
  	ROS_INFO("Sending goal");
  	ac_->sendGoal(goal_);
}
void ProxyNode::reconfigParameterCB(const geometry_msgs::Pose& param){
	// 将double转为string，在C++11中可以应用新标准，也可以使用boost库，此处使用C++03标准
	// C++11:	std::string varAsString = std::to_string(myDoubleVar);
	// boost:	std::string str = boost::lexical_cast<std::string>(dbl);
	ROS_INFO("reconfigParameterCB...");
	std::stringstream ss_x, ss_y, ss_th;
	ss_x << param.position.x;
	ss_x >> start_pose_x_str_;
	ss_y << param.position.y;
	ss_y >> start_pose_y_str_;
	ss_th << tf::getYaw(param.orientation);
	ss_th >> start_pose_th_str_;
	// ROS_INFO(start_pose_x_str_.c_str());
	// ROS_INFO(start_pose_y_str_.c_str());
	// ROS_INFO(start_pose_th_str_.c_str());
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
