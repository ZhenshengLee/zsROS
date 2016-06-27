// Author: Li Zhensheng

#include "proxy.h"

namespace zs_proxy{
	Proxy::Proxy():
	// TODO:列表初始化
	ac_(NULL),
	start_pose_x_(0.0),
    start_pose_y_(0.0),
    start_pose_th_(0.0),
	start_pose_x_str_(""),
	start_pose_y_str_(""),
	start_pose_th_str_("")
	{
		ac_ = new MoveBaseActionClient("move_base", false);
		ros::NodeHandle nh_;
		cancel_goal_srv_ = nh_.advertiseService("zs/cancel_goal", &Proxy::cancelGoalCB, this);
		zs_pose_sub_ = nh_.subscribe<geometry_msgs::Pose>("zs/zs_pose", 1, (boost::function <void(const geometry_msgs::Pose)>)boost::bind(&Proxy::reconfigParameterCB, this, _1 ));
	}

	bool Proxy::cancelGoalCB(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp){
		ac_->cancelGoal();
		return true;
	}
	void Proxy::reconfigParameterCB(const geometry_msgs::Pose param){
		// 将double转为string
		std::stringstream ss;
		ss << param.position.x;
		ss >> start_pose_x_str_;
		ss << param.position.y;
		ss >> start_pose_y_str_;
		ss << tf::getYaw(param.orientation);
		ss >> start_pose_th_str_;
		system(("rosrun dynamic_reconfigure dynparam set RosAria zsstart_pose_x " + start_pose_x_str_).c_str());
		system(("rosrun dynamic_reconfigure dynparam set RosAria zsstart_pose_y " + start_pose_y_str_).c_str());
		system(("rosrun dynamic_reconfigure dynparam set RosAria zsstart_pose_th "+ start_pose_th_str_).c_str());
	}
};
