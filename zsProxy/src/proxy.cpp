// Author: Li Zhensheng

#include <proxy/proxy.h>

namespace zs_proxy{
	Proxy::Proxy():
	// TODO:列表初始化
	ac_(NULL),
	start_pose_x_(0.0);
    start_pose_y_(0.0);
    start_pose_th_(0.0);
	start_pose_x_str_(""),
	start_pose_y_str_(""),
	start_pose_th_str_("")
	{
		ac_ = new MoveBaseActionClient(ros::NodeHandle(), "move_base", boost::bind(&Proxy::cancelGoalCB, this, _1), false);
		ros::NodeHandle nh_;
		cancel_goal_srv_ = nh_.advertiseService("cancel_goal", &Proxy::cancelGoalCB, this);
		zs_pose_sub_ = nh_.subscribe<geometry_msgs::Pose>("zs_pose",1);
		// TODO:add
	}

	Proxy::cancelGoalCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp){
		ac_->cancelGoal();
		return true;
	}
	Proxy::reconfigParameterCB(const geometry_msgs::Pose& param){
		// 将double转为string
		std::stringstream ss;
		ss << param.position.x >> start_pose_x_str_;
		ss << param.position.y >> start_pose_y_str_;
		ss << tf::getYaw(param.orientation) >> start_pose_th_str_;
		system("rosrun dynamic_reconfigure dynparam set RosAria zsstart_pose_x " + start_pose_x_str_);
		system("rosrun dynamic_reconfigure dynparam set RosAria zsstart_pose_y " + start_pose_y_str_);
		system("rosrun dynamic_reconfigure dynparam set RosAria zsstart_pose_th "+ start_pose_th_str_);
		return true;
	}
}
// int main(){  
// 　　double   d=123.456;   
// 　　string   str;   
// 　　stringstream   ss;   
// 　　ss<<d;   
// 　　ss>>str;  
// tf::getYaw(const geometry_msgs::Quaternion & msg_q);
// tf::createQuaternionMsgFromYaw(zsstart_pose_th*M_PI/180);
// std::stringstream bumper_info(std::stringstream::out);