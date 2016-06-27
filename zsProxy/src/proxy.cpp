// Author: Li Zhensheng

#include <proxy/proxy.h>

namespace zs_proxy{
	Proxy::Proxy():
	// TODO:列表初始化
	ac_(NULL),
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
		param.
		system("rosrun dynamic_reconfigure dynparam set RosAria zsstart_pose_x 3");
		system("rosrun dynamic_reconfigure dynparam set RosAria zsstart_pose_y 3");
		system("rosrun dynamic_reconfigure dynparam set RosAria zsstart_pose_th 3");
		

	}
}
// int main(){  
// 　　double   d=123.456;   
// 　　string   str;   
// 　　stringstream   ss;   
// 　　ss<<d;   
// 　　ss>>str;  