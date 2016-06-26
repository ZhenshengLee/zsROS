// Author: Li Zhensheng

#include <proxy/proxy.h>

namespace zs_proxy{
	Proxy::Proxy():
	// TODO:列表初始化
	ac_(NULL),
	{
		ac_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&Proxy::cancelMoveBaseCB, this, _1), false);
		ros::NodeHandle private_nh("~");
		
		// TODO:add
	}
}