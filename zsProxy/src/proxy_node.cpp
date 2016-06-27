//Author: Li Zhensheng

# include "proxy.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "zs_proxy_node");
    zs_proxy::Proxy proxy();
    ros::spin();

    return 0;
}