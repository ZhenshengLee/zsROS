//zs: publish a simple goal msg to the node move_base
#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include <geometry_msgs/PoseStamped.h>
#include "tf/tf.h"
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_SPACE 0x20

class zsGoalPubRosAria
{
public:
    zsGoalPubRosAria();
    void keyLoop();
private:
    ros::NodeHandle nh_;
    double x_offset_;
    ros::Publisher pose_pub_;
    ros::ServiceClient cancel_goal_srv_client_;
    std_srvs::Empty srv_;
    ros::Publisher zs_pose_pub_;
    geometry_msgs::Pose pose_;
};
zsGoalPubRosAria::zsGoalPubRosAria():
        x_offset_(1.0),
        srv_(),
{
    nh_.param("x_offset", x_offset_, x_offset_);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
    cancel_goal_srv_client_ = nh_.serviceClient<std_srvs::Empty>("zs/cancel_goal");
    zs_pose_pub_ = nh_.advertise<geometry_msgs::Pose>("zs/zs_pose", 1);
}
int kfd = 0;
struct termios cooked, raw;
void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "zsGoalPub_RosAria");
    zsGoalPubRosAria zsGoalPubRosAria;
    signal(SIGINT,quit);
    zsGoalPubRosAria.keyLoop();
    return(0);
}
void zsGoalPubRosAria::keyLoop()
{
    char c;
    bool dirty=false;
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the robot.");
    puts("ONLY support right arrows");
//    puts("Press the space bar to stop the robot.");
    puts("Press q to stop the program");
    for(;;)
    {
        // get the next event from the keyboard
        if(read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }
//        linear_=angular_=0;
        ROS_DEBUG("value: 0x%02X\n", c);
        switch(c)
        {
        //    case KEYCODE_L:
        //         ROS_DEBUG("LEFT");
        //        break;
            case KEYCODE_R:
                ROS_DEBUG("RIGHT");
//                x_offset_ += 0.5;
                x_offset_ = 1.0;
                ROS_INFO_STREAM("You push the RIGHT Arrow!");
                dirty = true;
                break;
//            case KEYCODE_U:
//
//                break;
            case KEYCODE_D:
                pose_.position.x = 3;
                pose_.position.y = 3;
                tf::quaternionTFToMsg(tf::createQuaternionFromYaw(90*M_PI/180), pose_.orientation);
                break;
            case KEYCODE_SPACE:
                ROS_INFO("zs: You push the SPACE button!");
                if (cancel_goal_srv_client_.call(srv))
                {
                    ROS_INFO("zs: send cancel_goal service!");
                }
                break;
            case KEYCODE_Q:
                ROS_DEBUG("QUIT");
                ROS_INFO_STREAM("You quit the program successfully");
                return;
                break;
        }
        geometry_msgs::PoseStamped zsGoal;
        zsGoal.header.stamp = ros::Time::now();
        zsGoal.header.frame_id = "odom";
        zsGoal.pose.position.x = x_offset_;
        zsGoal.pose.position.y = 0.0;
        zsGoal.pose.position.z = 0.0;
        //zs:
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(0.0*M_PI/180), zsGoal.pose.orientation);
//        twist.angular.z = a_scale_*angular_;
//        twist.linear.x = l_scale_*linear_;
        if(dirty == true)//
        {
            pose_pub_.publish(zsGoal);
            ROS_INFO("zs: published a move_base_simple/goal msg and offset equals to: %lf", x_offset_);
            dirty=false;
        }
    }
    return;
}
