#include "ros/ros.h"
#include <iostream>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>

#include <geometry_msgs/Pose.h>

#include <std_msgs/Float64.h>

#include <cyton_actin_ros_driver/cyton_hardware_manager.h>
#include <cyton_actin_ros_driver/CytonInterfaceConfig.h>
#include <boost/thread.hpp>//Need for to_string

#include <tf/transform_broadcaster.h>

using namespace std;

CytonHardwareManager *manager;

ros::Subscriber joystick_sub;//Subscribe to joystick(3d mouse)
ros::Subscriber ee_pose_sub;//Subscribe to end effector pose
ros::Subscriber ee_gripper_sub;//Subscribe to gripper commands
ros::Subscriber marker_pose_sub;//Subscribe to marker position
ros::Publisher joint_state_pub;//Publish the arm joint angles

enum MarkerTrackingState {MOVE_TO_OBJECT, MOVE_TO_GRIP, GRIPPING};
MarkerTrackingState marker_tracking_state = MOVE_TO_OBJECT;

double bounds[6] = {0.05, 0.05, 0.05, 2*M_PI/180.0, 2*M_PI/180.0, 2*M_PI/180.0};
double ibounds[6] = {1,1,1,1,1,1};
bool joystick_enabled = false;///< Used for knowing if joystick_sub is enabled so that we can shutdown the subscriber when not using
bool ee_pose_enabled = false;///< Used to know whether external pose subscribing is enable or not
bool marker_pose_enabled = false;///< Used to know whether marker pose subscribing is enable or not
double joint_publish_rate = 20;///< Publish Frequency in Hz
EcOrientation gripper_offset_orientation, gripper_offset_orientation_inverse;///< Transformation to make forward of gripper(x), left of gripper(y), up of gripper(z)
EcOrientation base_offset_orientation;///< Offset for flipping the base (Orientation of the new fram wrt to the original frame)
EcCoordinateSystemTransformation camera_offset_transform;///< Offset of camera wrt quad base
EcCoordinateSystemTransformation base_offset_transform, base_offset_transform_inverse;///< Offset of arm base wrt quad base
sensor_msgs::JointState joint_state;///< Joint angles of the arm
EcRealVector home_joint_positions;///< Default Joint Angles to go to home config
bool publish_velocities = false, publish_torques = false;///< Status flags for publishing joint velocities and torques. Joint angles are always published


/**
* Maps i between (-ibound,ibound) to (-bound, bound)
*/
inline double bind(double i, double bound, double ibound=1)
{
  //assert(i<=ibound);
  //assert(i>=-ibound);//May not be needed
  return i*(bound)/(ibound);
}

/**
 * Returns i rounded by N number of decimal places where
 * N = log10(product). For example N = 2 => product = 100
 */
inline double ROUND(double i, double product)
{
  return round(i*product)/product;
}

void publishTF(EcCoordinateSystemTransformation &transformation)
{
  static tf::TransformBroadcaster broadcaster;///< Broadcast tf for goal 
  const EcVector &position = transformation.translation();
  const EcOrientation &orientation = transformation.orientation();
  tf::Transform tf_transformation_;
  tf_transformation_.setOrigin(tf::Vector3(position[0],position[1],position[2]));
  tf_transformation_.setRotation(tf::Quaternion(orientation[1],orientation[2],orientation[3], orientation[0]));
  broadcaster.sendTransform(tf::StampedTransform(tf_transformation_, ros::Time::now(), "/base_link", "desired_end_effector_pose"));
}


void joyCallback(sensor_msgs::Joy::ConstPtr joy_msg)
{

  //If button 0 or 1 is pressed set command to gripper:
  if(joy_msg->buttons[0] || joy_msg->buttons[1])
  {
    EcCoordinateSystemTransformation gripper_pose;
    manager->getGripperPose(gripper_pose);
    EcReal gripper_position = gripper_pose.translation()[2];
    EcReal delta_gripper_position;///< Delta of gripper position Make this a parameter if needed
    if(joy_msg->buttons[0] == 1)
    {
      delta_gripper_position = 0.0005;
      if(gripper_position + delta_gripper_position < manager->arm_joint_limits_[7].upper_limit_)
        gripper_position += delta_gripper_position;
      gripper_pose.setTranslationZ(gripper_position);
      manager->commandGripper(gripper_pose);
    }
    else if(joy_msg->buttons[1] == 1)
    {
      delta_gripper_position = -0.0005;
      if(gripper_position + delta_gripper_position > manager->arm_joint_limits_[7].lower_limit_)
        gripper_position += delta_gripper_position;
      gripper_pose.setTranslationZ(gripper_position);
      manager->commandGripper(gripper_pose);
    }
    ROS_INFO("Gripper Position: %f",gripper_position);

    EcCoordinateSystemTransformation current_pose;
    manager->getEndEffectorPose(current_pose);
    manager->commandEndEffector(current_pose);
    publishTF(current_pose);//publish tf:

    return;
  }
  


  /////Arm Control:
  double delta_transform[6];
  EcCoordinateSystemTransformation current_pose;
  manager->getEndEffectorPose(current_pose);
  const EcOrientation &current_orientation = current_pose.orientation();
  const EcVector &current_position = current_pose.translation();

  for(int i = 0; i < 6; i++)
  {
    delta_transform[i] = bind(joy_msg->axes[i],bounds[i], ibounds[i]);
  }

  EcCoordinateSystemTransformation resulting_pose;

  EcOrientation delta_orientation;
  delta_orientation.setFrom321Euler(delta_transform[5], delta_transform[4], delta_transform[3]);

  resulting_pose.setOrientation(current_orientation*gripper_offset_orientation*delta_orientation*gripper_offset_orientation_inverse);
  //#DEBUG 
  //delta_orientation.get321Euler(delta_transform[5], delta_transform[4], delta_transform[3]);

  EcVector delta_position(delta_transform[0], delta_transform[1], delta_transform[2]);
  EcVector desired_position = base_offset_orientation*delta_position + current_position;
  resulting_pose.setTranslation(desired_position);

  //ROS_INFO("Resulting_translation(x,y,z): %f,%f,%f",desired_position[0], desired_position[1], desired_position[2]);

  //ROS_INFO("Resulting_orientation(rpy): %f,%f,%f",delta_transform[3], delta_transform[4], delta_transform[5]);

  //getchar();//#DEBUG

  manager->commandEndEffector(resulting_pose);
  publishTF(resulting_pose);//publish tf:
}

/**
* Propagate and publish position control system
*/
void propagatePublishJointPositions(const ros::TimerEvent &event)
{
  static ros::Time prev_time = ros::Time::now();
  manager->propagatePositionControlSystem();
  ros::Time current_time = ros::Time::now();
  if((current_time - prev_time).toSec() > (1.0/joint_publish_rate))
  {
    //ROS_INFO("Publishing Joint angles");
    //Publish joint states:
    joint_state.position = manager->system_joint_angles_;//Copy the angles from system
    //Get Joint Velocities and  torques based on flags:
    if(publish_velocities)
    {
      if(!manager->getJointStates(joint_state.velocity, 1))
        ROS_WARN("Failed to get joint velocities");
    }

    if(publish_torques)
    {
      if(!manager->getJointStates(joint_state.effort, 2))
        ROS_WARN("Failed to get joint torques");
    }

    joint_state.header.stamp = ros::Time::now();
    joint_state_pub.publish(joint_state);
    prev_time = current_time;
  }
}

/**
 * Callback to track a marker pose
 */
void markerPoseCallback(geometry_msgs::PoseStamped::ConstPtr pose_msg)
{
  EcCoordinateSystemTransformation marker_pose, ee_pose;
  EcVector desired_trans;
  EcOrientation desired_orientation;
  desired_orientation.setFrom321Euler(M_PI, 0, M_PI/2);
  if(marker_tracking_state == MOVE_TO_OBJECT)
  {
    desired_trans = EcVector(.16,-.20,0);
  }
  else 
  {
    desired_trans = EcVector(.03,-.20,0);
  }

  EcCoordinateSystemTransformation desired_pose(desired_trans, desired_orientation);
  EcCoordinateSystemTransformation gripper_offset_transformation_inverse(EcVector(0,0,0), gripper_offset_orientation_inverse);

  marker_pose.setTranslation(EcVector(pose_msg->pose.position.x, pose_msg->pose.position.y, pose_msg->pose.position.z));
  marker_pose.setOrientation(EcOrientation(pose_msg->pose.orientation.w, pose_msg->pose.orientation.x, pose_msg->pose.orientation.y, pose_msg->pose.orientation.z));
  
  EcCoordinateSystemTransformation marker_in_base = base_offset_transform_inverse*camera_offset_transform*marker_pose;

  ee_pose = marker_in_base*desired_pose*gripper_offset_transformation_inverse;

  EcCoordinateSystemTransformation current_pose;
  manager->getEndEffectorPose(current_pose);
  const EcVector& current_trans = current_pose.translation();

  if(marker_tracking_state == MOVE_TO_GRIP)
  {
    EcVector tdiff = current_trans-ee_pose.translation();
    //if((current_trans-ee_pose.translation()).mag() < .03)
    if(abs(tdiff[0]) < .05 && abs(tdiff[1]) < .03 && abs(tdiff[2]) < .05)
    {
      EcCoordinateSystemTransformation gripper_pose;
      manager->getGripperPose(gripper_pose);
      gripper_pose.setTranslationZ(-.007);
      manager->commandGripper(gripper_pose);
      marker_tracking_state = GRIPPING;
      ROS_INFO("Gripper Pose: %f", -0.007);//#DEBUG
    }
    else if((current_trans-ee_pose.translation()).mag() > .25)
    {
      marker_tracking_state = MOVE_TO_OBJECT;
    }
    else
    {
      EcCoordinateSystemTransformation gripper_pose;
      manager->getGripperPose(gripper_pose);
      gripper_pose.setTranslationZ(.008);
      manager->commandGripper(gripper_pose);
      manager->commandEndEffector(ee_pose);
      ROS_INFO("Gripper Pose: %f", 0.008);//#DEBUG
    }
  }
  else if(marker_tracking_state == GRIPPING)
  {
    if((current_trans-ee_pose.translation()).mag() > .06)
    {
      EcCoordinateSystemTransformation gripper_pose;
      manager->getGripperPose(gripper_pose);
      gripper_pose.setTranslationZ(.008);
      manager->commandGripper(gripper_pose);
      manager->commandEndEffector(ee_pose);
      marker_tracking_state = MOVE_TO_OBJECT;
      ROS_INFO("Gripper Pose: %f", 0.008);//#DEBUG
    }
    else
    {
      EcCoordinateSystemTransformation gripper_pose;
      manager->getGripperPose(gripper_pose);
      gripper_pose.setTranslationZ(-.007);
      manager->commandGripper(gripper_pose);
      ROS_INFO("Gripper Pose: %f", -0.007);//#DEBUG
    }
  }
  else if (marker_tracking_state == MOVE_TO_OBJECT)
  {
    /** Set yaw to point at marker **
    EcVector col0, col1, col2;
    EcOrientation ee_orientation = ee_pose.orientation();
    ee_orientation.getDcmColumns(col0, col1, col2);
    col2 = -marker_in_base.translation();
    col2.setZ(0);
    col2.normalize();
    col1 = col2.cross(col0);
    col1.normalize();
    col2 = col0.cross(col1);
    col2.normalize();
    
    ee_orientation.setFromDcmColumns(col0, col1, col2);
    ee_pose.setOrientation(ee_orientation);
    /****/
    EcCoordinateSystemTransformation gripper_pose;
    manager->getGripperPose(gripper_pose);
    gripper_pose.setTranslationZ(.008);
    manager->commandGripper(gripper_pose);
    manager->commandEndEffector(ee_pose);
    ROS_INFO("Gripper Pose: %f", 0.008);//#DEBUG

    if((current_trans-ee_pose.translation()).mag() < .02)
    {
      marker_tracking_state = MOVE_TO_GRIP;
    }
  }
  publishTF(ee_pose);
}

/**
 * Callback to receive external pose commands
 */
void eePoseCallback(geometry_msgs::Pose::ConstPtr pose_msg)
{
  EcOrientation desired_orientation;
  EcCoordinateSystemTransformation desired_pose;
  desired_pose.setTranslation(EcVector(pose_msg->position.x, pose_msg->position.y, pose_msg->position.z));
  //desired_pose.setOrientation(base_offset_orientation*EcOrientation(1, 0, 0, 0)*gripper_offset_orientation);
  //desired_pose.setOrientation(base_offset_orientation*EcOrientation(pose_msg->orientation.w, pose_msg->orientation.x, pose_msg->orientation.y, pose_msg->orientation.z)*gripper_offset_orientation_inverse);
  desired_pose.setOrientation(EcOrientation(pose_msg->orientation.w, pose_msg->orientation.x, pose_msg->orientation.y, pose_msg->orientation.z));
  manager->commandEndEffector(desired_pose);
  publishTF(desired_pose);//publish tf:
}

/**
* Control gripper using topic
*/
void eeGripperCallback(std_msgs::Float64::ConstPtr gripper_msg)
{
  EcCoordinateSystemTransformation gripper_pose;
  manager->getGripperPose(gripper_pose);
  if(gripper_msg->data < manager->arm_joint_limits_[7].upper_limit_-0.0005 && gripper_msg->data > manager->arm_joint_limits_[7].lower_limit_+0.0005)//Adding some slack from boundaries
  {
    gripper_pose.setTranslationZ(gripper_msg->data);
    manager->commandGripper(gripper_pose);
    ROS_INFO("Gripper Position: %f",gripper_msg->data);
  }
  else
  {
    ROS_WARN("Gripper position hits bounds: %f", gripper_msg->data);
  }
}

void cytonInterfaceCallback(cyton_actin_ros_driver::CytonInterfaceConfig &config, uint32_t level, ros::NodeHandle &nh) 
{

  if(config.home || config.fold_config)
  {
    config.control_mode = cyton_actin_ros_driver::CytonInterface_JointControl;
  }
  //Check if control mode is changed:
  // If control mode is changed, write the current values to reconfig first
  if(level & 0x0002)
  {
    ROS_INFO("Mode Changed");
    if(config.control_mode == cyton_actin_ros_driver::CytonInterface_JoyStick)
    {
      //Close external callbacks if exist
      if(ee_pose_enabled)
      {
        ee_pose_sub.shutdown();
        ee_gripper_sub.shutdown();
        ee_pose_enabled = false;
      }
      if(marker_pose_enabled)
      {
        marker_pose_sub.shutdown();
        marker_pose_enabled = false;
      }
      //Create a subscriber
      ROS_INFO("Creating a subscriber");
      joystick_sub = nh.subscribe("/spacenav/joy",1,joyCallback);
      joystick_enabled = true;
    }
    else if(config.control_mode == cyton_actin_ros_driver::CytonInterface_JointControl)
    {
      ROS_INFO("Entering Joint Angles mode");
      //Write the reconfig joint data with joint angles from position control system:
      config.J1 = ROUND(manager->system_joint_angles_[0],1000);
      config.J2 = ROUND(manager->system_joint_angles_[1],1000);
      config.J3 = ROUND(manager->system_joint_angles_[2],1000);
      config.J4 = ROUND(manager->system_joint_angles_[3],1000);
      config.J5 = ROUND(manager->system_joint_angles_[4],1000);
      config.J6 = ROUND(manager->system_joint_angles_[5],1000);
      config.J7 = ROUND(manager->system_joint_angles_[6],1000);
      config.J8 = ROUND(manager->system_joint_angles_[7],1000);
      
      ROS_INFO("Current Joint Angles: %f,%f,%f,%f,%f,%f,%f,%f",config.J1,config.J2,config.J3,config.J4,config.J5,config.J6,config.J7,config.J8);
      if(joystick_enabled)
      {
        //Shutdown the subscriber:
        joystick_sub.shutdown();
        joystick_enabled = false;
      }
      if(ee_pose_enabled)
      {
        ee_pose_sub.shutdown();
        ee_gripper_sub.shutdown();
        ee_pose_enabled = false;
      }
      if(marker_pose_enabled)
      {
        marker_pose_sub.shutdown();
        marker_pose_enabled = false;
      }
    }
    else if(config.control_mode == cyton_actin_ros_driver::CytonInterface_EndEffector)
    {
      manager->synchronizePositionControlSystemToHardware();//First set the position control system to the hardware
      EcCoordinateSystemTransformation current_pose;
      manager->getEndEffectorPose(current_pose);
      //#DEBUG:
      const EcVector &position = current_pose.translation();
      ROS_INFO("Current Position: %f,%f,%f",position[0], position[1], position[2]);

      config.x_d = ROUND(position[0],100.0);
      config.y_d = ROUND(position[1],100.0);
      config.z_d = ROUND(position[2],100.0);

      const EcOrientation &orientation = current_pose.orientation();
      EcReal angles[3];//psi(z axis yaw), theta(y axis pitch), roll(x axis roll)
      orientation.get321Euler(angles[0],angles[1],angles[2]);
      ROS_INFO("YPR: %f,%f,%f",angles[0], angles[1], angles[2]);

      config.roll_d = ROUND(angles[2],100.0);
      config.pitch_d = ROUND(angles[1],100.0);
      config.yaw_d = ROUND(angles[0],100.0);

      if(joystick_enabled)
      {
        //Shutdown the subscriber:
        joystick_sub.shutdown();
        joystick_enabled = false;
      }

      if(ee_pose_enabled)
      {
        ee_pose_sub.shutdown();
        ee_gripper_sub.shutdown();
        ee_pose_enabled = false;
      }
      if(marker_pose_enabled)
      {
        marker_pose_sub.shutdown();
        marker_pose_enabled = false;
      }
    }
    else if(config.control_mode == cyton_actin_ros_driver::CytonInterface_ExternalControl)
    {
      manager->synchronizePositionControlSystemToHardware();//First set the position control system to the hardware

      if(joystick_enabled)
      {
        //Shutdown the subscriber:
        joystick_sub.shutdown();
        joystick_enabled = false;
      }
      if(marker_pose_enabled)
      {
        marker_pose_sub.shutdown();
        marker_pose_enabled = false;
      }
      ee_pose_sub = nh.subscribe("ee_pose", 1, eePoseCallback);
      ee_gripper_sub = nh.subscribe("ee_gripper", 1, eeGripperCallback);
      ee_pose_enabled = true;
    }
    else if(config.control_mode == cyton_actin_ros_driver::CytonInterface_MarkerControl)
    {
      manager->synchronizePositionControlSystemToHardware();//First set the position control system to the hardware

      ROS_INFO("Marker control mode on");
      if(joystick_enabled)
      {
        //Shutdown the subscriber:
        joystick_sub.shutdown();
        joystick_enabled = false;
      }
      if(ee_pose_enabled)
      {
        ee_pose_sub.shutdown();
        ee_gripper_sub.shutdown();
        ee_pose_enabled = false;
      }
      marker_pose_sub = nh.subscribe("marker_pose", 1, markerPoseCallback);
      marker_pose_enabled = true;
    }

    manager->resetTime();//Reset the system time to avoid fast moving of the arm after switching modes
  }

  //Check if we have to go home:
  if(config.home)
  {
    //manager->commandJointStates(home_joint_positions);
    config.home = false;
    config.J1 = home_joint_positions[0];
    config.J2 = home_joint_positions[1];
    config.J3 = home_joint_positions[2];
    config.J4 = home_joint_positions[3];
    config.J5 = home_joint_positions[4];
    config.J6 = home_joint_positions[5];
    config.J7 = home_joint_positions[6];
    config.J8 = home_joint_positions[7];
    //config.control_mode = cyton_actin_ros_driver::CytonInterface_JointControl;
    //return;
  }
  else if(config.fold_config)
  {
    //manager->commandJointStates(home_joint_positions);
    config.fold_config = false;
    config.J1 = 0;
    config.J2 = 0;
    config.J3 = 0;
    config.J4 = -M_PI/2.;
    config.J5 = 0;
    config.J6 = 0;
    config.J7 = 0;
    config.J8 = home_joint_positions[7];
    //config.control_mode = cyton_actin_ros_driver::CytonInterface_JointControl;
    //return;
  }

 
  //Set final pose based on the inputs:
  switch(config.control_mode)
  {
    case cyton_actin_ros_driver::CytonInterface_JointControl:
    {
      //Use joint data to set the final pose:
      EcRealVector joint_angles(manager->numberOfJoints(),0);
      joint_angles[0] = config.J1;
      joint_angles[1] = config.J2;
      joint_angles[2] = config.J3;
      joint_angles[3] = config.J4;
      joint_angles[4] = config.J5;
      joint_angles[5] = config.J6;
      joint_angles[6] = config.J7;
      joint_angles[7] = config.J8;
      ROS_INFO("Setting Command Joint angles");
      manager->commandJointStates(joint_angles);
      if(manager->command_gripper_explicitly_)
      {
        manager->commandGripper(config.J8);
      }
      ROS_INFO("Done Commanding Joint angles");
      break;
    }
    case cyton_actin_ros_driver::CytonInterface_EndEffector:
    {
      //TODO: Check if the base_offset_orientation needs to be accounted here
      EcCoordinateSystemTransformation final_pose;
      EcOrientation desired_orientation;
      desired_orientation.setFrom321Euler(config.yaw_d, config.pitch_d, config.roll_d);
      final_pose.setOrientation(desired_orientation);
      EcVector desired_position(config.x_d, config.y_d, config.z_d);
      final_pose.setTranslation(desired_position);
      manager->commandEndEffector(final_pose);
      publishTF(final_pose);//publish tf:
      break;
    }
  }
  return;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "actin_driver");

  ros::NodeHandle nh("~");

  std::string simulation_file_name("1"), config_file_name("2");

  nh.getParam("simulation_file_name", simulation_file_name);
  nh.getParam("config_file_name", config_file_name);

  manager = new CytonHardwareManager(simulation_file_name, config_file_name);
  manager->command_gripper_explicitly_ = true;

  //getchar();//Wait till you are ready

  home_joint_positions.resize(manager->numberOfJoints(),0.0);
  
  //Set gripper offset orientation to have x forward, y to left and z up:
  {
    std::string mode = "GROUND";
    if(nh.hasParam("mode"))
    {
      nh.getParam("mode", mode); 
      nh.deleteParam("mode");
    }
    if(!strcmp(mode.c_str(),"QUAD"))
    {
      gripper_offset_orientation.setFrom321Euler(M_PI,M_PI/2,0);//only pitch by pi/2
      //Base offset orientation
      base_offset_orientation.setFrom321Euler(-M_PI/2,M_PI,0);
      base_offset_transform.setTranslation(EcVector(.071, 0, 0));
      base_offset_transform.setOrientation(base_offset_orientation);
      base_offset_transform_inverse = base_offset_transform.inverse();
       
      //Camera offset
      EcOrientation camera_offset_orientation;
      camera_offset_orientation.setFrom321Euler(-M_PI/2, 0, -3*M_PI/4.);
      camera_offset_transform.setTranslation(EcVector(.155, 0, .215));
      camera_offset_transform.setOrientation(camera_offset_orientation);
    }
    else if(!strcmp(mode.c_str(),"GROUND"))
    {
      gripper_offset_orientation.setFrom321Euler(0,M_PI/2,0);//only pitch by pi/2
      //Base offset orientation
      base_offset_orientation.setFrom321Euler(0,0,0);
    }
    else
    {
      ROS_ERROR("Specify the mode as GROUND or QUAD");
      return 0;
    }
    //gripper_offset_orientation.setFrom321Euler(0,M_PI/2,0);//only pitch by pi/2
    //gripper_offset_orientation.setFrom321Euler(M_PI,M_PI/2,0);//only pitch by pi/2
    gripper_offset_orientation_inverse = gripper_offset_orientation.inverse();
    //Base offset orientation
    //base_offset_orientation.setFrom321Euler(-M_PI/2,M_PI,0);//only pitch by pi/2
  }

  //Decide whether to publish joint velocities and torques
  {
    if(nh.hasParam("publish_velocities"))
    {
      nh.getParam("publish_velocities", publish_velocities); 
      nh.deleteParam("publish_velocities");
    }
    if(nh.hasParam("publish_torques"))
    {
      nh.getParam("publish_torques", publish_torques); 
      nh.deleteParam("publish_torques");
    }
  }

  //Set Joint Publish Rate based on parameter:
  if(nh.hasParam("joint_publish_rate"))
  {
    nh.getParam("joint_publish_rate", joint_publish_rate); 
    nh.deleteParam("joint_publish_rate");
  }

  home_joint_positions[0] = 0; 
  home_joint_positions[1] = 0;
  home_joint_positions[2] = 0;
  home_joint_positions[3] = 0;
  home_joint_positions[4] = 0;
  home_joint_positions[5] = 0;
  home_joint_positions[6] = 0;
  home_joint_positions[7] = 0;//Upper limit to gripper
  for(int jj = 0; jj < manager->numberOfJoints(); jj++)
  {
    std::string param_name = "J"+boost::to_string(jj+1);
    if(nh.hasParam(param_name))
    {
      double temp_angle;
      nh.getParam(param_name, temp_angle);

      //Check for bounds:
      if(temp_angle < manager->arm_joint_limits_[jj].upper_limit_-0.0005 && temp_angle > manager->arm_joint_limits_[jj].lower_limit_+0.0005)//Adding some slack from boundaries
        home_joint_positions[jj] = temp_angle;

      nh.deleteParam(param_name);
    }
  }
  //Send the joint angles:
  //manager->setJointStates(home_joint_positions);

  //Print the goal joint angles:
  ROS_INFO("Desired Joint Angles: %f,%f,%f,%f,%f,%f,%f,%f",home_joint_positions[0],home_joint_positions[1],home_joint_positions[2],home_joint_positions[3],home_joint_positions[4],home_joint_positions[5],home_joint_positions[6],home_joint_positions[7]);
  //ros::Duration(1.0).sleep();//Sleep till it actually gets to the position #TODO

  //Synchronize the position control system:
  manager->synchronizePositionControlSystemToHardware();

  //Create Joint State message and publisher
  //initializing joint msg
  joint_state.name.resize(8);
  joint_state.header.frame_id = "Cyton1500";
  joint_state.position.resize(8);
  joint_state.name[0] = "shoulder_roll_joint";
  joint_state.name[1] = "shoulder_pitch_joint";
  joint_state.name[2] = "shoulder_yaw_joint";
  joint_state.name[3] = "elbow_pitch_joint";
  joint_state.name[4] = "elbow_yaw_joint";
  joint_state.name[5] = "wrist_pitch_joint";
  joint_state.name[6] = "wrist_roll_joint";
  joint_state.name[7] = "gripper_joint";

  //Create Joint State Publisher:
  joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states",1);

  //Create a reconfiguration interface to set position commands
	dynamic_reconfigure::Server<cyton_actin_ros_driver::CytonInterfaceConfig> server;
  //Set the max and min for joint angles from the hardware:
  cyton_actin_ros_driver::CytonInterfaceConfig temp_config;
  //Getmax config
  server.getConfigMax(temp_config);
  //set the limits of joints to temp_config:
  temp_config.J1 = ROUND(manager->arm_joint_limits_[0].upper_limit_,1000);
  temp_config.J2 = ROUND(manager->arm_joint_limits_[1].upper_limit_,1000);
  temp_config.J3 = ROUND(manager->arm_joint_limits_[2].upper_limit_,1000);
  temp_config.J4 = ROUND(manager->arm_joint_limits_[3].upper_limit_,1000);
  temp_config.J5 = ROUND(manager->arm_joint_limits_[4].upper_limit_,1000);
  temp_config.J6 = ROUND(manager->arm_joint_limits_[5].upper_limit_,1000);
  temp_config.J7 = ROUND(manager->arm_joint_limits_[6].upper_limit_,1000);
  temp_config.J8 = ROUND(manager->arm_joint_limits_[7].upper_limit_,1000);
  ROS_INFO("Joints max: %f,%f,%f,%f,%f,%f,%f,%f", temp_config.J1, temp_config.J2, temp_config.J3, temp_config.J4, temp_config.J5, temp_config.J6, temp_config.J7, temp_config.J8);
  //Set max config:
  server.setConfigMax(temp_config);
  //Get min config:
  server.getConfigMin(temp_config);
  //set the limits of joints to temp_config:
  temp_config.J1 = ROUND(manager->arm_joint_limits_[0].lower_limit_,1000);
  temp_config.J2 = ROUND(manager->arm_joint_limits_[1].lower_limit_,1000);
  temp_config.J3 = ROUND(manager->arm_joint_limits_[2].lower_limit_,1000);
  temp_config.J4 = ROUND(manager->arm_joint_limits_[3].lower_limit_,1000);
  temp_config.J5 = ROUND(manager->arm_joint_limits_[4].lower_limit_,1000);
  temp_config.J6 = ROUND(manager->arm_joint_limits_[5].lower_limit_,1000);
  temp_config.J7 = ROUND(manager->arm_joint_limits_[6].lower_limit_,1000);
  temp_config.J8 = ROUND(manager->arm_joint_limits_[7].lower_limit_,1000);
  ROS_INFO("Joints min: %f,%f,%f,%f,%f,%f,%f,%f", temp_config.J1, temp_config.J2, temp_config.J3, temp_config.J4, temp_config.J5, temp_config.J6, temp_config.J7, temp_config.J8);
  //Set min config:
  server.setConfigMin(temp_config);

	dynamic_reconfigure::Server<cyton_actin_ros_driver::CytonInterfaceConfig>::CallbackType f;
	f = boost::bind(&cytonInterfaceCallback, _1, _2, nh);
	server.setCallback(f);

  //Command initial joint angles
  manager->commandJointStates(home_joint_positions);
  

  //Create a timer to keep propagating the position control system
  ros::Timer timer = nh.createTimer(ros::Duration(manager->step_size_), propagatePublishJointPositions);

  EcVector circle_position;

  ros::spin();

  return 0;
}

