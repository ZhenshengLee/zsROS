#ifndef CYTON_HARDWARE_MANAGER
#define CYTON_HARDWARE_MANAGER
/** 
* This is a manager for loading and using Cyton Hardware Interface
* The methods provide utility wrappers available from ecQuickStartHardware Example
* This is almost independent of ROS except for ROS_INFO
*
* The function propagatePositionControlSystem should be called continously using a timer
* to move the arm
*/
#include <ros/ros.h>
#include <boost/thread/tss.hpp>
#include <hardwareInterface/cytonHardwareInterface.h>
#include <xml/ecXmlVectorType.h>
#include <controlCore/ecFrameEndEffector.h>
#include <control/ecIncludeControlExpressions.h>
#include <controlCore/ecPointEndEffector.h>
#include <control/ecPosContSystem.h>
#include <control/ecVelContSystem.h>
#include <systemSimulation/ecSysSimulation.h>
#include <xmlReaderWriter/ecXmlObjectReaderWriter.h>
#include <convertSystem/ecVisualizableStatedSystem.h>

#ifdef ENABLE_GUI
#include <rendCore/ecRenderWindow.h>
#endif

class CytonHardwareManager
{
  private:
    EcSystemSimulation simulation_;//Create Simulation Object Cyton
    boost::shared_ptr<cyton::hardwareInterface > hardware_;///< interface to set velocities etc to the arm
    EcPositionControlSystem position_control_system_;///< position control system
    EcU32 num_joints_;///< Number of joints in this arm

    // declare a visualizable stated system object
    EcVisualizableStatedSystem vis_stated_system_;
    
    EcReal gripper_command_;
    EcReal system_time_;///< Time of system when command was given
#ifdef ENABLE_GUI
   EcRenderWindow renderer_;
#endif
  public:
    struct JointLimits{
      EcReal upper_limit_;
      EcReal lower_limit_;
    };

  public:
    bool command_gripper_explicitly_;
    EcU32 wait_time_in_ms_;///< Timeout for setting commands
    std::vector<JointLimits> arm_joint_limits_;
    //EcReal gripper_limits_[2];///< Lower and upper limits of gripper link
    EcReal step_size_;///< Step size of position control system
    EcRealVector system_joint_angles_;///< Joint angles from position control system. These are set when u synchronize the position control system with hardware or when u run propagatePosition system

  public:
    /** 
     * Constructor for loading the simulation file
     * @
     */
    CytonHardwareManager(EcString simulation_file_name, EcString config_file_name) : wait_time_in_ms_(EcU32(200)), system_time_(0), command_gripper_explicitly_(false)
    {
      // -----------------------------------------------------------------
      // Step #1 - Loading a simulation from a file.
      // -----------------------------------------------------------------
      ROS_INFO("Loading Cyton Simulation: %s",simulation_file_name.c_str());
      bool success = EcXmlObjectReaderWriter::readFromFile(simulation_, simulation_file_name);
      if(!success)
      {
        ROS_WARN("Could not load simulation.");
        return;
      }

      // -----------------------------------------------------------------
      // Step #2 - Configuring hardware
      // -----------------------------------------------------------------
      ROS_INFO("Configuring cyton hardware: %s", config_file_name.c_str());
      hardware_.reset(new cyton::hardwareInterface("cytonPlugin", config_file_name));

      // get ports
      EcStringVector ports = hardware_->availablePorts();
      const EcU32 num_ports = ports.size();
      EcBoolean found_port=EcFalse;
      if(num_ports)
      {

        for(EcU32 ii=0; ii<num_ports; ++ii)
        {
          hardware_->setPort(ports[ii].c_str());
          if(hardware_->init())
          {
            found_port=EcTrue;
            break;
          }
        }

        if(found_port)
        {
          // This gives a velocity of 0.3 radians per second 
          EcReal delta_time = 10.0; // 10 seconds

          // This method will set the internal routines to go from one position to
          // another in this much time.  It is a convenience routine in lieu of
          // specifying a joint velocity on a per-joint basis.
          hardware_->setVelocityFromDeltaTime(delta_time);
        }
        else
        {
          // did not find port
          ROS_WARN("Cound not find valid port.\n");
          return;
        }
      }

      num_joints_ = hardware_->numJoints();
      if(num_joints_)
      {
        ROS_INFO("Found %d servos .",num_joints_);
      }
      else
      {
        ROS_WARN("Invalid configuration. No joints available.\n");
        return;
      }

      EcRealVector initial_hardware_angles(num_joints_, 0.0);

      //update
      if(!hardware_->getJointStates(initial_hardware_angles) ||
          !hardware_->waitUntilCommandFinished(wait_time_in_ms_))
      {
        ROS_WARN("Problem getting initial angles.\n");
        return;
      }

      ROS_DEBUG("Initial State: (");
      for(EcU32 ii=0; ii<initial_hardware_angles.size(); ++ii)
      {
        ROS_DEBUG("%f,",initial_hardware_angles[ii]);
      }
      ROS_DEBUG(")");

   // -----------------------------------------------------------------
   // Step #3 - Viewing a stated system.
   // -----------------------------------------------------------------
   
   //update VizStatedSystem from hardware angles
   simulation_.getVisualizableStatedSystem(vis_stated_system_);//Get a visualizable system for making the position controller work
   
   EcPositionStateVector states = vis_stated_system_.statedSystem().state().positionStates();

   states[0].setJointPositions(initial_hardware_angles);
   vis_stated_system_.statedSystem().setPositionStates(states);
     
#ifdef ENABLE_GUI
   // set the size of the window
   renderer_.setWindowSize(800,800);

   // set the system
   if(!renderer_.setVisualizableStatedSystem(vis_stated_system_))
   {
     ROS_WARN("Cannot set the stated system for renderer");
      return;
   }
   
   std::cout << "Rendering.\n";
   // view the system
   renderer_.renderScene();

   // pause 
   EcSLEEPMS(500);
#endif

      // -----------------------------------------------------------------
      // Step #4 - Defining a position-control system.
      // -----------------------------------------------------------------

      // set the position control system from the loaded simualtion
      position_control_system_=simulation_.positionControlSystem();

      // set the stated system
      const EcStatedSystem &stated_system = simulation_.statedSystem();///<Holds a description of the manipulator system and its state

      position_control_system_.setStatedSystem(&vis_stated_system_.statedSystem());//Set the stated systemf or position control system

      //Get upper and lower limits to all the joints:
      const EcIndividualManipulator& cytonManip = stated_system.system().manipulators()[0];
      arm_joint_limits_.resize(num_joints_);
      for(int jj = 0; jj < num_joints_; jj++)
      {
        const EcManipulatorLink* link = cytonManip.linkByIndex(jj);
        arm_joint_limits_[jj].upper_limit_ = link->jointActuator().upperLimit();
        arm_joint_limits_[jj].lower_limit_ = link->jointActuator().lowerLimit();
      }
      //Set default gripper command
      gripper_command_ = arm_joint_limits_[num_joints_-1].upper_limit_; 
      //Store step size 
      step_size_ = position_control_system_.timeStep();
      //Resize system joint angles:
      system_joint_angles_.resize(num_joints_,0.0);
    }

    /** 
     * Method to get the current joint angles
     * @param joint_angles Joint angles vector which is updated with the current joint angles
     */
    bool getJointAngles(std::vector<EcReal> &joint_angles)
    {
      //Resize the provided joint angles if it does not have enough positions to hold
      joint_angles.resize(num_joints_,0.0);

      //update
      if(!hardware_->getJointStates(joint_angles) ||
          !hardware_->waitUntilCommandFinished(wait_time_in_ms_))
      {
        ROS_WARN("Problem getting initial angles.\n");
        return false;
      }
      return true;
    }

    /** 
     * Method to get the current joint angles(id = 0)/velocities(id=1)/torques(id=3)
     * @param joint_states Current angles/velocities/torques based on request
     * @param id           Choose what to get either angles or velocities or torques 
     */
    bool getJointStates(std::vector<EcReal> &joint_states, int id = 0)
    {
      //Resize the provided joint angles if it does not have enough positions to hold
      joint_states.resize(num_joints_,0.0);

      EcU32 state_type;
      switch(id)
      {
        case 0:
          state_type = cyton::JointAngleInRadians;
          break;
        case 1:
          state_type = cyton::JointVelocity;
          break;
        case 2:
          state_type = cyton::JointTorque;
          break;
        default:
          state_type = cyton::JointAngleInRadians;
          break;
      }

      //update
      if(!hardware_->getJointStates(joint_states, state_type) ||
          !hardware_->waitUntilCommandFinished(wait_time_in_ms_))
      {
        ROS_WARN("Problem getting joint state.\n");
        return false;
      }
      return true;
    }

    /**  
     * Obtain the current end effector position
     */
    void getEndEffectorPose(EcCoordinateSystemTransformation &end_effector_pose)
    {
      EcU32 hand_index=0;
      EcU32 ee_set_index=1;
      end_effector_pose = position_control_system_.actualPlacement(0,hand_index);
    }

    /**
    * Get Gripper Pose
    */
    void getGripperPose(EcCoordinateSystemTransformation &gripper_pose)
    {
      EcU32 gripper_index = 1;
      gripper_pose=position_control_system_.actualPlacement(0,gripper_index);
    }

    /**
     * Computes the current joint angles and sets the position control system 
     * initial state to the hardware state. It also resets the 
     * position control system time to 0
     */
    void synchronizePositionControlSystemToHardware()
    {
      EcU32 hand_index=0;
      EcU32 ee_set_index=1;
      //#NOTE For some weird reason joint angles from position control system are 9 rather than 8 WHY???
      EcXmlRealVector joint_angles(num_joints_+1,0);//Create joint angles xml vector
      {
        EcRealVector joint_angles_real(num_joints_,0);//Temporary joint angles vector
        getJointAngles(joint_angles_real);//Fill joint angles with hardware values
        for(int jj = 0; jj < num_joints_; jj++)
        {
          joint_angles[jj] = joint_angles_real[jj];
          system_joint_angles_[jj] = joint_angles_real[jj];//Copy the real angles to system joint angles too
        }
        joint_angles[num_joints_] = joint_angles_real[num_joints_-1];//Copy the last virtual joint angle to be input to the position control system
      }
      //Find the current state of the manipulator from position control system:

      //select the active manipulator and endeffector
      position_control_system_.setActiveEndEffectorSet(0,ee_set_index);

      //Get the last state:
      EcManipulatorSystemState system_state = position_control_system_.lastCalculatedState();

      {
        //Get the joint angles and print whats there first:
        const EcXmlRealVector &xml_joint_angles = system_state.positionStates()[0].jointPositions();
      }

      //set the joint angles of system_state:
      system_state.positionStates()[0].setJointPositions(joint_angles);

      //set the initial state of position control system back:
      position_control_system_.setCurrentState(system_state);

      //set the time of position control system to 0:
      position_control_system_.setTime(0);
    }

    /**
     * Command Joint angles through position control system
     * Uses timer to execute the commands
     */
    bool commandJointStates(EcRealVector &joint_angles)
    {
      bool success = position_control_system_.setActiveEndEffectorSet(0,EcVelocityController::JOINT_CONTROL_INDEX);
      if(!success)
      {
        ROS_WARN("Failed to set joint control mode");
        return false;
      }
      EcEndEffectorPlacement current_placement = position_control_system_.actualPlacement(0,0);

      const EcXmlRealVector &current_joint_angles = current_placement.data();
      EcXmlRealVector desired_joint_angles(current_placement.data().size(), 0);
      //#DEBUG:
      ROS_INFO("Joint angles: ");
      for(int jj = 0; jj < num_joints_; jj++)
      {
        desired_joint_angles[jj] = joint_angles[jj];
        ROS_INFO("Desired: %f",joint_angles[jj]);
        ROS_INFO("Current from systemII: %f",current_joint_angles[jj]);
      }
      desired_joint_angles[num_joints_] = joint_angles[num_joints_-1];//Additional one
      current_placement.setData(desired_joint_angles);

      success = position_control_system_.setDesiredPlacement(0,0, current_placement);
      if(!success)
      {
        ROS_WARN("Failed to set desired joint angles");
        return false;
      }
      //system_time_ = position_control_system_.time();//set the starting time
      //ROS_INFO("System_time: %f",system_time_);
    }

    /**
     * Set Joint states angles and velocities(optional) immediately to hardware
     * Have to run synchronize hardware after this command to get position control 
     * system to sync with hardware
     * #TODO Maybe have to set system_joint_angles_ here and not in synchronization 
     */
    bool setJointStates(std::vector<EcReal> &joint_angles, std::vector<EcReal> *joint_velocities=0)
    {
      if(joint_velocities != 0)
      {
        if(!hardware_->setJointCommands(joint_angles,*joint_velocities))
        {
          ROS_WARN("Problem setting angles.\n");
          return false;
        }
      }
      else
      {
        if(!hardware_->setJointCommands(joint_angles))
        {
          ROS_WARN("Problem setting angles.\n");
          return false;
        }
      }
      return true;
    }

    /** 
     * Command Endeffector position  and orientation
     */
    bool commandEndEffector(EcCoordinateSystemTransformation &end_effector_pose)
    {
      EcU32 ee_set_index=1;
      EcU32 hand_index=0;

      bool success = position_control_system_.setActiveEndEffectorSet(0,ee_set_index);
      if(!success)
      {
        ROS_WARN("Failed to set end effector pose");
        return false;
      }

      success = position_control_system_.setDesiredPlacement(0,hand_index,end_effector_pose);
      if(!success)
      {
        ROS_WARN("Failed to set end effector pose");
        return false;
      }
      //system_time_ = position_control_system_.time();//set the starting time
      //ROS_INFO("System_time: %f",system_time_);

      return true;
    }


    /**
    * Command Gripper Position
    * Set the Grippers z position to joint value
    */
    bool commandGripper(EcCoordinateSystemTransformation &gripper_pose)
    {
      if(command_gripper_explicitly_)
      {
        EcReal ul = arm_joint_limits_[num_joints_-1].upper_limit_;
        EcReal ll = arm_joint_limits_[num_joints_-1].lower_limit_;
        gripper_command_ = gripper_pose.translation()[2] > ul ? ul : (gripper_pose.translation()[2] < ll ? ll : gripper_pose.translation()[2]);
        return true;
      }

      EcU32 gripper_index = 1;
      bool success = position_control_system_.setDesiredPlacement(0, gripper_index, gripper_pose);
      if(!success)
      {
        ROS_WARN("Failed to set gripper pose");
        return false;
      }
      return true;
    }

    /**
    * Command Gripper Position
    * Set the Grippers z position to joint value
    */
    bool commandGripper(EcReal &gripper_z)
    {
      EcCoordinateSystemTransformation gripper_pose;
      gripper_pose.setTranslation(EcVector(0,0,gripper_z));
      commandGripper(gripper_pose);
    }

    /**
     * Move the position control system by one step
     */
    void propagatePositionControlSystem()
    {
      EcManipulatorSystemState dynamic_state;
      // get the current time
      system_time_ += step_size_;
      //ROS_INFO("system_time: %f",system_time_);

      // calculate the state at current time 
      position_control_system_.calculateState(system_time_,dynamic_state);

      EcXmlRealVector xml_joint_angles = dynamic_state.positionStates()[0].jointPositions();
      //ROS_INFO("Joint angles:");
      for(int jj = 0; jj < num_joints_; jj++)
      {
        system_joint_angles_[jj] = xml_joint_angles[jj];
        //ROS_INFO("%f",system_joint_angles_[jj]);
      }

      if(command_gripper_explicitly_)
      {
        system_joint_angles_[num_joints_-1] = gripper_command_;
      }

      if(!hardware_->setJointCommands(system_joint_angles_))
      {
        std::cerr << "Problem setting angles.\n";
        return;
      }

      // set the hardware
      // commandJointStates(joint_angles);

      EcSLEEPMS(static_cast<EcU32>(1000*step_size_));

    }
    

    /*
     * Command End effector in velocity mode
     * Not yet implemented
     */

    /**
     * Command to set time taken to final desired position
     * this will allow future joint angle commands to automatically compute 
     * joint velocities
     */
    bool setDeltaT(EcReal delta_time)
    {
      hardware_->setVelocityFromDeltaTime(delta_time);
    }

    /**
    * Get number of joints
    */
    EcU32 numberOfJoints()
    {
      return num_joints_;
    }

    /**
    * Reset the position control system time. 
    * Should be done after changing mode 
    */
    void resetTime()
    {
      position_control_system_.setTime(0);
      system_time_ = 0;//Reset the time
    }

    /** Destructor
    */
    ~CytonHardwareManager()
    {
      hardware_->shutdown();
    }
};
#endif
