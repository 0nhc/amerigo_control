#include <ros_interface/amerigo_hardware_interface.h>

AmerigoHardwareInterface::AmerigoHardwareInterface(ros::NodeHandle& nh) : nh_(nh)
{
  if(!nh_.getParam(ros::this_node::getName() + "/loop_frequency", loop_frequency_))
  {
    loop_frequency_ = 100.0;
    ROS_WARN("Could not find parameter 'loop_frequency', set to 100.0 Hz by default.");
  }
  if(!nh_.getParam("joint_names", joint_names_))
  {
    ROS_ERROR("Could not find parameter 'joint_names'.");
    abort();
  }

  num_joints_ = joint_names_.size();
  joint_position_.resize(num_joints_, 0.0);
  joint_velocity_.resize(num_joints_, 0.0);
  joint_effort_.resize(num_joints_, 0.0);
  joint_position_command_.resize(num_joints_, 0.0);

  controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
  ros::Duration update_freq = ros::Duration(1.0/loop_frequency_);
  non_realtime_loop_ = nh_.createTimer(update_freq, &AmerigoHardwareInterface::update, this);
  isaac_sim_joints_publisher_ = nh_.advertise<sensor_msgs::JointState>("/isaac_sim/joint_command", 1);
  isaac_sim_joints_subscriber_ = nh_.subscribe("/isaac_sim/joint_states", 1, &AmerigoHardwareInterface::IsaacSimJointsCallback_, this);
  init();
}

AmerigoHardwareInterface::~AmerigoHardwareInterface()
{
}

void AmerigoHardwareInterface::init()
{
  for(int i=0; i<num_joints_; i++)
  {
    // Create joint state interface
    hardware_interface::JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
    joint_state_interface_.registerHandle(jointStateHandle);

    // Create position joint interface
    hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
    position_joint_interface_.registerHandle(jointPositionHandle);

    // Create Joint Limit interface   
    joint_limits_interface::JointLimits limits;
    joint_limits_interface::getJointLimits(joint_names_[i], nh_, limits);
    joint_limits_interface::PositionJointSaturationHandle jointLimitsHandle(jointPositionHandle, limits);
    positionJointSaturationInterface.registerHandle(jointLimitsHandle);
  }

  // Register all joints interfaces    
  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);
  registerInterface(&positionJointSaturationInterface);
}

void AmerigoHardwareInterface::update(const ros::TimerEvent& e)
{
  elapsed_time_ = ros::Duration(e.current_real - e.last_real);
  read();
  controller_manager_->update(ros::Time::now(), elapsed_time_);
  write(elapsed_time_);
}

void AmerigoHardwareInterface::read()
{
  for(int i=0; i<num_joints_; i++)
  {
    // ROS_INFO("Position[%i]: %.2f", i, joint_position_command_[i]);
  }

}

void AmerigoHardwareInterface::write(ros::Duration elapsed_time)
{
  for(int i=0; i<num_joints_; i++)
  {
    positionJointSaturationInterface.enforceLimits(elapsed_time);
    joint_position_command_frame_.header.stamp = ros::Time::now();
    joint_position_command_frame_.header.frame_id = "amerigo";
    joint_position_command_frame_.name = joint_names_;
    joint_position_command_frame_.position = joint_position_command_;
    isaac_sim_joints_publisher_.publish(joint_position_command_frame_);
    // ROS_INFO("Command[%i]: %.2f", i, joint_position_command_[i]);
  }
}

void AmerigoHardwareInterface::IsaacSimJointsCallback_(const sensor_msgs::JointState::ConstPtr& msg)
{
  for(int i=0; i<num_joints_; i++)
  {
    std::string joint_name = joint_names_[i];
    std::vector<std::string> msg_names = msg->name;
    for(int j=0; j<msg_names.size(); j++)
    {
      std::string msg_name = msg_names[j];
      if(joint_name == msg_name)
      {
        // ROS_WARN("%s", msg_name.c_str());
        joint_position_[i] = msg->position[j];
      }
    }
  }
}