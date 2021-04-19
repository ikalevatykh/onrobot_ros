#ifndef _ONROBOT_GAZEBO_GRIPPER_PLUGIN_
#define _ONROBOT_GAZEBO_GRIPPER_PLUGIN_

// ROS includes
#include <ros/ros.h>

// ROS messages
#include <control_msgs/GripperCommandAction.h>
#include <sensor_msgs/JointState.h>

// actionlib
#include <actionlib/server/simple_action_server.h>

// Boost includes
#include <boost/bind.hpp>

// Gazebo includes
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
class GripperPlugin : public ModelPlugin
{
  typedef actionlib::SimpleActionServer<control_msgs::GripperCommandAction> ActionServer;
  typedef ActionServer::GoalHandle GoalHandle;

public:
  virtual ~GripperPlugin();

  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:
  void OnUpdate();
  void PublishJointStates();
  void CheckForSuccess();

protected:
  std::string name_;
  bool fix_grasp_ = false;
  double max_effort_ = 5.0;
  double publish_rate_ = 50.0;
  double monitor_rate_ = 20.0;
  double lower_limit_;
  double upper_limit_;
  double command_position_ = 0.0;
  double command_effort_ = 0.0;
  double position_ = 0.0;
  double velocity_ = 0.0;
  int counter_ = 0;

  ros::NodeHandle* rosnode_ = nullptr;

  sensor_msgs::JointState joint_state_;
  ros::Publisher joint_state_publisher_;
  common::Time next_publish_time_;

  boost::shared_ptr<ActionServer> action_server_;
  control_msgs::GripperCommandResultPtr pre_alloc_result_;
  common::Time next_monitor_time_;

  physics::ModelPtr model_;
  physics::WorldPtr world_;

  std::vector<std::string> joint_names_;
  std::vector<physics::JointPtr> joints_;
  std::vector<physics::JointPtr> mimics_;
  std::vector<physics::LinkPtr> tip_links_;
  physics::LinkPtr palm_link_;

  event::ConnectionPtr update_conn_;
};
}  // namespace gazebo

#endif