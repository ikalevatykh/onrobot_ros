#include <onrobot_gazebo/gripper_plugin.h>

#include <boost/tokenizer.hpp>

namespace gazebo
{
void GripperPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  std::string joint_name, mimic_name, tip_name, palm_name, ns;

  _sdf->Get("name", name_, std::string{ "gripper_plugin" });
  _sdf->Get("robotNamespace", ns, std::string{});
  _sdf->Get("fixGrasp", fix_grasp_, false);
  _sdf->Get("maxEffort", max_effort_, 1.0);
  _sdf->Get("publishRate", publish_rate_, 50.0);
  _sdf->Get("monitorRate", monitor_rate_, 20.0);
  _sdf->Get("jointName", joint_name, std::string{});
  _sdf->Get("mimicName", mimic_name, std::string{});
  _sdf->Get("tipName", tip_name, std::string{});
  _sdf->Get("palmName", palm_name, std::string{});

  // Parse model
  model_ = _parent;
  world_ = model_->GetWorld();

  boost::char_separator<char> sep(", ");
  using tokenizer = boost::tokenizer<boost::char_separator<char>>;

  auto joint_name_tok = tokenizer{ joint_name, sep };
  for (auto it = joint_name_tok.begin(); it != joint_name_tok.end(); ++it)
  {
    joints_.push_back(model_->GetJoint(*it));
  }
  auto mimic_name_tok = tokenizer{ mimic_name, sep };
  for (auto it = mimic_name_tok.begin(); it != mimic_name_tok.end(); ++it)
  {
    mimics_.push_back(model_->GetJoint(*it));
  }
  auto tip_name_tok = tokenizer{ tip_name, sep };
  for (auto it = tip_name_tok.begin(); it != tip_name_tok.end(); ++it)
  {
    tip_links_.push_back(model_->GetLink(*it));
  }
  palm_link_ = model_->GetLink(palm_name);

  lower_limit_ = joints_[0]->LowerLimit(0);
  upper_limit_ = joints_[0]->UpperLimit(0);

  // ROS node
  rosnode_ = new ros::NodeHandle(ns);

  // ROS API: Joint state publisher
  joint_state_publisher_ = rosnode_->advertise<sensor_msgs::JointState>("joint_states", 100);
  next_publish_time_ = world_->SimTime();

  // ROS API: Gripper action server
  action_server_.reset(new ActionServer(*rosnode_, "gripper_controller/gripper_cmd", false));
  action_server_->start();
  next_monitor_time_ = world_->SimTime();

  // Listen to the update event. This event is broadcast every simulation iteration.
  update_conn_ = event::Events::ConnectWorldUpdateBegin(  //
      boost::bind(&GripperPlugin::OnUpdate, this));

  ROS_INFO_STREAM_NAMED(name_, "The \"" << name_ << "\" plugin is loaded.");
}

void GripperPlugin::OnUpdate()
{
  if (action_server_->isNewGoalAvailable())
  {
    auto goal = action_server_->acceptNewGoal();
    command_position_ = goal->command.position;
    command_effort_ = goal->command.max_effort;
    counter_ = int(monitor_rate_) * 2;
  }

  position_ = joints_[0]->Position(0);

  auto direction = command_position_ > 1e-3 ? 1 : -1;
  auto effort = max_effort_ * direction;
  auto margin = 0.1;

  if (direction < 0 && position_ < lower_limit_ + margin)
  {
    effort *= (position_ - lower_limit_) / margin;
  }
  else if (direction > 0 && position_ > upper_limit_ - margin)
  {
    effort *= (upper_limit_ - position_) / margin;
  }

  for (std::size_t i = 0; i < joints_.size(); ++i)
  {
    joints_[i]->SetForce(0, effort);

    if (i < mimics_.size())
    {
      auto position = joints_[i]->Position(0);
      mimics_[i]->SetPosition(0, position, true);
    }
  }

  joints_[0]->SetForce(0, effort);
  joints_[1]->SetForce(0, effort);

  auto current_time = this->world_->SimTime();
  if (current_time > next_monitor_time_)
  {
    CheckForSuccess();
    next_monitor_time_ += common::Time(1.0 / monitor_rate_);
  }
  if (current_time > next_publish_time_)
  {
    PublishJointStates();
    next_publish_time_ += common::Time(1.0 / publish_rate_);
  }
}

void GripperPlugin::CheckForSuccess()
{
  if (!action_server_->isActive())
    return;

  if (counter_-- == 0)
  {
    control_msgs::GripperCommandResult result;
    result.effort = max_effort_;
    result.position = position_;
    result.reached_goal = true;
    result.stalled = false;
    action_server_->setSucceeded(result);
  }
}

void GripperPlugin::PublishJointStates()
{
  joint_state_.header.stamp = ros::Time::now();
  joint_state_.name.resize(joints_.size());
  joint_state_.position.resize(joints_.size());
  joint_state_.velocity.resize(joints_.size());

  for (int i = 0; i < joints_.size(); i++)
  {
    physics::JointPtr joint = joints_[i];
    double velocity = joint->GetVelocity(0);
    double position = joint->Position(0);
    joint_state_.name[i] = joint->GetName();
    joint_state_.position[i] = position;
    joint_state_.velocity[i] = velocity;
  }
  joint_state_publisher_.publish(joint_state_);
}

GripperPlugin::~GripperPlugin()
{
  update_conn_.reset();
  action_server_.reset();

  if (rosnode_)
  {
    rosnode_->shutdown();
  }
}

GZ_REGISTER_MODEL_PLUGIN(GripperPlugin);
}  // namespace gazebo