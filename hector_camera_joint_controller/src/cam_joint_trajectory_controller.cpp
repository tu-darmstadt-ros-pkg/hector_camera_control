//=================================================================================================
// Copyright (c) 2016, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <hector_camera_joint_controller/cam_joint_trajectory_controller.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <angles/angles.h>
#include <std_msgs/Float64.h>
#include <std_msgs/builtin_double.h>

#include <moveit_msgs/GetMotionPlan.h>

#include <hector_perception_msgs/CameraPatternInfo.h>
#include <control_msgs/QueryTrajectoryState.h>

namespace cam_control {

enum
{
  FIRST = 0, SECOND = 1, THIRD = 2
};

typedef enum { MODE_OFF = 0, MODE_LOOKAT = 1, MODE_PATTERN = 2, MODE_OVERRIDE = 4, MODE_ORIENTATION = 5 } ControlMode;

enum
{
  xyz, zyx
};

// Constructor
CamJointTrajControl::CamJointTrajControl()
  : joint_trajectory_preempted_(false)
  , patterns_param_("camera/patterns")
  , control_mode_(MODE_OFF)
  , use_direct_position_commands_(false)
  , new_goal_received_(false)
  , pattern_index_ (0)
  , use_planning_based_pointing_(true)
  , disable_orientation_camera_command_input_(false)
{
  transform_listener_ = 0;

  this->Init();
}

// Destructor
CamJointTrajControl::~CamJointTrajControl()
{
  delete transform_listener_;
}

// Load the controller
void CamJointTrajControl::Init()
{
  pnh_ = ros::NodeHandle("~");
  nh_ = ros::NodeHandle("");

  pattern_info_pub_ = pnh_.advertise<hector_perception_msgs::CameraPatternInfo>("/available_camera_patterns",2, true);

  pnh_.param<std::string>("move_group", move_group_name_, "sensor_head_group");

  pnh_.getParam("controller_namespace", controller_namespace_);
  pnh_.getParam("control_loop_period", control_loop_period_);
  pnh_.getParam("default_direction_reference_frame", default_look_dir_frame_);
  pnh_.getParam("stabilize_default_direction_reference", stabilize_default_look_dir_frame_);
  pnh_.getParam("robot_link_reference_frame", robot_link_reference_frame_);

  lookat_frame_ = std::string("sensor_head_mount_link");
  pnh_.getParam("lookat_reference_frame", lookat_frame_);
  
  command_goal_time_from_start_ = 0.3;
  pnh_.getParam("command_goal_time_from_start", command_goal_time_from_start_);
  
  max_axis_speed_ = 0.0;
  pnh_.getParam("max_axis_speed", max_axis_speed_);

  double default_interval_double = 1.0;
  pnh_.getParam("default_interval", default_interval_double);
  default_interval_ = ros::Duration(default_interval_double);

  patterns_param_ = std::string(pnh_.getNamespace()+"/patterns");
  pnh_.getParam("patterns_param", patterns_param_);

  std::string type_string;
  pnh_.getParam("joint_order_type", type_string);


  pnh_.getParam("use_direct_position_commands", use_direct_position_commands_);
  pnh_.getParam("use_planning_based_pointing", use_planning_based_pointing_);

  if (type_string == "xyz"){
    rotationConv = xyz;
  }else{
    rotationConv = zyx;
  }


  this->loadPatterns();

  transform_listener_ = new tf::TransformListener();

  get_plan_service_client_ = pnh_.serviceClient<moveit_msgs::GetMotionPlan>("/plan_kinematic_path");

  controller_nh_ = ros::NodeHandle(controller_namespace_);

  if (use_direct_position_commands_){
    servo_pub_1_ = nh_.advertise<std_msgs::Float64>("servo1_command", 1);
    servo_pub_2_ = nh_.advertise<std_msgs::Float64>("servo2_command", 1);

  }else{
    // Actions, subscribers
    joint_traj_client_.reset(new actionlib::ActionClient<control_msgs::FollowJointTrajectoryAction>(controller_nh_.getNamespace() + "/follow_joint_trajectory"));
    joint_trajectory_action_status_sub_ = controller_nh_.subscribe("follow_joint_trajectory/status", 1, &CamJointTrajControl::trajActionStatusCallback, this);

    if (move_group_name_.empty()){
      getJointNamesFromController(controller_nh_);
    }else{
      getJointNamesFromMoveGroup();
    }
  }

  control_timer = nh_.createTimer(ros::Duration(control_loop_period_), &CamJointTrajControl::controlTimerCallback, this, false, true);
  
  pnh_.getParam("disable_orientation_camera_command_input", disable_orientation_camera_command_input_);
  
  if (!disable_orientation_camera_command_input_){
    sub_ = nh_.subscribe("/camera/command", 1, &CamJointTrajControl::cmdCallback, this);    
  }

  this->Reset();

  //Sleep for short time to let tf receive messages
  ros::Duration(1.0).sleep();


  look_at_server_ = std::make_shared<actionlib::SimpleActionServer<hector_perception_msgs::LookAtAction>>(pnh_, "look_at", false);
  look_at_server_->registerGoalCallback(boost::bind(&CamJointTrajControl::lookAtGoalCallback, this));
  look_at_server_->registerPreemptCallback(boost::bind(&CamJointTrajControl::lookAtPreemptCallback, this));
  look_at_server_->start();
}

void CamJointTrajControl::getJointNamesFromMoveGroup()
{
  // Load moveit robot model to retrieve joint names
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description", false);
  moveit_robot_model_ = robot_model_loader.getModel();
  if (!moveit_robot_model_){
    ROS_FATAL_STREAM("Could not load robot model. Exiting.");
    exit(0);
  }
  moveit_robot_state_ = std::make_shared<robot_state::RobotState>(moveit_robot_model_);
  const robot_state::JointModelGroup* group = moveit_robot_state_->getJointModelGroup(move_group_name_);
  if (!group) {
    ROS_FATAL_STREAM("Could not find move group with name '" << move_group_name_ << "'. Exiting.");
    exit(0);
  }
  joint_names_ = group->getJointModelNames();
  if (joint_names_.size() != 2) {
    ROS_FATAL_STREAM("The move group '" << move_group_name_ << "' does not contain the correct amount of joints. Expected: 2; Found: " << joint_names_.size() << ". Exiting.");
    exit(0);
  }
  for (unsigned int i = 0; i < joint_names_.size(); ++i){
    ROS_INFO("[cam joint ctrl] Joint %d : %s", static_cast<int>(i), joint_names_[i].c_str());
  }
}

void CamJointTrajControl::getJointNamesFromController(ros::NodeHandle& nh)
{
  ros::ServiceClient query_joint_traj_state_client = nh.serviceClient<control_msgs::QueryTrajectoryState>("query_state");

  ROS_INFO ("Trying to retrieve state for controller namespace: %s", controller_nh_.getNamespace().c_str());

  bool retrieved_names = false;
  control_msgs::QueryTrajectoryState::Response queried_joint_traj_state;

  do{
    if (query_joint_traj_state_client.waitForExistence(ros::Duration(2.0))){
      control_msgs::QueryTrajectoryState srv;

      ros::Duration(1.0).sleep();

      // Hack to make this work in sim (otherwise time might be 0)
      //sleep(1);
      transform_listener_->waitForTransform("odom", default_look_dir_frame_, ros::Time(0), ros::Duration(2.0));

      srv.request.time = ros::Time::now();

      if (query_joint_traj_state_client.call(srv)){
        queried_joint_traj_state = srv.response;
        retrieved_names = true;


        ROS_INFO("[cam joint ctrl] Retrieved joint names: ");
        for (size_t i = 0; i < queried_joint_traj_state.name.size(); ++i){
          ROS_INFO("[cam joint ctrl] Joint %d : %s", static_cast<int>(i), queried_joint_traj_state.name[i].c_str());
        }
        joint_names_ = queried_joint_traj_state.name;

      }
    }else{
      ROS_ERROR("Could not retrieve controller state (and joint names), continuing to try.");
    }

  }while (!retrieved_names);
}

// Reset
void CamJointTrajControl::Reset()
{
  // Reset orientation
  latest_orientation_cmd_.reset();
}

bool CamJointTrajControl::planAndMoveToPoint(const geometry_msgs::PointStamped& point, double velocity_scaling_factor)
{
  moveit_msgs::GetMotionPlanRequest req;
  moveit_msgs::GetMotionPlanResponse res;

  moveit_msgs::VisibilityConstraint visibility;
  visibility.cone_sides = 4;

  visibility.max_view_angle = 0.0;
  visibility.max_range_angle = M_PI * 5.0 / 180.0;
  visibility.sensor_view_direction = moveit_msgs::VisibilityConstraint::SENSOR_X;
  visibility.weight = 1.0;
  visibility.target_radius = 0.05;

  visibility.target_pose.header.frame_id = point.header.frame_id;
  visibility.sensor_pose.header.frame_id = lookat_frame_;

  visibility.sensor_pose.pose.position.x = 0.1;
  visibility.sensor_pose.pose.orientation.w = 1.0;


  visibility.target_pose.pose.position = point.point;
  visibility.target_pose.pose.orientation.w = 1.0;

  moveit_msgs::Constraints constraints;
  constraints.name = "visibility";
  constraints.visibility_constraints.push_back(visibility);

  moveit_msgs::JointConstraint joint_constraint;
  joint_constraint.position = 0.0;
  joint_constraint.tolerance_above = M_PI;
  joint_constraint.tolerance_below = M_PI;
  joint_constraint.weight = 0.5;

  for (size_t i = 0; i < joint_names_.size(); ++i){
    joint_constraint.joint_name = joint_names_[i];
    constraints.joint_constraints.push_back(joint_constraint);
  }

  req.motion_plan_request.group_name = move_group_name_;
  req.motion_plan_request.goal_constraints.push_back(constraints);
  req.motion_plan_request.allowed_planning_time = 0.2;
  req.motion_plan_request.max_velocity_scaling_factor = velocity_scaling_factor;

  bool plan_retrieval_success = this->get_plan_service_client_.call(req, res);

  if (!plan_retrieval_success){
    ROS_WARN("[cam joint ctrl] Planning service returned false, aborting");
    return false;
  }

  if (!(res.motion_plan_response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) ){
    ROS_WARN("[cam joint ctrl] Plan result error code is %d, aborting.", static_cast<int>(res.motion_plan_response.error_code.val));
    return false;
  }

  ROS_DEBUG("[cam joint ctrl] Plan retrieved successfully");


  control_msgs::FollowJointTrajectoryGoal goal;

  goal.goal_time_tolerance = ros::Duration(1.0);

  goal.trajectory = res.motion_plan_response.trajectory.joint_trajectory;
  //goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.025);
  goal.trajectory.header.stamp = ros::Time::now();

  last_plan_time_ = ros::Time::now();

  gh_list_.push_back(joint_traj_client_->sendGoal(goal, boost::bind(&CamJointTrajControl::transitionCb, this, _1)));

  return true;
}

bool CamJointTrajControl::ComputeDirectionForPoint(const geometry_msgs::PointStamped& lookat_point, geometry_msgs::QuaternionStamped& orientation)
{
  geometry_msgs::PointStamped lookat_camera;

  tf::StampedTransform base_camera_transform;

  try {
    transform_listener_->waitForTransform(robot_link_reference_frame_, lookat_point.header.frame_id, ros::Time(), ros::Duration(1.0));
    transform_listener_->transformPoint(robot_link_reference_frame_, ros::Time(), lookat_point, lookat_point.header.frame_id, lookat_camera);
  } catch (std::runtime_error& e) {
    ROS_WARN("Could not transform look_at position to target frame_id %s", e.what());
    return false;
  }

  try {
    transform_listener_->waitForTransform(robot_link_reference_frame_, lookat_frame_, ros::Time(), ros::Duration(1.0));
    transform_listener_->lookupTransform(robot_link_reference_frame_, lookat_frame_, ros::Time(), base_camera_transform);
  } catch (std::runtime_error& e) {
    ROS_WARN("Could not transform from base frame to camera_frame %s", e.what());
    return false;
  }

  orientation.header = lookat_camera.header;
  orientation.header.frame_id = robot_link_reference_frame_;
  tf::Vector3 dir(lookat_camera.point.x - base_camera_transform.getOrigin().x(), lookat_camera.point.y - base_camera_transform.getOrigin().y(), lookat_camera.point.z - base_camera_transform.getOrigin().z());
  tf::Quaternion quaternion = tf::createQuaternionFromRPY(0.0, -atan2(dir.z(), sqrt(dir.x()*dir.x() + dir.y()*dir.y())), atan2(dir.y(), dir.x()));

  //if (last_orientation.length2() == 0.0 || quaternion.angle(last_orientation) >= update_min_angle) {
  tf::quaternionTFToMsg(quaternion, orientation.quaternion);
  //  orientationPub.publish(orientation);
  //  last_orientation = quaternion;
  //}
  return true;
}

void CamJointTrajControl::ComputeAndSendJointCommand(const geometry_msgs::QuaternionStamped& command_to_use)
{
  tf::StampedTransform transform;

  try{
    transform_listener_->lookupTransform(robot_link_reference_frame_, command_to_use.header.frame_id, ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_WARN("Failed to transform, not sending command to joints: %s",ex.what());
    return;
  }

  rotation_ = Eigen::Quaterniond(command_to_use.quaternion.w, command_to_use.quaternion.x, command_to_use.quaternion.y, command_to_use.quaternion.z);

  Eigen::Quaterniond quat(transform.getRotation().getW(), transform.getRotation().getX(),transform.getRotation().getY(),transform.getRotation().getZ());

  //stab_transform.getBasis().setRPY(roll, pitch, 0.0);
  //std::cout << "\nquat rot\n" << rotation_.matrix() << "\nquat:\n" << quat.matrix() << "\n";

  rotation_ = quat * rotation_;

  double temp[5];

  switch(rotationConv)  {
    case zyx:
      temp[0] =  2*(rotation_.x()*rotation_.y() + rotation_.w()*rotation_.z());
      temp[1] =     rotation_.w()*rotation_.w() + rotation_.x()*rotation_.x() - rotation_.y()*rotation_.y() - rotation_.z()*rotation_.z();
      temp[2] = -2*(rotation_.x()*rotation_.z() - rotation_.w()*rotation_.y());
      temp[3] =  2*(rotation_.y()*rotation_.z() + rotation_.w()*rotation_.x());
      temp[4] =     rotation_.w()*rotation_.w() - rotation_.x()*rotation_.x() - rotation_.y()*rotation_.y() + rotation_.z()*rotation_.z();
      break;

    case xyz:
      temp[0] =  -2*(rotation_.y()*rotation_.z() - rotation_.w()*rotation_.x());
      temp[1] =      rotation_.w()*rotation_.w() - rotation_.x()*rotation_.x() - rotation_.y()*rotation_.y() + rotation_.z()*rotation_.z();
      temp[2] =   2*(rotation_.x()*rotation_.z() + rotation_.w()*rotation_.y());
      temp[3] =  -2*(rotation_.x()*rotation_.y() - rotation_.w()*rotation_.z());
      temp[4] =      rotation_.w()*rotation_.w() + rotation_.x()*rotation_.x() - rotation_.y()*rotation_.y() - rotation_.z()*rotation_.z();
      break;

    default:
      ROS_ERROR("Invalid joint rotation convention!");
      break;
  }

  Eigen::Vector3d desAngle(
    atan2(temp[0], temp[1]),
    asin(temp[2]),
    atan2(temp[3], temp[4]));




  tf::StampedTransform current_state_transform;
  try{
    transform_listener_->lookupTransform(robot_link_reference_frame_, lookat_frame_, ros::Time(0), current_state_transform);
  }
  catch (tf::TransformException ex){
    ROS_WARN("Failed to transform, not sending command to joints: %s",ex.what());
    return;
  }

  double roll, pitch, yaw;
  current_state_transform.getBasis().getRPY(roll, pitch, yaw);

  double error_yaw, error_pitch, error_roll;
  double diff_1, diff_2;

  switch(rotationConv)  {
    case zyx:

      error_yaw = angles::shortest_angular_distance(desAngle[0], yaw);
      error_pitch = angles::shortest_angular_distance(desAngle[1], pitch);
      diff_1 = error_yaw;
      diff_2 = error_pitch;


      break;

    case xyz:

      error_pitch = angles::shortest_angular_distance(desAngle[0], pitch);
      error_roll = angles::shortest_angular_distance(desAngle[1], yaw);
      diff_1 = error_pitch;
      diff_2 = error_roll;

      break;

    default:
      ROS_ERROR("Invalid joint rotation convention!");
      break;
  }

  if (control_mode_ == MODE_LOOKAT && lookat_oneshot_){
    ROS_WARN_THROTTLE(5.0,"Lookat with return not implemented yet!");
    //look_at_server_->setSucceeded();
    //control_mode_ = MODE_OFF;

  }

  //Eigen::Vector3d angles = rotation_.matrix().eulerAngles(2, 0, 2);
  //std::cout << "\nangles:\n" << angles << "\n";
  //std::cout << "\nangles:\n" << desAngle << "\n" << "roll_diff: " << roll << " pitch_diff: " <<  pitch << " yaw diff:" << yaw << "\n";
  //std::cout << "\nangles:"  << " pitch_diff: " <<  error_pitch << " yaw diff:" << error_yaw << "\n";

  if (!use_direct_position_commands_){
      
    control_msgs::FollowJointTrajectoryGoal goal;
      
    goal.goal_time_tolerance = ros::Duration(1.0);
    //goal.goal.path_tolerance = 1.0;
    //goal.goal.trajectory.joint_names

    size_t num_joints = joint_names_.size();

    goal.trajectory.joint_names = joint_names_;
    goal.trajectory.points.resize(1);

    goal.trajectory.points[0].positions.resize(num_joints);
    goal.trajectory.points[0].velocities.resize(num_joints);
    goal.trajectory.points[0].accelerations.resize(num_joints);

    for (size_t i = 0; i < num_joints; ++i){
      goal.trajectory.points[0].positions[i] = desAngle[i];
      goal.trajectory.points[0].velocities[i] = 0.0;
      goal.trajectory.points[0].accelerations[i] = 0.0;
    }

    if (max_axis_speed_ != 0.0){
      double max_diff = 0.0;
      
      if (num_joints == 1){
        max_diff = std::abs(diff_1);
      }else{
        max_diff = std::max(std::abs(diff_1), std::abs(diff_2));  
      }
    
      double target_time = max_diff / max_axis_speed_;
      
      target_time = std::max (target_time, command_goal_time_from_start_);
      
      goal.trajectory.points[0].time_from_start = ros::Duration(target_time);
      
    }else{      
      goal.trajectory.points[0].time_from_start = ros::Duration(command_goal_time_from_start_);
    }

    //latest_gh_ = joint_traj_client_->sendGoal(goal);
    //boost::bind(&CamJointTrajControl::doneCb, this, _1, _2));

    gh_list_.push_back(joint_traj_client_->sendGoal(goal, boost::bind(&CamJointTrajControl::transitionCb, this, _1)));
  }else{
    servo_pub_1_.publish(desAngle[0]);
    servo_pub_2_.publish(desAngle[1]);
  }
}

// NEW: Store the velocities from the ROS message
void CamJointTrajControl::cmdCallback(const geometry_msgs::QuaternionStamped::ConstPtr& cmd_msg)
{
  if (!joint_trajectory_preempted_){
    joint_trajectory_preempted_ = true;
    ROS_INFO("[cam joint ctrl] Preempted running goal by orientation command.");
  }

  latest_orientation_cmd_ = cmd_msg;
  control_mode_ = MODE_ORIENTATION;
}

void CamJointTrajControl::controlTimerCallback(const ros::TimerEvent& event)
{
  // If we got preempted, do nothing
  if (!joint_trajectory_preempted_){

    if (control_mode_ == MODE_LOOKAT){

      //lookat_oneshot_ = true;


      if (!new_goal_received_){
        if (lookat_oneshot_ && !use_direct_position_commands_){
          std::list<actionlib::ClientGoalHandle<control_msgs::FollowJointTrajectoryAction> >::iterator it = gh_list_.begin();

          while  (it != gh_list_.end()){

            if (it->getCommState() == actionlib::CommState::ACTIVE ){
              ROS_DEBUG("Controller active, waiting to return");
              return;
            }
            it++;
          }
        }else{
          if ((ros::Time::now() - last_plan_time_).toSec() < 0.8){
            ROS_DEBUG("Replanning with fixed rate, skipping current iteration");
            return;
          }

        }
      }

      new_goal_received_ = false;
      
      
      if (use_planning_based_pointing_){

        ros::Rate rate (10);

        for (size_t i = 0; i < 5; ++i)
        {
          if (this->planAndMoveToPoint(this->lookat_point_))
          {
            if (lookat_oneshot_)
              control_mode_ = MODE_OFF;
            return;
          }
          rate.sleep();
        }

        if (look_at_server_->isActive()){
          ROS_INFO("Attempting to plan lookat failed multiple times, aborting.");
          look_at_server_->setAborted();
          control_mode_ = MODE_OFF;
          this->stopControllerTrajExecution();
        }
      }else{
        geometry_msgs::QuaternionStamped command_quat;
        this->ComputeDirectionForPoint(this->lookat_point_, command_quat);
        this->ComputeAndSendJointCommand(command_quat);
        return;
      }



    }else if (control_mode_ == MODE_PATTERN){
      ros::Time now = ros::Time::now();
      ROS_DEBUG("In pattern mode");

      if ((gh_list_.size() == 0) && (now > pattern_switch_time_)){
        ROS_INFO("Planning to next target point with index %d", (int) pattern_index_);
          

        const std::vector<TargetPointPatternElement>& curr_pattern = patterns_.at(current_pattern_name_);
        const TargetPointPatternElement& curr_target = curr_pattern[pattern_index_];

        double velocity_scaling_factor = curr_pattern[pattern_index_].goto_velocity_factor;

        if (use_planning_based_pointing_){
          ros::Rate rate (10);

          for (size_t i = 0; i < 5; ++i)
          {
            if (this->planAndMoveToPoint(curr_target.target_point))
            {
              return;
            }
            rate.sleep();
          }

          ROS_WARN("Tried reaching wp 5 times and failed, switching to next one");
          pattern_index_ = (pattern_index_ + 1) % curr_pattern.size();
        }else{
          geometry_msgs::QuaternionStamped command_quat;
          this->ComputeDirectionForPoint(curr_target.target_point, command_quat);
          this->ComputeAndSendJointCommand(command_quat);
        }

        
        return;

      }else{
        ROS_DEBUG("Waiting in pattern mode");
        return;
      }

      //this->ComputeAndSendJointCommand(pattern_[pattern_index_].orientation);

    }else if (control_mode_ == MODE_OFF){
      //If set to stabilized in params, do it
      if (stabilize_default_look_dir_frame_){

        tf::StampedTransform stab_transform;
        try{
          transform_listener_->lookupTransform("odom", default_look_dir_frame_, ros::Time(0), stab_transform);
        }catch (tf::TransformException ex){
          ROS_WARN("Failed to perform stabilization, not sending command to joints: %s",ex.what());
          return;
        }

        double roll, pitch, yaw;
        stab_transform.getBasis().getRPY(roll, pitch, yaw);

        stab_transform.getBasis().setRPY(roll, pitch, 0.0);

        geometry_msgs::QuaternionStamped command_to_use;
        command_to_use.header.frame_id = default_look_dir_frame_;

        tf::quaternionTFToMsg(stab_transform.getRotation().inverse(), command_to_use.quaternion);
        this->ComputeAndSendJointCommand(command_to_use);
      }


    }else{
      ROS_WARN("In unknown control mode %d, not commanding joints.", (int)control_mode_);
    }
  }else{
    //Not in Action mode
    ROS_DEBUG("joint_trajectory_preempted_ true");

    if (control_mode_ == MODE_ORIENTATION){
      if (latest_orientation_cmd_.get()){
        this->ComputeAndSendJointCommand(*latest_orientation_cmd_);
      }
    }

  }
}

void CamJointTrajControl::directPointingTimerCallback(const ros::TimerEvent& event)
{
  control_mode_ = MODE_OFF;
  look_at_server_->setSucceeded();
}

void CamJointTrajControl::transitionCb(actionlib::ClientGoalHandle<control_msgs::FollowJointTrajectoryAction> gh)
{  

  std::list<actionlib::ClientGoalHandle<control_msgs::FollowJointTrajectoryAction> >::iterator it = gh_list_.begin();

  ROS_DEBUG("----------------Num ghs: %d --------------", (int)gh_list_.size());
  // Erase goal handles that are DONE

  //actionlib::CommState latest_comm_state;
  //actionlib::TerminalState latest_term_state;

  while  (it != gh_list_.end()){


    if (it->getCommState() == actionlib::CommState::DONE ){
      ROS_DEBUG("Commstate: %s  Terminalstate: %s", it->getCommState().toString().c_str(), it->getTerminalState().toString().c_str());
    }else{
      ROS_DEBUG("Commstate: %s", it->getCommState().toString().c_str());
    }

    //latest_comm_state = it->getCommState();


    if (it->getCommState() == actionlib::CommState::DONE ){
      /*
      if (it->getTerminalState() == actionlib::TerminalState::PREEMPTED){
        ROS_INFO("Preempted traj controller, aborting action");

        joint_trajectory_preempted_ = true;

        if (look_at_server_->isActive()){
          look_at_server_->setPreempted();
        }
      }
      */

      if ((gh_list_.size() == 1) && (it->getTerminalState() == actionlib::TerminalState::PREEMPTED) && !new_goal_received_)
      {
        ROS_INFO("[cam joint ctrl] Preempted by traj controller, aborting lookat");

        joint_trajectory_preempted_ = true;

        if (look_at_server_->isActive()){
          look_at_server_->setPreempted();
        }
        control_mode_ = MODE_OFF;
        it = gh_list_.erase(it);
        return;
      }

      it = gh_list_.erase(it);
    }else{
      ++it;
    }    
  }

  // If we're done and have not requested continuous replanning, set succeeded.
  if ((gh_list_.size() == 0) && lookat_oneshot_ && !new_goal_received_ && !(control_mode_ == MODE_PATTERN)){

    if (look_at_server_->isActive()){
      control_mode_ = MODE_OFF;
      look_at_server_->setSucceeded();
    }
  }else if((gh_list_.size() == 0) && control_mode_ == MODE_PATTERN){

    ROS_INFO("In pattern mode and handle complete, switching to next index");

    const std::vector<TargetPointPatternElement>& curr_pattern = patterns_.at(current_pattern_name_);
    //const TargetPointPatternElement& curr_target = curr_pattern[pattern_index_];

    pattern_index_ = (pattern_index_ + 1) % curr_pattern.size();
    pattern_switch_time_ = ros::Time::now() + curr_pattern[pattern_index_].stay_time;
  }
    

  // If there are no goal handles left after we erased the ones that are DONE,
  // this means our last sent one got preempted by the server as someone else
  // took control of the server. In that case, cease sending new actions.

  /*
  if (gh_list_.size() == 0){
    ROS_INFO("Current joint action command got preempted, cancelling sending commands.");
    joint_trajectory_preempted_ = true;

    if (look_at_server_->isActive()){
      look_at_server_->setPreempted();
    }
  }
  */

  // Uncomment below for debugging

  ROS_DEBUG("-------------");
  /*
  for (std::list<actionlib::ClientGoalHandle<control_msgs::FollowJointTrajectoryAction> >::iterator it = gh_list_.begin(); it != gh_list_.end(); ++it){


    //if (*it == gh){
    //  ROS_INFO("Same goal handle!");
    //}else{
    //  ROS_ERROR("Different goal handles!");
    //}


    if (it->getCommState() == actionlib::CommState::DONE ){
      ROS_INFO("Commstate: %s  Terminalstate: %s", it->getCommState().toString().c_str(), it->getTerminalState().getText().c_str());
    }else{
      ROS_INFO("Commstate: %s", it->getCommState().toString().c_str());
    }
    ROS_INFO("+++++++++");
  }
  ROS_INFO("-------------");
  */
}

void CamJointTrajControl::lookAtGoalCallback()
{
  actionlib::SimpleActionServer<hector_perception_msgs::LookAtAction>::GoalConstPtr goal = look_at_server_->acceptNewGoal();
  joint_trajectory_preempted_ = false;

  if (!goal->look_at_target.pattern.empty()){


    if (findPattern(goal->look_at_target.pattern)){
      ROS_INFO("Starting pattern %s", goal->look_at_target.pattern.c_str());

      // Setup pattern following
      current_pattern_name_ = goal->look_at_target.pattern;
      control_mode_ = MODE_PATTERN;
      pattern_index_ = 0;
      pattern_switch_time_ = ros::Time(0);

    }else{
      ROS_WARN("Unknown pattern name %s, not commanding joints!", goal->look_at_target.pattern.c_str());
      // Keep current mode for now
      control_mode_ = MODE_OFF;
    }
  }else{
    lookat_point_ = goal->look_at_target.target_point;
    lookat_oneshot_ = goal->look_at_target.no_continuous_tracking;
    control_mode_ = MODE_LOOKAT;

    if (use_direct_position_commands_){
      reached_lookat_target_timer_ = nh_.createTimer(
            ros::Duration(4.0),
            &CamJointTrajControl::directPointingTimerCallback,
            this,
            true,
            true);

    }

    new_goal_received_ = true;
  }
}


void CamJointTrajControl::lookAtPreemptCallback()
{
  ROS_INFO("[cam joint ctrl] Preempt Callback");
  look_at_server_->setPreempted();
  joint_trajectory_preempted_ = true;
  this->stopControllerTrajExecution();
  control_mode_ = MODE_OFF;
}

void CamJointTrajControl::trajActionStatusCallback(const actionlib_msgs::GoalStatusArrayConstPtr& msg)
{
  ROS_DEBUG("Action status callback");
  
  if (control_mode_ == MODE_PATTERN){
    for (size_t i = 0; i < msg->status_list.size(); ++i){

      const actionlib_msgs::GoalStatus& curr_status = msg->status_list[i];

      std::size_t found = curr_status.goal_id.id.find(this->pnh_.getNamespace());

      if (found != std::string::npos){
        continue;
      }

      // If we reach this, another client has an active goal, thus preempt pattern
      // which is waiting for sending next goal to controller
      if (curr_status.status == actionlib_msgs::GoalStatus::ACTIVE){
        control_mode_ = MODE_OFF;
        joint_trajectory_preempted_ = true;
        look_at_server_->setPreempted();
        ROS_INFO("Other client sent goal %s, preempting pattern", curr_status.goal_id.id.c_str());
      }else{
        ROS_DEBUG("Other goal with status %d", (int)curr_status.status);
      }

    }
  }
}

void CamJointTrajControl::stopControllerTrajExecution()
{
  if (gh_list_.size() > 0){
    ROS_INFO("[cam joint ctrl] Setting joint traj controller to cancelled");
    joint_traj_client_->cancelAllGoals();
    //gh_list_.rbegin()->setCanceled();
  }

  //joint_traj_client_->cancelAllGoals();
}

bool CamJointTrajControl::loadPatterns()
{
  ros::NodeHandle nh;

  hector_perception_msgs::CameraPatternInfo cam_pattern_info;

  patterns_.clear();

  XmlRpc::XmlRpcValue description;
  ros::Duration interval(default_interval_);

  if (!nh.getParamCached(patterns_param_, description))
  {
    ROS_ERROR("Could not find patterns param! Not loading any.");
      return false;
  }

  for(int i = 0; i < description.size(); ++i) {

    if (description[i].hasMember("name")){

      ROS_INFO("name: %s", std::string(description[i]["name"]).c_str());
    }else{
      ROS_ERROR("Pattern element has no name, skipping.");
      continue;
    }

    std::string name = std::string(description[i]["name"]);

    cam_pattern_info.pattern_names.push_back(name);

    std::vector<TargetPointPatternElement>& waypoint_vec = patterns_[name];

    for(int j = 0; j < description[i]["waypoints"].size(); ++j) {
      XmlRpc::XmlRpcValue& element_description = description[i]["waypoints"][j];

      TargetPointPatternElement element;

      element.target_point.header.frame_id  = std::string(element_description["frame_id"]);
      element.stay_time = ros::Duration(element_description["stay_time"]);
      element.goto_velocity_factor = element_description["goto_velocity_factor"];

      element.target_point.point.x = static_cast<double>(element_description["target_point"][0]);
      element.target_point.point.y = static_cast<double>(element_description["target_point"][1]);
      element.target_point.point.z = static_cast<double>(element_description["target_point"][2]);

      waypoint_vec.push_back(element);
    }
    if (waypoint_vec.size() == 0){
      ROS_ERROR("Camera pattern with no waypoints!");
    }

  }

  pattern_info_pub_.publish(cam_pattern_info);

 return true;
}


bool CamJointTrajControl::findPattern(const std::string& pattern_name)
{

  std::map<std::string, std::vector<TargetPointPatternElement> >::iterator found_pattern = patterns_.find(pattern_name);

  if (found_pattern != patterns_.end())
    return true;

  ROS_ERROR("Pattern %s not found in %s!", pattern_name.c_str(), patterns_param_.c_str());
  return false;
}

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, ROS_PACKAGE_NAME);

    cam_control::CamJointTrajControl cc;

    ros::spin();

    return 0;
}
