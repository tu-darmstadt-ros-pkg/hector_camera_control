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

  pnh_.getParam("controller_namespace", controller_namespace_);
  pnh_.getParam("control_loop_period", control_rate_);
  pnh_.getParam("default_direction_reference_frame", default_look_dir_frame_);
  pnh_.getParam("stabilize_default_direction_reference", stabilize_default_look_dir_frame_);
  pnh_.getParam("robot_link_reference_frame", robot_link_reference_frame_);

  lookat_frame_ = std::string("sensor_head_mount_link");
  pnh_.getParam("lookat_reference_frame", lookat_frame_);
  pnh_.getParam("command_goal_time_from_start", command_goal_time_from_start_);

  double default_interval_double = 1.0;
  pnh_.getParam("default_interval", default_interval_double);
  default_interval_ = ros::Duration(default_interval_double);

  patterns_param_ = std::string(pnh_.getNamespace()+"/patterns");
  pnh_.getParam("patterns_param", patterns_param_);

  std::string type_string;
  pnh_.getParam("joint_order_type", type_string);


  pnh_.getParam("use_direct_position_commands", use_direct_position_commands_);

  if (type_string == "xyz"){
    rotationConv = xyz;
  }else{
    rotationConv = zyx;
  }

  transform_listener_ = new tf::TransformListener();

  get_plan_service_client_ = pnh_.serviceClient<moveit_msgs::GetMotionPlan>("/plan_kinematic_path");

  controller_nh_ = ros::NodeHandle(controller_namespace_);

  if (use_direct_position_commands_){
    servo_pub_1_ = nh_.advertise<std_msgs::Float64>("servo1_command", 1);
    servo_pub_2_ = nh_.advertise<std_msgs::Float64>("servo2_command", 1);

  }else{
    joint_traj_client_.reset(new actionlib::ActionClient<control_msgs::FollowJointTrajectoryAction>(controller_nh_.getNamespace() + "/follow_joint_trajectory"));

    ros::ServiceClient query_joint_traj_state_client = controller_nh_.serviceClient<control_msgs::QueryTrajectoryState>("query_state");

    bool retrieved_names = false;

    ROS_INFO ("Trying to retrieve state for controller namespace: %s", controller_nh_.getNamespace().c_str());

    do{
      if (query_joint_traj_state_client.waitForExistence(ros::Duration(2.0))){
        control_msgs::QueryTrajectoryState srv;

        // Hack to make this work in sim (otherwise time might be 0)
        //sleep(1);
        transform_listener_->waitForTransform("odom", default_look_dir_frame_, ros::Time(0), ros::Duration(2.0));


        srv.request.time = ros::Time::now();

        //ROS_ERROR("Time: %d, %d", srv.request.time.sec, srv.request.time.nsec);
        if (query_joint_traj_state_client.call(srv)){
          latest_queried_joint_traj_state_ = srv.response;
          retrieved_names = true;

          ROS_INFO("[cam joint ctrl] Retrieved joint names: ");
          for (size_t i = 0; i < latest_queried_joint_traj_state_.name.size(); ++i){
            ROS_INFO("[cam joint ctrl] Joint %d : %s", static_cast<int>(i), latest_queried_joint_traj_state_.name[i].c_str());
          }

        }
      }else{
        ROS_ERROR("Could not retrieve controller state (and joint names), continuing to try.");
      }

    }while (!retrieved_names);
  }

  // Do not retrieve joint trajectory controller state for the moment
  //joint_traj_state_sub_ = controller_nh_.subscribe("state", 1, &CamJointTrajControl::jointTrajStateCb, this);

  control_timer = nh_.createTimer(ros::Duration(control_rate_), &CamJointTrajControl::controlTimerCallback, this, false, true);


  //double controlRate = 20.0;

  //controlPeriod = ros::Duration(controlRate > 0.0 ? 1.0/controlRate : 0.0);

  sub_ = nh_.subscribe("/camera/command", 1, &CamJointTrajControl::cmdCallback, this);

  this->Reset();

  //Sleep for short time to let tf receive messages
  ros::Duration(1.0).sleep();


  look_at_server_.reset(new actionlib::SimpleActionServer<hector_perception_msgs::LookAtAction>(pnh_, "look_at", 0, false));

  //look_at_server_->registerGoalCallback(boost::bind(&CamJointTrajControl::lookAtGoalCallback, this, _1));
  look_at_server_->registerGoalCallback(boost::bind(&CamJointTrajControl::lookAtGoalCallback, this));
  look_at_server_->registerPreemptCallback(boost::bind(&CamJointTrajControl::lookAtPreemptCallback, this));

  look_at_server_->start();
}

// Reset
void CamJointTrajControl::Reset()
{
  // Reset orientation
  latest_orientation_cmd_.reset();
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

  //geometry_msgs::QuaternionStamped orientation;
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

    goal.trajectory.joint_names = this->latest_queried_joint_traj_state_.name;
    goal.trajectory.points.resize(1);

    goal.trajectory.points[0].positions.resize(2);

    goal.trajectory.points[0].positions[0] = desAngle[0];
    goal.trajectory.points[0].positions[1] = desAngle[1];

    goal.trajectory.points[0].velocities.resize(2);

    goal.trajectory.points[0].velocities[0] = 0.0;
    goal.trajectory.points[0].velocities[1] = 0.0;

    goal.trajectory.points[0].accelerations.resize(2);

    goal.trajectory.points[0].accelerations[0] = 0.0;
    goal.trajectory.points[0].accelerations[1] = 0.0;

    goal.trajectory.points[0].time_from_start = ros::Duration(command_goal_time_from_start_);

    //latest_gh_ = joint_traj_client_->sendGoal(goal);
    //boost::bind(&CamJointTrajControl::doneCb, this, _1, _2));

    gh_list_.push_back(joint_traj_client_->sendGoal(goal, boost::bind(&CamJointTrajControl::transistionCb, this, _1)));
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

void CamJointTrajControl::jointTrajStateCb(const control_msgs::JointControllerState::ConstPtr& msg)
{
  this->latest_joint_traj_state_ = msg;
}

void CamJointTrajControl::controlTimerCallback(const ros::TimerEvent& event)
{
  // If we got preempted, do nothing
  if (!joint_trajectory_preempted_){

    if (control_mode_ == MODE_LOOKAT){

      //lookat_oneshot_ = true;


      if (!new_goal_received_){
        if (lookat_oneshot_){
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

      //this->ComputeDirectionForPoint(this->lookat_point_, command_quat);
      //this->ComputeAndSendJointCommand(command_quat);

      moveit_msgs::GetMotionPlanRequest req;
      moveit_msgs::GetMotionPlanResponse res;

      moveit_msgs::VisibilityConstraint visibility;
      visibility.cone_sides = 4;

      visibility.max_view_angle = 0.0;
      visibility.max_range_angle = M_PI * 5.0 / 180.0;
      visibility.sensor_view_direction = moveit_msgs::VisibilityConstraint::SENSOR_X;
      visibility.weight = 1.0;
      visibility.target_radius = 0.05;

      visibility.target_pose.header.frame_id = this->lookat_point_.header.frame_id;
      visibility.sensor_pose.header.frame_id = "arm_zoom_cam_link";

      visibility.sensor_pose.pose.position.x = 0.1;
      visibility.sensor_pose.pose.orientation.w = 1.0;


      visibility.target_pose.pose.position = this->lookat_point_.point;
      visibility.target_pose.pose.orientation.w = 1.0;
      //nh_combined_planner_.param<std::string>("camera_frame", cam_frame_, "arm_zoom_cam_link");
      //constraints_.visibility_constraints.begin()->sensor_pose.header.frame_id = cam_frame_;
      //marker_.header.frame_id = scene_->getPlanningFrame();

      //req.motion_plan_request.trajectory_constraints.constraints

      moveit_msgs::Constraints constraints;
      constraints.name = "visbility";
      constraints.visibility_constraints.push_back(visibility);

      moveit_msgs::JointConstraint joint_constraint;
      joint_constraint.joint_name = latest_queried_joint_traj_state_.name[0];
      joint_constraint.position = 0.0;
      joint_constraint.tolerance_above = M_PI;
      joint_constraint.tolerance_below = M_PI;
      joint_constraint.weight = 0.5;
      constraints.joint_constraints.push_back(joint_constraint);

      joint_constraint.joint_name = latest_queried_joint_traj_state_.name[1];
      constraints.joint_constraints.push_back(joint_constraint);

      req.motion_plan_request.group_name = "sensor_head_group";
      req.motion_plan_request.goal_constraints.push_back(constraints);
      req.motion_plan_request.allowed_planning_time = 0.2;

      bool plan_retrieval_success = this->get_plan_service_client_.call(req, res);

      if (!plan_retrieval_success){
        ROS_WARN("[cam joint ctrl] Planning service returned false, aborting");
        return;
      }

      if (!(res.motion_plan_response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) ){
        ROS_WARN("[cam joint ctrl] Plan result error code is %d, aborting.", (int)res.motion_plan_response.error_code.val);
        return;
      }

      ROS_DEBUG("[cam joint ctrl] Plan retrieved successfully");


      control_msgs::FollowJointTrajectoryGoal goal;

      goal.goal_time_tolerance = ros::Duration(1.0);

      //goal.trajectory.joint_names = this->latest_queried_joint_traj_state_.name;
      goal.trajectory = res.motion_plan_response.trajectory.joint_trajectory;
      //goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.025);
      goal.trajectory.header.stamp = ros::Time::now();

      last_plan_time_ = ros::Time::now();

      gh_list_.push_back(joint_traj_client_->sendGoal(goal, boost::bind(&CamJointTrajControl::transistionCb, this, _1)));

    }else if (control_mode_ == MODE_PATTERN){
      ros::Time now = ros::Time::now();

      if (now > pattern_switch_time_){
        pattern_index_ = (pattern_index_ + 1) % pattern_.size();
        pattern_switch_time_ = now + pattern_[pattern_index_].interval;
      }

      this->ComputeAndSendJointCommand(pattern_[pattern_index_].orientation);

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



    //this->ComputeAndSendCommand();
  }else{
    //Not in Action mode

    if (control_mode_ == MODE_ORIENTATION){
      if (latest_orientation_cmd_.get()){
        this->ComputeAndSendJointCommand(*latest_orientation_cmd_);
      }
    }

  }
}

void CamJointTrajControl::transistionCb(actionlib::ClientGoalHandle<control_msgs::FollowJointTrajectoryAction> gh)
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
      }

      it = gh_list_.erase(it);
    }else{
      ++it;
    }    
  }


  // If we're done and have not requested continuous replanning, set succeeded.
  if ((gh_list_.size() == 0) && lookat_oneshot_ && !new_goal_received_){

    if (look_at_server_->isActive()){
      control_mode_ = MODE_OFF;
      look_at_server_->setSucceeded();
    }
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
    if (this->loadPattern(goal->look_at_target.pattern)){

      // Setup pattern following
      control_mode_ = MODE_PATTERN;
      pattern_index_ = 0;
      pattern_switch_time_ = ros::Time::now() + pattern_[pattern_index_].interval;

    }else{
      ROS_WARN("Unknown pattern name, not commanding joints!");
      // Keep current mode for now
      //control_mode_ = MODE_OFF;
    }
  }else{
    lookat_point_ = goal->look_at_target.target_point;
    lookat_oneshot_ = goal->look_at_target.no_continuous_tracking;
    control_mode_ = MODE_LOOKAT;

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

void CamJointTrajControl::stopControllerTrajExecution()
{
  if (gh_list_.size() > 0){
    ROS_INFO("[cam joint ctrl] Setting controller to cancelled");
    joint_traj_client_->cancelAllGoals();
    //gh_list_.rbegin()->setCanceled();
  }

  //joint_traj_client_->cancelAllGoals();
}

bool CamJointTrajControl::loadPattern(const std::string& pattern_name)
{
  ros::NodeHandle nh;

  pattern_.clear();
  if (pattern_name.empty()) return true;

  XmlRpc::XmlRpcValue description;
  double pan = 0.0, tilt = 0.0;
  ros::Duration interval(default_interval_);
  PatternElement element;
  element.orientation.header.frame_id = this->robot_link_reference_frame_;

  if (!nh.getParamCached(patterns_param_, description)) return false;
  for(int i = 0; i < description.size(); ++i) {
    if (!description[i].hasMember("name") || description[i]["name"] != pattern_name) continue;
    if (!description[i].hasMember("pattern")) continue;

    ROS_INFO("[cam joint ctrl] Loaded pattern %s!", pattern_name.c_str());

    if (description[i].hasMember("frame_id")) element.orientation.header.frame_id = std::string(description[i]["frame_id"]);
    if (description[i].hasMember("interval")) interval = ros::Duration(description[i]["interval"]);

    for(int j = 0; j < description[i]["pattern"].size(); ++j) {
      XmlRpc::XmlRpcValue& element_description = description[i]["pattern"][j];

      element.interval = interval;
      if (element_description.hasMember("interval")) element.interval = ros::Duration(element_description["interval"]);

      if (element_description.hasMember("pan"))  pan  = element_description["pan"];
      if (element_description.hasMember("tilt")) tilt = element_description["tilt"];
      tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0.0, tilt * M_PI/180.0, pan * M_PI/180.0), element.orientation.quaternion);
      pattern_.push_back(element);
    }

    return !pattern_.empty();
  }

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
