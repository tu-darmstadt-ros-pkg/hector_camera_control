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

#ifndef CAM_JOINT_TRAJ_CONTROL_H_
#define CAM_JOINT_TRAJ_CONTROL_H_

//#include <gazebo/common/Plugin.hh>
//#include <gazebo/common/Time.hh>
//#include <gazebo/math/Quaternion.hh>

// ROS 
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/JointState.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointControllerState.h>

#include <hector_perception_msgs/LookAtAction.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

namespace cam_control
{

struct TargetPointPatternElement{
  geometry_msgs::PointStamped target_point;
  ros::Duration stay_time;
  double goto_velocity_factor;
};


class CamJointTrajControl
{

public:
  CamJointTrajControl();
  virtual ~CamJointTrajControl();

protected:
  void Init();
  void Reset();

  void directPointingTimerCallback(const ros::TimerEvent& event);
  void controlTimerCallback(const ros::TimerEvent& event);

private:
  void getJointNamesFromMoveGroup();
  void getJointNamesFromController(ros::NodeHandle& nh);

  bool planAndMoveToPoint(const geometry_msgs::PointStamped& point, double velocity_scaling_factor = 1.0);
  bool ComputeDirectionForPoint(const geometry_msgs::PointStamped& lookat_point, geometry_msgs::QuaternionStamped& orientation);
  void ComputeAndSendJointCommand(const geometry_msgs::QuaternionStamped& command_to_use);

  void transitionCb(actionlib::ClientGoalHandle<control_msgs::FollowJointTrajectoryAction> gh);

  void lookAtGoalCallback();
  void lookAtPreemptCallback();

  void trajActionStatusCallback(const actionlib_msgs::GoalStatusArrayConstPtr& msg);

  void stopControllerTrajExecution();

  bool loadPatterns();

  bool findPattern(const std::string& pattern_name);

  void cmdCallback(const geometry_msgs::QuaternionStamped::ConstPtr& cmd_msg);

  unsigned int rotationConv;

  std::string controller_namespace_;
  std::string robot_link_reference_frame_;
  std::string lookat_frame_;

  std::string default_look_dir_frame_;
  bool stabilize_default_look_dir_frame_;

  double control_loop_period_;
  double command_goal_time_from_start_;
  double max_axis_speed_;
  double pitch_axis_factor_;

  bool joint_trajectory_preempted_;

  unsigned int control_mode_;

  geometry_msgs::PointStamped lookat_point_;
  bool lookat_oneshot_;

  // ROS STUFF
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::NodeHandle controller_nh_;

  ros::Timer control_timer;

  ros::Subscriber sub_;
  tf::TransformListener* transform_listener_;

  ros::Publisher pattern_info_pub_;

  ros::ServiceClient get_plan_service_client_;

  std::string move_group_name_;
  robot_model::RobotModelPtr moveit_robot_model_;
  robot_state::RobotStatePtr moveit_robot_state_;
  std::vector<std::string> joint_names_;

  geometry_msgs::QuaternionStamped::ConstPtr latest_orientation_cmd_;
  Eigen::Quaterniond rotation_;

  boost::shared_ptr<actionlib::ActionClient<control_msgs::FollowJointTrajectoryAction> > joint_traj_client_;
  ros::Subscriber joint_trajectory_action_status_sub_;

  std::shared_ptr<actionlib::SimpleActionServer<hector_perception_msgs::LookAtAction> > look_at_server_;

  std::list<actionlib::ClientGoalHandle<control_msgs::FollowJointTrajectoryAction> > gh_list_;

  ros::Time last_plan_time_;
  bool new_goal_received_;

  ros::Publisher servo_pub_1_;
  ros::Publisher servo_pub_2_;
  ros::Timer reached_lookat_target_timer_;

  std::map<std::string, std::vector<TargetPointPatternElement> > patterns_;

  unsigned int pattern_index_;
  std::string current_pattern_name_;

  ros::Duration default_interval_;

  std::string patterns_param_;
  ros::Time pattern_switch_time_;

  bool use_direct_position_commands_;
  double direct_position_command_default_wait_time_;
  
  bool use_planning_based_pointing_;
  bool disable_orientation_camera_command_input_;
};

}

#endif
