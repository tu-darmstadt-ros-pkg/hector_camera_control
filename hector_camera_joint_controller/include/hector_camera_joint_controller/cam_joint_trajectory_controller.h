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
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointControllerState.h>
#include <control_msgs/QueryTrajectoryState.h>

// Boost
//#include <boost/thread.hpp>
//#include <boost/bind.hpp>

namespace cam_control
{

class CamJointTrajControl
{

public:
  CamJointTrajControl();
  virtual ~CamJointTrajControl();

protected:
  //virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void Init();
  void Reset();
  //void ComputeCommand();

  void jointTrajStateCb(const control_msgs::JointControllerState::ConstPtr& msg);

  void controlTimerCallback(const ros::TimerEvent& event);

private:
  void CalculateVelocities();
  //void publish_joint_states();

private:

  // Simulation time of the last update
  //ros::Time prevUpdateTime;


  struct Servo {
    std::string name;
    Eigen::Vector3d axis;
    //physics::JointPtr joint;
    float velocity;
    Servo() : velocity() {}
  } servo[3];

  unsigned int countOfServos;
  unsigned int orderOfAxes[3];
  unsigned int rotationConv;

  ros::Duration controlPeriod;

  std::string controller_namespace_;
  std::string robot_link_reference_frame_;

  std::string default_look_dir_frame_;
  bool stabilize_default_look_dir_frame_;

  double control_rate_;
  double command_goal_time_from_start_;


  // ROS STUFF
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::NodeHandle controller_nh_;

  ros::Timer control_timer;

  //ros::Publisher jointStatePub_;
  ros::Subscriber sub_;
  tf::TransformListener* transform_listener_;

  void cmdCallback(const geometry_msgs::QuaternionStamped::ConstPtr& cmd_msg);

  //boost::mutex mutex;
  geometry_msgs::QuaternionStamped::ConstPtr current_cmd;
  control_msgs::JointControllerState::ConstPtr latest_joint_traj_state_;
  control_msgs::QueryTrajectoryState::Response latest_queried_joint_traj_state_;
  Eigen::Quaterniond rotation_;

  boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> > joint_traj_client_;
  ros::Subscriber joint_traj_state_sub_;

};

}

#endif
