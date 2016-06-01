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

namespace cam_control {

enum
{
  FIRST = 0, SECOND = 1, THIRD = 2
};

enum
{
  xyz, zyx
};

// Constructor
CamJointTrajControl::CamJointTrajControl()
  : joint_trajectory_preempted_(false)
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
  pnh_.getParam("command_goal_time_from_start", command_goal_time_from_start_);

  std::string type_string;
  pnh_.getParam("joint_order_type", type_string);

  if (type_string == "xyz"){
    rotationConv = xyz;
  }else{
    rotationConv = zyx;
  }

  transform_listener_ = new tf::TransformListener();

  controller_nh_ = ros::NodeHandle(controller_namespace_);

  joint_traj_client_.reset(new actionlib::ActionClient<control_msgs::FollowJointTrajectoryAction>(controller_nh_.getNamespace() + "/follow_joint_trajectory"));

  // Do not retrieve joint trajectory controller state for the moment
  //joint_traj_state_sub_ = controller_nh_.subscribe("state", 1, &CamJointTrajControl::jointTrajStateCb, this);

  control_timer = nh_.createTimer(ros::Duration(control_rate_), &CamJointTrajControl::controlTimerCallback, this, false, true);

  ros::ServiceClient query_joint_traj_state_client = controller_nh_.serviceClient<control_msgs::QueryTrajectoryState>("query_state");

  bool retrieved_names = false;

  ROS_INFO ("Trying to retrieve state for controller namespace: %s", controller_nh_.getNamespace().c_str());

  do{
    if (query_joint_traj_state_client.waitForExistence(ros::Duration(2.0))){
      control_msgs::QueryTrajectoryState srv;

      // Hack to make this work in sim (otherwise time might be 0)
      //sleep(1);
      transform_listener_->waitForTransform("map", default_look_dir_frame_, ros::Time(0), ros::Duration(2.0));


      srv.request.time = ros::Time::now();

      //ROS_ERROR("Time: %d, %d", srv.request.time.sec, srv.request.time.nsec);
      if (query_joint_traj_state_client.call(srv)){
        latest_queried_joint_traj_state_ = srv.response;
        retrieved_names = true;

        ROS_INFO("Retrieved joint names: ");
        for (size_t i = 0; i < latest_queried_joint_traj_state_.name.size(); ++i){
          ROS_INFO("Joint %d : %s", static_cast<int>(i), latest_queried_joint_traj_state_.name[i].c_str());
        }

      }
    }else{
      ROS_ERROR("Could not retrieve controller state (and joint names), continuing to try.");
    }

  }while (!retrieved_names);

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
  current_cmd.reset();
}

void CamJointTrajControl::CalculateVelocities()
{
  tf::StampedTransform transform;

  geometry_msgs::QuaternionStamped command_to_use;

  if(!current_cmd){
    command_to_use.header.frame_id = default_look_dir_frame_;
    command_to_use.quaternion.w = 1;
  }else{
    command_to_use = *current_cmd;
  }

  if (stabilize_default_look_dir_frame_){
    tf::StampedTransform stab_transform;
    try{
      transform_listener_->lookupTransform("map", default_look_dir_frame_, ros::Time(0), stab_transform);
    }catch (tf::TransformException ex){
      ROS_WARN("Failed to perform stabilization, not sending command to joints: %s",ex.what());
      return;
    }

    double roll, pitch, yaw;
    stab_transform.getBasis().getRPY(roll, pitch, yaw);

    stab_transform.getBasis().setRPY(roll, pitch, 0.0);

    tf::quaternionTFToMsg(stab_transform.getRotation().inverse(), command_to_use.quaternion);
  }

  try{
    transform_listener_->lookupTransform(robot_link_reference_frame_, command_to_use.header.frame_id, ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_WARN("Failed to transform, not sending command to joints: %s",ex.what());
    return;
  }

  rotation_ = Eigen::Quaterniond(command_to_use.quaternion.w, command_to_use.quaternion.x, command_to_use.quaternion.y, command_to_use.quaternion.z);

  Eigen::Quaterniond quat(transform.getRotation().getW(), transform.getRotation().getX(),transform.getRotation().getY(),transform.getRotation().getZ());


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

  //Eigen::Vector3d angles = rotation_.matrix().eulerAngles(2, 0, 2);

  //std::cout << "\nangles:\n" << angles << "\n";

  //std::cout << "\nangles:\n" << desAngle << "\n";

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
}

// NEW: Store the velocities from the ROS message
void CamJointTrajControl::cmdCallback(const geometry_msgs::QuaternionStamped::ConstPtr& cmd_msg)
{
  current_cmd = cmd_msg;
}

void CamJointTrajControl::jointTrajStateCb(const control_msgs::JointControllerState::ConstPtr& msg)
{
  this->latest_joint_traj_state_ = msg;
}

void CamJointTrajControl::controlTimerCallback(const ros::TimerEvent& event)
{
  if (!joint_trajectory_preempted_){
    this->CalculateVelocities();
  }
}

void CamJointTrajControl::transistionCb(actionlib::ClientGoalHandle<control_msgs::FollowJointTrajectoryAction> gh)
{  
  std::list<actionlib::ClientGoalHandle<control_msgs::FollowJointTrajectoryAction> >::iterator it = gh_list_.begin();

  // Erase goal handles that are DONE
  while  (it != gh_list_.end()){
    if (it->getCommState() == actionlib::CommState::DONE ){
      it = gh_list_.erase(it);
    }else{
      ++it;
    }
  }

  // If there are no goal handles left after we erased the ones that are DONE,
  // this means our last send one got preempted by the server as someone else
  // took control of the server. In that case, cease sending new actions.
  if (gh_list_.size() == 0){
    ROS_INFO("Current joint action command got preempted, cancelling sending commands.");
    joint_trajectory_preempted_ = true;

    if (look_at_server_->isActive()){
      look_at_server_->setPreempted();
    }
  }

  // Uncomment below for debugging
  /*
  ROS_INFO("-------------");
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
}


void CamJointTrajControl::lookAtPreemptCallback()
{
  look_at_server_->setPreempted();
  joint_trajectory_preempted_ = true;
}

/*
void CamJointTrajControl::lookAtGoalCallback(actionlib::ServerGoalHandle<hector_perception_msgs::LookAtAction> goal)
{

}
*/


/*
void CamJointTrajControl::lookAtPreemptCallback(actionlib::ServerGoalHandle<hector_perception_msgs::LookAtActionGoal> preempt)
{

}
*/
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, ROS_PACKAGE_NAME);

    cam_control::CamJointTrajControl cc;

    ros::spin();

    return 0;
}
