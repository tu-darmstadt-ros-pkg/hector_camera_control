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
//#include <hector_gazebo_plugins/servo_plugin.h>
//#include <gazebo/common/Events.hh>
//#include <gazebo/physics/physics.hh>

//#include <sensor_msgs/JointState.h>

/*
#if (GAZEBO_MAJOR_VERSION > 1) || (GAZEBO_MINOR_VERSION >= 2)
  #define RADIAN Radian
#else
  #define RADIAN GetAsRadian
#endif
*/

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
{
  transform_listener_ = 0;

  this->Init();
}

// Destructor
CamJointTrajControl::~CamJointTrajControl()
{
  //event::Events::DisconnectWorldUpdateBegin(updateConnection);
  delete transform_listener_;
}

// Load the controller
void CamJointTrajControl::Init()
{
  pnh_ = ros::NodeHandle("~");
  nh_ = ros::NodeHandle("");
  // Get the world name.
  //world = _model->GetWorld();

  // default parameters
  //topicName = "drive";
  //jointStateName = "joint_states";
  //robotNamespace.clear();
  //controlPeriod = 0;
  //proportionalControllerGain = 8.0;
  //derivativeControllerGain = 0.0;
  //maximumVelocity = 0.0;
  //maximumTorque = 0.0;

  pnh_.getParam("controller_namespace", controller_namespace_);
  pnh_.getParam("control_loop_period", control_rate_);
  pnh_.getParam("robot_link_reference_frame", robot_link_reference_frame_);

  controller_nh_ = ros::NodeHandle(controller_namespace_);

  joint_traj_client_.reset(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(controller_nh_.getNamespace() + "/follow_joint_trajectory", true));

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
      sleep(1);

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


  //pnh_.getParam("camera_frame_id", _camera_frame_id);
  //pnh_.getParam("patterns_param", patterns_param_);

  // load parameters
  /*
  if (_sdf->HasElement("robotNamespace")) robotNamespace = _sdf->Get<std::string>("robotNamespace");
  if (_sdf->HasElement("topicName")) topicName = _sdf->Get<std::string>("topicName");
  if (_sdf->HasElement("jointStateName")) jointStateName = _sdf->Get<std::string>("jointStateName");
  if (_sdf->HasElement("firstServoName")) servo[FIRST].name = _sdf->Get<std::string>("firstServoName");
  if (_sdf->HasElement("firstServoAxis")) servo[FIRST].axis = _sdf->Get<math::Vector3>("firstServoAxis");
  if (_sdf->HasElement("secondServoName")) servo[SECOND].name = _sdf->Get<std::string>("secondServoName");
  if (_sdf->HasElement("secondServoAxis")) servo[SECOND].axis = _sdf->Get<math::Vector3>("secondServoAxis");
  if (_sdf->HasElement("thirdServoName")) servo[THIRD].name = _sdf->Get<std::string>("thirdServoName");
  if (_sdf->HasElement("thirdServoAxis")) servo[THIRD].axis = _sdf->Get<math::Vector3>("thirdServoAxis");
  if (_sdf->HasElement("proportionalControllerGain")) proportionalControllerGain = _sdf->Get<double>("proportionalControllerGain");
  if (_sdf->HasElement("derivativeControllerGain")) derivativeControllerGain = _sdf->Get<double>("derivativeControllerGain");
  if (_sdf->HasElement("maxVelocity")) maximumVelocity = _sdf->Get<double>("maxVelocity");
  if (_sdf->HasElement("torque")) maximumTorque = _sdf->Get<double>("torque");
  */
  
  double controlRate = 20.0;
  //if (_sdf->HasElement("controlRate")) controlRate = _sdf->Get<double>("controlRate");
  controlPeriod = ros::Duration(controlRate > 0.0 ? 1.0/controlRate : 0.0);

  //servo[FIRST].joint  = _model->GetJoint(servo[FIRST].name);
  //servo[SECOND].joint = _model->GetJoint(servo[SECOND].name);
  //servo[THIRD].joint  = _model->GetJoint(servo[THIRD].name);

  //if (!servo[FIRST].joint)
  //  gzthrow("The controller couldn't get first joint");

  countOfServos = 1;
  /*
  if (servo[SECOND].joint) {
    countOfServos = 2;
    if (servo[THIRD].joint) {
      countOfServos = 3;
    }
  }
  else {
    if (servo[THIRD].joint) {
      gzthrow("The controller couldn't get second joint, but third joint was loaded");
    }
  }
  */

  /*
  if (!ros::isInitialized()) {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  }
  */



  transform_listener_ = new tf::TransformListener();
  //transform_listener_->setExtrapolationLimit(ros::Duration(1.0));

  //if (!topicName.empty()) {
  //ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::QuaternionStamped>("/camera/command", 1,
  //                                                                                             boost::bind(&CamJointTrajControl::cmdCallback, this, _1),
  //                                                                                             ros::VoidPtr(), &queue_);
  //sub_ = nh_.subscribe(so);
  sub_ = nh_.subscribe("/camera/command", 1, &CamJointTrajControl::cmdCallback, this);

  //}

  this->Reset();
  /*
  if (!jointStateName.empty()) {
    jointStatePub_ = rosnode_->advertise<sensor_msgs::JointState>(jointStateName, 10);
  }
  */

  //joint_state.header.frame_id = transform_listener_->resolve(_model->GetLink()->GetName());

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  //updateConnection = event::Events::ConnectWorldUpdateBegin(
  //    boost::bind(&CamJointTrajControl::Update, this));
}

/*
// Initialize the controller
void CamJointTrajControl::Init()
{
  Reset();
}
*/

// Reset
void CamJointTrajControl::Reset()
{
  // Reset orientation
  current_cmd.reset();

  enableMotors = true;

  servo[FIRST].velocity = 0;
  servo[SECOND].velocity = 0;
  servo[THIRD].velocity = 0;

  //prevUpdateTime = world->GetSimTime();
}

// Update the controller
void CamJointTrajControl::ComputeCommand()
{
  // handle callbacks
  //queue_.callAvailable();

  ros::Duration stepTime;
  stepTime = ros::Time::now() - prevUpdateTime;

  if (controlPeriod.toSec() == 0.0 || stepTime > controlPeriod) {
    CalculateVelocities();
    //publish_joint_states();
    prevUpdateTime = ros::Time::now();
  }

  if (enableMotors)
  {
    /*
    servo[FIRST].joint->SetVelocity(0, servo[FIRST].velocity);
    if (countOfServos > 1) {
      servo[SECOND].joint->SetVelocity(0, servo[SECOND].velocity);
      if (countOfServos > 2) {
        servo[THIRD].joint->SetVelocity(0, servo[THIRD].velocity);
      }
    }

    servo[FIRST].joint->SetMaxForce(0, maximumTorque);
    if (countOfServos > 1) {
      servo[SECOND].joint->SetMaxForce(0, maximumTorque);
      if (countOfServos > 2) {
        servo[THIRD].joint->SetMaxForce(0, maximumTorque);
      }
    }
  } else {
    servo[FIRST].joint->SetMaxForce(0, 0.0);
    if (countOfServos > 1) {
      servo[SECOND].joint->SetMaxForce(0, 0.0);
      if (countOfServos > 2) {
        servo[THIRD].joint->SetMaxForce(0, 0.0);
      }
    }
    */
  }
  
}

void CamJointTrajControl::CalculateVelocities()
{
  tf::StampedTransform transform;
  boost::mutex::scoped_lock lock(mutex);

  if(!current_cmd){
    geometry_msgs::QuaternionStamped *default_cmd = new geometry_msgs::QuaternionStamped;
    default_cmd->header.frame_id = "map";
    default_cmd->quaternion.w = 1;
    current_cmd.reset(default_cmd);
  }

  try{
    // ros::Time simTime(world->GetSimTime().sec, world->GetSimTime().nsec);
    transform_listener_->lookupTransform(robot_link_reference_frame_, current_cmd->header.frame_id, ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_WARN("%s",ex.what());
    servo[FIRST].velocity = 0.0;
    servo[SECOND].velocity = 0.0;
    servo[THIRD].velocity = 0.0;
    return;
  }

  //rotation_.Set(current_cmd->quaternion.w, current_cmd->quaternion.x, current_cmd->quaternion.y, current_cmd->quaternion.z);
  //rotation_ = Eigen::Quaterniond(current_cmd->quaternion.w, current_cmd->quaternion.x, current_cmd->quaternion.y, current_cmd->quaternion.z);

  //Eigen::Quaterniond quat(transform.getRotation().getW(),transform.getRotation().getX(),transform.getRotation().getY(),transform.getRotation().getZ());

  rotation_ = Eigen::Quaterniond(current_cmd->quaternion.x, current_cmd->quaternion.y, current_cmd->quaternion.z, current_cmd->quaternion.w);

  Eigen::Quaterniond quat(transform.getRotation().getX(),transform.getRotation().getY(),transform.getRotation().getZ(),transform.getRotation().getW());


  std::cout << "\nquat rot\n" << rotation_.matrix() << "\nquat:\n" << quat.matrix() << "\n";

  rotation_ = quat * rotation_;
  //rotation_ = rotation_ * quat;

  double temp[5];
  double desAngle[3];
  //double actualAngle[3] = {0.0, 0.0, 0.0};
  //double actualVel[3] = {0.0, 0.0, 0.0};

  //TODO use countOfServos for calculation
  rotationConv = 99;
  orderOfAxes[0] = 99;
  orderOfAxes[1] = 99;
  orderOfAxes[2] = 99;

  /*

  switch(countOfServos) {
    case 2:

      if ((servo[FIRST].axis.z == 1) && (servo[SECOND].axis.y == 1)) {
        rotationConv = zyx;
        orderOfAxes[0] = 0;
        orderOfAxes[1] = 1;
      }
      else {
        if ((servo[FIRST].axis.x == 1) && (servo[SECOND].axis.y == 1)) {
          rotationConv = xyz;
          orderOfAxes[0] = 0;
          orderOfAxes[1] = 1;
        }
      }
      break;

    case 3:

      if ((servo[FIRST].axis.z == 1) && (servo[SECOND].axis.y == 1) && (servo[THIRD].axis.x == 1)) {
        rotationConv = zyx;
        orderOfAxes[0] = 0;
        orderOfAxes[1] = 1;
        orderOfAxes[2] = 2;
      }
      else if ((servo[FIRST].axis.x == 1) && (servo[SECOND].axis.y == 1) && (servo[THIRD].axis.z == 1)) {
        rotationConv = xyz;
        orderOfAxes[0] = 0;
        orderOfAxes[1] = 1;
        orderOfAxes[2] = 2;
      }
      break;

    case 1:

      if (servo[FIRST].axis.y == 1) {
         rotationConv = xyz;
         orderOfAxes[0] = 1;
      }
      break;


    default:
      ROS_ERROR("Something went wrong. The count of servos is greater than 3");
      break;
  }
  */

  rotationConv = zyx;
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
      //gzthrow("joint order " << rotationConv << " not supported");
      break;
  }

  desAngle[0] = atan2(temp[0], temp[1]);
  desAngle[1] = asin(temp[2]);
  desAngle[2] = atan2(temp[3], temp[4]);

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.goal_time_tolerance = ros::Duration(1.0);
  //goal.goal.path_tolerance = 1.0;
  //goal.goal.trajectory.joint_names

  goal.trajectory.joint_names = this->latest_queried_joint_traj_state_.name;
  goal.trajectory.points.resize(1);

  goal.trajectory.points[0].positions.resize(2);

  goal.trajectory.points[0].positions[0] = desAngle[1];
  goal.trajectory.points[0].positions[1] = desAngle[0];

  goal.trajectory.points[0].velocities.resize(2);

  goal.trajectory.points[0].velocities[0] = 0.0;
  goal.trajectory.points[0].velocities[1] = 0.0;

  goal.trajectory.points[0].accelerations.resize(2);

  goal.trajectory.points[0].accelerations[0] = 0.0;
  goal.trajectory.points[0].accelerations[1] = 0.0;


  goal.trajectory.points[0].time_from_start = ros::Duration(1.0);

  joint_traj_client_->sendGoal(goal);
  //goal.goal.trajectory.points[0].push_back(desAngle[1]);

  /*
  actualAngle[FIRST] = servo[FIRST].joint->GetAngle(0).RADIAN();
  actualVel[FIRST] = servo[FIRST].joint->GetVelocity(0);
  ROS_DEBUG_NAMED("servo_plugin", "%s servo angle: %f - %f", servo[FIRST].name.c_str(), desAngle[orderOfAxes[FIRST]], actualAngle[FIRST]);
  servo[FIRST].velocity = ( proportionalControllerGain*(desAngle[orderOfAxes[FIRST]] - actualAngle[FIRST]) - derivativeControllerGain*actualVel[FIRST]);
  if (maximumVelocity > 0.0 && fabs(servo[FIRST].velocity) > maximumVelocity) servo[FIRST].velocity = (servo[FIRST].velocity > 0 ? maximumVelocity : -maximumVelocity);

  if (countOfServos > 1) {
    actualAngle[SECOND] = servo[SECOND].joint->GetAngle(0).RADIAN();
    actualVel[SECOND] = servo[SECOND].joint->GetVelocity(0);
    ROS_DEBUG_NAMED("servo_plugin", "%s servo angle: %f - %f", servo[SECOND].name.c_str(), desAngle[orderOfAxes[SECOND]], actualAngle[SECOND]);
    servo[SECOND].velocity = ( proportionalControllerGain*(desAngle[orderOfAxes[SECOND]] - actualAngle[SECOND]) - derivativeControllerGain*actualVel[SECOND]);
    if (maximumVelocity > 0.0 && fabs(servo[SECOND].velocity) > maximumVelocity) servo[SECOND].velocity = (servo[SECOND].velocity > 0 ? maximumVelocity : -maximumVelocity);

    if (countOfServos == 3) {
      actualAngle[THIRD] = servo[THIRD].joint->GetAngle(0).RADIAN();
      actualVel[THIRD] = servo[THIRD].joint->GetVelocity(0);
      ROS_DEBUG_NAMED("servo_plugin", "%s servo angle: %f - %f", servo[THIRD].name.c_str(), desAngle[orderOfAxes[THIRD]], actualAngle[THIRD]);
      servo[THIRD].velocity = ( proportionalControllerGain*(desAngle[orderOfAxes[THIRD]] - actualAngle[THIRD]) - derivativeControllerGain*actualVel[THIRD]);
      if (maximumVelocity > 0.0 && fabs(servo[THIRD].velocity) > maximumVelocity) servo[THIRD].velocity = (servo[THIRD].velocity > 0 ? maximumVelocity : -maximumVelocity);
    }
  }
  */

  // Changed motors to be always on, which is probably what we want anyway
  enableMotors = true; //myIface->data->cmdEnableMotors > 0;
}

// NEW: Store the velocities from the ROS message
void CamJointTrajControl::cmdCallback(const geometry_msgs::QuaternionStamped::ConstPtr& cmd_msg)
{
  boost::mutex::scoped_lock lock(mutex);
  current_cmd = cmd_msg;

  //this->ComputeCommand();
}

void CamJointTrajControl::jointTrajStateCb(const control_msgs::JointControllerState::ConstPtr& msg)
{
  this->latest_joint_traj_state_ = msg;
}

void CamJointTrajControl::controlTimerCallback(const ros::TimerEvent& event)
{
  //this->ComputeCommand();
  //ROS_INFO("control loop triggered");
  this->CalculateVelocities();
}


/*
void CamJointTrajControl::publish_joint_states()
{
  if (!jointStatePub_) return;

  joint_state.header.stamp.sec = (world->GetSimTime()).sec;
  joint_state.header.stamp.nsec = (world->GetSimTime()).nsec;

  joint_state.name.resize(countOfServos);
  joint_state.position.resize(countOfServos);
  joint_state.velocity.resize(countOfServos);
  joint_state.effort.resize(countOfServos);

  for (unsigned int i = 0; i < countOfServos; i++) {
    joint_state.name[i] = servo[i].joint->GetName();
    joint_state.position[i] = servo[i].joint->GetAngle(0).RADIAN();
    joint_state.velocity[i] = servo[i].joint->GetVelocity(0);
    joint_state.effort[i] = servo[i].joint->GetForce(0u);
  }

  jointStatePub_.publish(joint_state);
}
*/

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, ROS_PACKAGE_NAME);

    cam_control::CamJointTrajControl cc;
    //c.configure();

    ros::spin();

    //c.cleanup();

    //ros::shutdown();
    return 0;
}
