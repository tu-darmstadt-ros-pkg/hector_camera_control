#include <ros/ros.h>

#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <monstertruck_msgs/LookAt.h>
#include <std_msgs/String.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <vector>

class CameraControl
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle priv;
  ros::Subscriber lookatSub;
  ros::Subscriber patternSub;
  ros::Publisher orientationPub;
  ros::ServiceServer lookatServer;
  tf::TransformListener tf;

  std::string _base_frame_id;
  std::string _camera_frame_id;
  ros::Duration default_interval_;
  std::string patterns_param_;
  ros::Duration lookat_period;

  ros::Timer pattern_timer;
  ros::Timer lookat_timer;
  typedef enum { MODE_OFF = 0, MODE_LOOKAT = 1, MODE_PATTERN = 2, MODE_OVERRIDE = 4 } Mode;
  unsigned int mode;

  tf::Quaternion last_orientation;
  double update_min_angle;

  struct PatternElement {
    geometry_msgs::QuaternionStamped orientation;
    ros::Duration interval;
  };
  std::vector<PatternElement> pattern_;
  unsigned int pattern_index_;

  geometry_msgs::PointStampedConstPtr lookat_point;

public:
  CameraControl()
    : priv("~")
    , _base_frame_id("base_link")
    , _camera_frame_id("camera_frame")
    , patterns_param_("camera/patterns")
    , mode(MODE_OFF)
  {
    lookatSub = nh.subscribe("camera/look_at", 1, &CameraControl::lookatCallback, this);
    patternSub = nh.subscribe("camera/pattern", 1, &CameraControl::patternCallback, this);
    orientationPub = nh.advertise<geometry_msgs::QuaternionStamped>("camera/command", 1);
    pattern_timer = nh.createTimer(ros::Duration(0.0), &CameraControl::patternTimerCallback, this, false, false);
    lookat_timer = nh.createTimer(ros::Duration(0.0), &CameraControl::lookatTimerCallback, this, false, false);

    lookatServer = nh.advertiseService("camera/look_at", &CameraControl::lookatServiceCallback, this);

    priv.getParam("base_frame_id", _base_frame_id);
    priv.getParam("camera_frame_id", _camera_frame_id);
    priv.getParam("patterns_param", patterns_param_);

    double look_at_period_double = 0.0;
    priv.getParam("look_at_period", look_at_period_double);
    lookat_period.fromSec(look_at_period_double);
    lookat_timer.setPeriod(lookat_period);

    std::string default_pattern;
    priv.getParam("default_pattern", default_pattern);
    if (!default_pattern.empty()) {
      loadPattern(default_pattern);
      switchMode(MODE_PATTERN);
    }

    double default_interval_double = 1.0;
    priv.getParam("default_interval", default_interval_double);
    default_interval_ = ros::Duration(default_interval_double);

    update_min_angle = 0.0;
    priv.getParam("update_min_angle", update_min_angle);
  }

protected:

  std::string getModeString(unsigned int mode) {
    std::string s;
    if (mode == MODE_OFF)
      return "OFF";
    if (mode & MODE_LOOKAT)
      s = "LOOKAT";
    if (mode & MODE_PATTERN)
      s = "PATTERN";
    if (mode & MODE_OVERRIDE)
      s = s + "+OVERRIDE";
    return s;
  }

  void switchMode(unsigned int new_mode) {
    if (mode == new_mode) return;

    pattern_timer.stop();
    lookat_timer.stop();

    if (new_mode & MODE_LOOKAT) {
      lookat_timer.start();
    }

    else if (new_mode & MODE_PATTERN) {
      pattern_timer.start();
      pattern_index_ = 0;
    }

    last_orientation = tf::Quaternion(0.0, 0.0, 0.0, 0.0);

    ROS_INFO("Switched camera mode from %s to %s", getModeString(mode).c_str(), getModeString(new_mode).c_str());
    mode = new_mode;
  }

  void lookatCallback(const geometry_msgs::PointStampedConstPtr& lookat_msg) {
    if (mode & MODE_OVERRIDE) return;
    lookat_point = lookat_msg;
    switchMode(MODE_LOOKAT);
  }

  bool lookatServiceCallback(monstertruck_msgs::LookAt::Request& request, monstertruck_msgs::LookAt::Response& response)
  {
    if (mode & MODE_OVERRIDE) return false;

    unsigned int previous_mode = mode;
    geometry_msgs::PointStampedConstPtr previous_lookat_point = lookat_point;
    ros::Duration previous_lookat_period = lookat_period;

    lookat_point.reset(new geometry_msgs::PointStamped(request.point));

    if (request.duration.isZero())
    {
      switchMode(MODE_LOOKAT);

    } else {
      switchMode(MODE_LOOKAT | MODE_OVERRIDE);
      ros::Time startTime = ros::Time::now();
      
      while(ros::ok() && (ros::Time::now() - startTime) < request.duration) {
        ros::spinOnce();
      }

      lookat_point = previous_lookat_point;
      lookat_period = previous_lookat_period;
      switchMode(previous_mode);
    }

    return true;
  }

  void lookatTimerCallback(const ros::TimerEvent& event) {
    if (!(mode & MODE_LOOKAT) || !lookat_point) {
      lookat_timer.stop();
      return;
    }

    if (lookat_period.isZero()) switchMode(MODE_OFF);

    geometry_msgs::PointStamped lookat_camera;
    tf::StampedTransform base_camera_transform;

    try {
      tf.waitForTransform(_base_frame_id, lookat_point->header.frame_id, ros::Time(), ros::Duration(1.0));
      tf.transformPoint(_base_frame_id, ros::Time(), *lookat_point, lookat_point->header.frame_id, lookat_camera);
    } catch (std::runtime_error& e) {
      ROS_WARN("Could not transform look_at position to target frame_id %s", e.what());
      return;
    }

    try {
      tf.waitForTransform(_base_frame_id, _camera_frame_id, ros::Time(), ros::Duration(1.0));
      tf.lookupTransform(_base_frame_id, _camera_frame_id, ros::Time(), base_camera_transform);
    } catch (std::runtime_error& e) {
      ROS_WARN("Could not transform from base frame to camera_frame %s", e.what());
      return;
    }

    geometry_msgs::QuaternionStamped orientation;
    orientation.header = lookat_camera.header;
    orientation.header.frame_id = "base_link";
    tf::Vector3 dir(lookat_camera.point.x - base_camera_transform.getOrigin().x(), lookat_camera.point.y - base_camera_transform.getOrigin().y(), lookat_camera.point.z - base_camera_transform.getOrigin().z());
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(0.0, -atan2(dir.z(), sqrt(dir.x()*dir.x() + dir.y()*dir.y())), atan2(dir.y(), dir.x()));

    if (last_orientation.length2() == 0.0 || quaternion.angle(last_orientation) >= update_min_angle) {
      tf::quaternionTFToMsg(quaternion, orientation.quaternion);
      orientationPub.publish(orientation);
      last_orientation = quaternion;
    }
  }

  void patternCallback(const std_msgs::StringConstPtr& pattern_msg) {
    if (mode & MODE_OVERRIDE) return;

    if (pattern_msg->data.empty()) {
      switchMode(MODE_OFF);
    } else {
      if (!loadPattern(pattern_msg->data)) {
        ROS_ERROR("Unknown pattern command: %s", pattern_msg->data.c_str());
        return;
      }
      switchMode(MODE_PATTERN);
    }
  }

  void patternTimerCallback(const ros::TimerEvent& event)
  {
    if (!(mode & MODE_PATTERN)) { pattern_timer.stop(); return; }
    if (pattern_.empty()) { switchMode(MODE_OFF); return; }
    if (pattern_index_ > pattern_.size()) pattern_index_ = 0;

    pattern_[pattern_index_].orientation.header.stamp = ros::Time::now();
    orientationPub.publish(pattern_[pattern_index_].orientation);

    if (!pattern_[pattern_index_].interval.isZero()) {
      pattern_index_ = (pattern_index_ + 1) % pattern_.size();
      pattern_timer.setPeriod(pattern_[pattern_index_].interval);
      pattern_timer.start();
    } else {
      switchMode(MODE_OFF);
    }
  }

  bool loadPattern(const std::string& pattern_name)
  {
    pattern_.clear();
    if (pattern_name.empty()) return true;

    XmlRpc::XmlRpcValue description;
    double pan = 0.0, tilt = 0.0;
    ros::Duration interval(default_interval_);
    PatternElement element;
    element.orientation.header.frame_id = _base_frame_id;

    if (!nh.getParamCached(patterns_param_, description)) return false;
    for(int i = 0; i < description.size(); ++i) {
      if (!description[i].hasMember("name") || description[i]["name"] != pattern_name) continue;
      if (!description[i].hasMember("pattern")) continue;

      ROS_INFO("Loaded pattern %s!", pattern_name.c_str());

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

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "camera_control");
  CameraControl camera_control;
  ros::spin();
  return 0;
}
