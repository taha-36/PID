#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#define min(a, b) (((a) < (b)) ? (a): (b))

const int MAX_SPEED = 2;

enum Clamp { NO_CLAMP, CLAMP } clamp;

float get_time() {
  return millis() / 1000.0;
}

struct AxisRot {
  double yaw = 0, pitch = 0, roll = 0;
  AxisRot() {}
  AxisRot(double yaw, double pitch, double roll) : yaw(yaw), pitch(pitch), roll(roll) {}
  AxisRot(tf2::Quaternion q) {
    tf2::Matrix3x3 matrix(q);
    matrix.getRPY(roll, pitch, yaw);
  }
};

struct FetchData {
  tf2::Quaternion q;
  geometry_msgs::Point p;
  geometry_msgs::Vector3 linear;
  geometry_msgs::Vector3 angular;

  void fetch_data(const nav_msgs::Odometry::ConstPtr& msg) {
    q.setX(msg->pose.pose.orientation.x);
    q.setY(msg->pose.pose.orientation.y);
    q.setZ(msg->pose.pose.orientation.z);
    q.setW(msg->pose.pose.orientation.w);
    p.x = msg->pose.pose.position.x;
    p.y = msg->pose.pose.position.y;
    p.z = msg->pose.pose.position.z;
    linear.x = msg->twist.twist.linear.x;
    linear.y = msg->twist.twist.linear.y;
    linear.z = msg->twist.twist.linear.z;
    angular.x = msg->twist.twist.angular.x;
    angular.y = msg->twist.twist.angular.y;
    angular.z = msg->twist.twist.angular.z;
  }
};

struct Point {
  float x, y, theta;

  void FetchPoint(const geometry_msgs::Pose2D::ConstPtr& msg) {
    x = msg->x;
    y = msg->y;
    theta = msg->theta;
  }
};

struct PidParam {
  float kp, ki, kd;

  void FetchParam(const geometry_msgs::Vector3::ConstPtr& msg) {
    kp = msg->x;
    ki = msg->y;
    kd = msg->z;
  }
} param;

struct Pid {
private:
  float accum = 0;
  float prev_time_i = 0;
  float prev_time_d = 0;
  float prev_point = 0;
  float prev_output = 0;
  float prev_error = 0;

  void integrate(const float current) {
    float curr_time = get_time();
    this->accum += current * (curr_time - prev_time_i);
    prev_time_i = curr_time;
  }

  float differentiate(const float last_point) {
    float curr_time = get_time();
    float d = (prev_point - last_point) / (curr_time - prev_time_d);
    prev_time_d = curr_time;
    prev_point = last_point;
    return d;
  }

  Clamp check_clamping() {
    if (prev_error != 0 && prev_output != 0) {
      if (prev_output > MAX_SPEED && prev_error / prev_output > 0) {
        return CLAMP;
      }
    }
    return NO_CLAMP;
  }
};