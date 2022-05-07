#ifndef TCP_ROS__COORDINATE_H_
#define TCP_ROS__COORDINATE_H_

#include <string>
#include <bitset>
#include <vector>
#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "tcp_ros/parser.h"
#include "status_code.h"
#include "tcp_ros/Status.h"
#include "tcp_ros/Feedback.h"
#include "tcp_ros/Position.h"

using namespace std;

#define BUFFER_SIZE 1024
#define POSE_START_NUM 5
#define POSE_OFFSET_COUNT 8

class Coordinate : public Parser
{
public:
  Coordinate(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

  virtual ~Coordinate() = default;

  void systemStatus(vector<string> data);
  void pose(vector<string> data);
  void statusCodeINFO(int cmd,int code);
  double stod_(string str);
  int stoi_(string str);

private:
  /// ROS parameters
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  char buffer_[BUFFER_SIZE];

  ros::Publisher pub_status_msg_;
  ros::Publisher pub_position_msg_;

};

#endif  // TCP_ROS__COORDINATE_H_