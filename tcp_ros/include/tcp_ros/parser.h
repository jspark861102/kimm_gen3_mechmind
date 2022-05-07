#ifndef MI_ROS__PARSER_H_
#define MI_ROS__PARSER_H_

#include <string>
#include <bitset>
#include <vector>
#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace std;

#define BUFFER_SIZE 1024

class Parser
{
public:
  Parser(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

  virtual ~Parser() = default;

  /**
   * \brief Parse the message
   * \param msg Message read through serial communication
   */
  bool parse(string msg, vector<string> *data);

private:
  /// ROS parameters
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  const char data_[BUFFER_SIZE];
  
};

#endif  // TCP_ROS__PARSER_H_