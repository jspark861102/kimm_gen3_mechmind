#ifndef TCP_TEST_TCP_CLIENT_H_
#define TCP_TEST_TCP_CLIENT_H_

#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string>
#include <iostream>
#include <thread>
#include <mutex>

#include "ros/ros.h"
#include "tcp_ros/coordinate.h"
#include "status_code.h"

#include "std_msgs/String.h"
#include "tcp_ros/Feedback.h"
#include "tcp_ros/GetResult.h"
#include "tcp_ros/Recipe.h"
#include "tcp_ros/Trigger.h"
#define BUFFER_SIZE 1024

using namespace std;

class TcpClient : public Coordinate
{
public:
  TcpClient(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);
  std::mutex mutex_;

  virtual ~TcpClient();

  /**
   * \brief Initialize
   */
  bool init();

    /**
   * \brief Callback to send the message
   * \param msg Message
   */
  void sendMsgCallback(const std_msgs::String::ConstPtr& msg);
  void sendTriggerCallback(const tcp_ros::Trigger::ConstPtr& msg);
  void sendGetResultCallback(const tcp_ros::GetResult::ConstPtr& msg);
  void sendRecipeCallback(const tcp_ros::Recipe::ConstPtr& msg);
  void sendSystemStatusCallback(const std_msgs::String::ConstPtr& msg);

  /**
   * \brief receive the main loop
   */
  void receive();

private:
  /// ROS parameters
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  ros::Subscriber sub_data_msg_;
  ros::Subscriber sub_trigger_msg_;
  ros::Subscriber sub_getResult_msg_;
  ros::Subscriber sub_recipe_msg_;
  ros::Subscriber sub_status_call_;
  ros::Subscriber sub_pose_msg_;

  ros::Publisher pub_data_msg_;

  std::string ip_;
  int port_;

  struct sockaddr_in server_addr_, client_addr_;
  int client_fd_, addr_len_;
  int recv_len_, opt_;

  char buffer_[BUFFER_SIZE];
};

#endif  // TCP_TEST_TCP_CLIENT_H_