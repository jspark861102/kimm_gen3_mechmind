#ifndef TCP_TEST_TCP_SERVER_H_
#define TCP_TEST_TCP_SERVER_H_

#include "unistd.h"
#include "sys/socket.h"
#include "errno.h"
#include "netinet/in.h"
#include "arpa/inet.h"
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

class TcpServer : public Coordinate
{
  
public:
  TcpServer(ros::NodeHandle &nh, ros::NodeHandle &nh_priv);
  std::mutex mutex_;

  virtual ~TcpServer();

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
   * \brief Initialize
   */
  bool init();  

  /**
   * \brief Run the main loop
   */
  bool run();
  //void send(string data);
  
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

  ros::Publisher pub_data_msg_;

  int port_;

  struct sockaddr_in server_addr_, client_addr_;

  int server_fd_, client_fd_, addr_len_;
  int recv_len_, opt_;

  char buffer_[BUFFER_SIZE];
};

#endif  // TCP_TEST_TCP_SERVER_H_
