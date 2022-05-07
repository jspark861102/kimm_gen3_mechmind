#include "tcp_ros/tcp_server.h"

TcpServer::TcpServer(ros::NodeHandle &nh, ros::NodeHandle &nh_priv)
    : Coordinate(nh, nh_priv),
      nh_(nh),
      nh_priv_(nh_priv),
      port_(2000),
      server_fd_(0),
      client_fd_(0),
      addr_len_(0),
      recv_len_(1),
      opt_(1)
{
  nh_priv_.getParam("port", port_);

  sub_data_msg_ = nh_.subscribe("/send", 1, &TcpServer::sendMsgCallback, this);
  sub_trigger_msg_ = nh_.subscribe("/trigger", 1, &TcpServer::sendTriggerCallback, this);
  sub_getResult_msg_ = nh_.subscribe("/getResult", 1, &TcpServer::sendGetResultCallback, this);
  sub_recipe_msg_ = nh_.subscribe("/recipe", 1, &TcpServer::sendRecipeCallback, this);
  sub_status_call_ = nh_.subscribe("/systemStatusCall", 1, &TcpServer::sendSystemStatusCallback, this);

  pub_data_msg_ = nh_.advertise<std_msgs::String>("/received", 50);
}

TcpServer::~TcpServer()
{
  close(client_fd_);
  close(server_fd_);
  ROS_INFO("Closed socket");
}

bool TcpServer::init()
{
  if ((server_fd_ = socket(AF_INET, SOCK_STREAM, 0)) < 0)
  {
    ROS_ERROR("Failed to create socket");
    return false;
  }
  ROS_INFO("Created socket");

  if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt_, sizeof(opt_)))
  {
    ROS_ERROR("Failed to set sockopt");
    return false;
  }

  server_addr_.sin_family = AF_INET;
  server_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
  // server_addr_.sin_addr.s_addr = inet_addr(server_fd_);
  server_addr_.sin_port = htons(port_);

  if (bind(server_fd_, (struct sockaddr *)&server_addr_, sizeof(server_addr_)) < 0)
  {
    ROS_ERROR("Failed to bind");
    return false;
  }

  if (listen(server_fd_, 3) < 0)
  {
    ROS_ERROR("Failed to listen");
    return false;
  }

  ROS_INFO("Success TcpServer initialized...");

  return true;
}

void TcpServer::sendMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  mutex_.lock();
  std::string tmp = msg->data + "\n";
  send(client_fd_, tmp.c_str(), strlen(tmp.c_str()), 0);
  mutex_.unlock();
}

void TcpServer::sendTriggerCallback(const tcp_ros::Trigger::ConstPtr& msg)
{
  mutex_.lock();
  std::string tmp = "101," + msg->project + "," + msg->request + ",0" ",0"+ "\r" + "\n";
  send(client_fd_, tmp.c_str(), strlen(tmp.c_str()), 0);
  mutex_.unlock();
}

void TcpServer::sendGetResultCallback(const tcp_ros::GetResult::ConstPtr& msg)
{
  mutex_.lock();
  std::string tmp = "102," + msg->project + "\r" + "\n";
  send(client_fd_, tmp.c_str(), strlen(tmp.c_str()), 0);
  mutex_.unlock();
}

void TcpServer::sendRecipeCallback(const tcp_ros::Recipe::ConstPtr& msg)
{
  mutex_.lock();
  std::string tmp = "103," + msg->project + "," + msg->recipe + "\r" + "\n";
  send(client_fd_, tmp.c_str(), strlen(tmp.c_str()), 0);
  mutex_.unlock();

}

void TcpServer::sendSystemStatusCallback(const std_msgs::String::ConstPtr& msg)
{
  mutex_.lock();
  std::string tmp = "901\r\n";
  send(client_fd_, tmp.c_str(), strlen(tmp.c_str()), 0);
  mutex_.unlock();
}

bool TcpServer::run()
{
  addr_len_ = sizeof(client_addr_);
  if ((client_fd_ = accept(server_fd_, (struct sockaddr *)&client_addr_, (socklen_t *)&addr_len_)) < 0)
  {
    ROS_ERROR("Failed to accept");
    return false;
  }

  ROS_INFO("Connected with client. Client IP is %s", inet_ntoa(client_addr_.sin_addr));
  return true;
}

void TcpServer::receive()
{
  recv_len_ = 1;
  std_msgs::String recv_msg;
  vector<string> tcpData;

  run();

  do
  {
    recv_len_ = recv(client_fd_, buffer_, BUFFER_SIZE, 0);
    buffer_[recv_len_] = '\0';
    std::string msg(buffer_);

    recv_msg.data = msg.substr(0, recv_len_);

    pub_data_msg_.publish(recv_msg);

    tcpData.clear();
    if(parse(recv_msg.data, &tcpData))
    {
      systemStatus(tcpData);
      pose(tcpData);
    }

    ros::Rate(50.0).sleep();

  }while (recv_len_ > 0);
}

int main(int argc, char **argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "tcp_server_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  auto tcp = std::make_shared<TcpServer>(nh, nh_priv);

  if (tcp->init())
  {
    ros::AsyncSpinner spinner(1);
    spinner.start();

    thread ReceiveThread([&tcp]() -> void
                         { tcp->receive(); });

    while (ros::ok())
    {
      //std::cout << tcp->recv_msg.data << std::endl;
      //ros::spin();
    }

    spinner.stop();
  }

  return EXIT_FAILURE;
}