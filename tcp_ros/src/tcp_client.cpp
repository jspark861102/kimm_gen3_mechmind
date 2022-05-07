#include "tcp_ros/tcp_client.h"

TcpClient::TcpClient(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
  : Coordinate(nh, nh_priv),
  nh_(nh),
  nh_priv_(nh_priv),
  ip_("192.168.0.1"), 
  port_(8000),
  client_fd_(0),
  addr_len_(0),
  recv_len_(0),
  opt_(1)
{
  nh_priv_.getParam("ip", ip_);
  nh_priv_.getParam("port", port_);

  sub_data_msg_ = nh_.subscribe("/send", 1, &TcpClient::sendMsgCallback, this);
  sub_trigger_msg_ = nh_.subscribe("/trigger", 1, &TcpClient::sendTriggerCallback, this);
  sub_getResult_msg_ = nh_.subscribe("/getResult", 1, &TcpClient::sendGetResultCallback, this);
  sub_recipe_msg_ = nh_.subscribe("/recipe", 1, &TcpClient::sendRecipeCallback, this);
  sub_status_call_ = nh_.subscribe("/systemStatusCall", 1, &TcpClient::sendSystemStatusCallback, this);

  pub_data_msg_ = nh_.advertise<std_msgs::String>("/received", 50);
}

void TcpClient::sendMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  mutex_.lock();
  std::string tmp = msg->data + "\n";
  //send(client_socket_, tmp.c_str(), strlen(tmp.c_str()), 0);
  mutex_.unlock();
}

void TcpClient::sendTriggerCallback(const tcp_ros::Trigger::ConstPtr& msg)
{
  mutex_.lock();
  std::string tmp = "101," + msg->project + "," + msg->request + ",0" ",0"+ "\r" + "\n";
  send(client_fd_, tmp.c_str(), strlen(tmp.c_str()), 0);
  mutex_.unlock();
}

void TcpClient::sendGetResultCallback(const tcp_ros::GetResult::ConstPtr& msg)
{
  mutex_.lock();
  std::string tmp = "102," + msg->project + "\r" + "\n";
  send(client_fd_, tmp.c_str(), strlen(tmp.c_str()), 0);
  mutex_.unlock();
}

void TcpClient::sendRecipeCallback(const tcp_ros::Recipe::ConstPtr& msg)
{
  mutex_.lock();
  std::string tmp = "103," + msg->project + "," + msg->recipe + "\r" + "\n";
  send(client_fd_, tmp.c_str(), strlen(tmp.c_str()), 0);
  mutex_.unlock();

}

void TcpClient::sendSystemStatusCallback(const std_msgs::String::ConstPtr& msg)
{
  mutex_.lock();
  std::string tmp = "901\r\n";
  send(client_fd_, tmp.c_str(), strlen(tmp.c_str()), 0);
  mutex_.unlock();
}

TcpClient::~TcpClient()
{
  close(client_fd_);
  ROS_INFO("Closed socket");
}

bool TcpClient::init()
{
  if ((client_fd_ = socket(AF_INET, SOCK_STREAM, 0)) < 0)
  {
    ROS_ERROR("Failed to create socket");
    return false;
  }
  ROS_INFO("Created socket");

  server_addr_.sin_family = AF_INET;
  // server_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
  server_addr_.sin_addr.s_addr = inet_addr(ip_.c_str());
  server_addr_.sin_port = htons(port_);

  if (connect(client_fd_, (struct sockaddr*)&server_addr_, sizeof(server_addr_)) < 0)
  {
    ROS_ERROR("Failed to connect the server");
    return false;
  }

  return true;
}

void TcpClient::receive()
{
  recv_len_ = 1;
  std_msgs::String recv_msg;
  vector<string> tcpData;

  do
  {
    recv_len_ = recv(client_fd_, buffer_, BUFFER_SIZE, 0);
    buffer_[recv_len_] = '\0';
    std::string msg(buffer_);

    recv_msg.data = msg.substr(0, recv_len_);

    //std::cout << recv_msg.data << std::endl;

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
  ros::init(argc, argv, "tcp_clinet_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  auto tcp = std::make_shared<TcpClient>(nh, nh_priv);

  if (tcp->init())
  {
    ros::AsyncSpinner spinner(1);
    spinner.start();

    thread receiveThread([&tcp]() -> void
                         { tcp->receive(); });

    while (ros::ok())
    {
      // std::cout << tcp->recv_msg.data << std::endl;
      // ros::spin();
    }

    spinner.stop();
  }

  return EXIT_FAILURE;
}
