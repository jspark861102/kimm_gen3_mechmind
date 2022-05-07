#include "tcp_ros/parser.h"

Parser::Parser(ros::NodeHandle &nh, ros::NodeHandle &nh_priv)
    : nh_(nh),
      nh_priv_(nh_priv),
      data_{}
{
}

bool Parser::parse(string msg, vector<string> *data)
{
  // "," 확인
  if (msg.find(",") == string::npos)
    return false;

  int previous = 0;
  int current = 0;

  data->clear();

  current = msg.find(',');

  while (current != string::npos)
  {
    string substring = msg.substr(previous, current - previous);

    data->push_back(substring);
    previous = current + 1;
    current = msg.find(',', previous);
  }

  data->push_back(msg.substr(previous, current - previous));

  return true;
}