#include "tcp_ros/coordinate.h"

Coordinate::Coordinate(ros::NodeHandle &nh, ros::NodeHandle &nh_priv)
    : Parser(nh, nh_priv),
      nh_(nh),
      nh_priv_(nh_priv),
      buffer_{}
{
    pub_status_msg_ = nh_.advertise<tcp_ros::Status>("/systemStatus", 50);
    pub_position_msg_ = nh_.advertise<tcp_ros::Feedback>("/pose", 10);
}

void Coordinate::pose(vector<string> data)
{
    tcp_ros::Status statusCode_;
    tcp_ros::Feedback feedback_;

    int cmd = stoi_(data.at(0));
    int code = stoi_(data.at(1));

    try
    {
        if (code == NORMAL_STATE01)
        {
            if (cmd == 102)
            {
                feedback_.header.stamp = ros::Time::now();
                feedback_.header.frame_id = data.at(0);

                int size = stoi_(data.at(3));

                feedback_.position.resize(size);

                for (int i = 0; i < size; i++)
                {
                    feedback_.position[i].request = (int8_t)i;
                    feedback_.position[i].linear.x = stod_(data.at(POSE_START_NUM + (POSE_OFFSET_COUNT * i)));
                    feedback_.position[i].linear.y = stod_(data.at(POSE_START_NUM + (POSE_OFFSET_COUNT * i) + 1));
                    feedback_.position[i].linear.z = stod_(data.at(POSE_START_NUM + (POSE_OFFSET_COUNT * i) + 2));

                    feedback_.position[i].angular.x = stod_(data.at(POSE_START_NUM + (POSE_OFFSET_COUNT * i) + 3));
                    feedback_.position[i].angular.y = stod_(data.at(POSE_START_NUM + (POSE_OFFSET_COUNT * i) + 4));
                    feedback_.position[i].angular.z = stod_(data.at(POSE_START_NUM + (POSE_OFFSET_COUNT * i) + 5));
                    feedback_.position[i].label = (int8_t)stoi_(data.at(POSE_START_NUM + (POSE_OFFSET_COUNT * i) + 6));
                }
                pub_position_msg_.publish(feedback_);

                feedback_.position.clear();
            }
        }
        else
        {
           systemStatus(data);
        }

        ros::Rate(50.0).sleep();
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Type transformation data error");
        ROS_ERROR("%s ", e.what());
        //std::cerr << e.what() << '\n';
    }
}

  double Coordinate::stod_(string str)
  {
    double ret = 0.0;

      try
      {
          ret = stod(str);
      }
      catch (const std::exception &e)
      {
          ROS_ERROR("[%s] Type transformation data error", e.what());
          //std::cerr << e.what() << '\n';
          ret = 0.0;
      }
      return ret;

  }

  int Coordinate::stoi_(string str)
  {
      int ret = 0;

      try
      {
          ret = stoi(str);
      }
      catch (const std::exception &e)
      {
           ROS_ERROR("[%s] Type transformation data error", e.what());
          //std::cerr << e.what() << '\n';
          ret = 0;
      }
      return ret;
  }

void Coordinate::systemStatus(vector<string> data)
{
    tcp_ros::Status codeMsg_;

    int cmd = stoi_(data.at(0));
    int code = stoi_(data.at(1));

    statusCodeINFO(cmd, code);

    codeMsg_.command = (int8_t)cmd;
    codeMsg_.code = (int16_t)code;

    pub_status_msg_.publish(codeMsg_);
}

void Coordinate::statusCodeINFO(int cmd, int code)
{
    switch (code)
    {
    case ERROR01:
        ROS_ERROR("Commend: [%03d], ERROR: [%04d] The vision project is turn off", cmd, code);
        break;

    case ERROR02:
        ROS_ERROR("Commend: [%03d], ERROR: [%04d] Object exists but is not recognized", cmd, code);
        break;

    case ERROR03:
        ROS_ERROR("Commend: [%03d], ERROR: [%04d] There are no objects within the photographing area", cmd, code);
        break;

    case ERROR04:
        ROS_ERROR("Commend: [%03d], ERROR: [%04d] Send coordinate acquisition command during vision processing", cmd, code);
        break;

    case ERROR05:
        ROS_ERROR("Commend: [%03d], ERROR: [%04d] Vision project does not exist", cmd, code);
        break;

    case ERROR06:
        ROS_ERROR("Commend: [%03d], ERROR: [%04d] The recipe command value is outside the set value", cmd, code);
        break;

    case ERROR07:
        ROS_ERROR("Commend: [%03d], ERROR: [%04d] If there is no recipe value after the recipe command", cmd, code);
        break;

    case ERROR08:
        ROS_ERROR("Commend: [%03d], ERROR: [%04d] If you are having trouble setting up your recipe", cmd, code);
        break;

    case ERROR09:
        ROS_ERROR("Commend: [%03d], ERROR: [%04d] Vision issue (IPC capacity issue or logic anomaly)", cmd,code);
        break;

    case ERROR10:
        ROS_ERROR("Commend: [%03d], ERROR: [%04d] If there is a problem with the deep learning server (insufficient IPC RAM capacity or crash)", cmd, code);
        break;

    case ERROR11:
        ROS_ERROR("Commend: [%03d], ERROR: [%04d] If you are having trouble setting up a label", cmd, code);
        break;

    case ERROR12:
        ROS_ERROR("Commend: [%03d], ERROR: [%04d] The quantity requested is larger than the product quantity recognized by Vision.", cmd, code);
        break;

    case ERROR13:
        ROS_ERROR("Commend: [%03d], ERROR: [%04d] Vision calculation is performed for more than 5 minutes", cmd, code);
        break;

    case NORMAL_STATE01:
        ROS_INFO("Commend: [%03d], INFO: [%04d] Coordinate normal output", cmd, code);
        break;

    case NORMAL_STATE02:
        ROS_INFO("Commend: [%03d], INFO: [%04d] Status When requested, the camera is ready to shoot", cmd, code);
        break;

    case NORMAL_STATE03:
        ROS_INFO("Commend: [%03d], INFO: [%04d] Vision Trigger Received Successfully", cmd, code);
        break;

    case NORMAL_STATE04:
        ROS_INFO("Commend: [%03d], INFO: [%04d] Vision recipe setup successful", cmd, code);
        break;

    case COMMUNICATION_ERROR01:
        ROS_ERROR("Commend: [%03d], ERROR: [%04d] Invalid command", cmd, code);
        break;

    case COMMUNICATION_ERROR02:
        ROS_ERROR("Commend: [%03d], ERROR: [%04d] Client disconnected", cmd, code);
        break;

    case COMMUNICATION_ERROR03:
        ROS_ERROR("Commend: [%03d], ERROR: [%04d] Server disconnected", cmd, code);
        break;

    case COMMUNICATION_ERROR04:
        ROS_ERROR("Commend: [%03d], ERROR: [%04d] Connection timeout error", cmd, code);
        break;

    case COMMUNICATION_ERROR05:
        ROS_ERROR("Commend: [%03d], ERROR: [%04d] Other errors", cmd, code);
        break;

    case COMMUNICATION_STATE01:
        ROS_INFO("Commend: [%03d], ERROR: [%04d] Client connection successful", cmd, code);
        break;

    case COMMUNICATION_STATE02:
        ROS_INFO("Commend: [%03d], ERROR: [%04d] Server connection successful", cmd, code);
        break;

    case COMMUNICATION_STATE03:
        ROS_INFO("Commend: [%03d], ERROR: [%04d] Waiting for client connection", cmd, code);
        break;
    }
}