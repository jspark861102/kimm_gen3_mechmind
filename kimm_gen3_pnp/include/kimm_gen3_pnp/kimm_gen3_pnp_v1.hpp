#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"

#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>

namespace k_api = Kinova::Api;
using namespace Eigen;

/////////////////////////// ROS Setting ////////////////////////////////////////////////    
ros::Subscriber mech_pose_sub, vision_pose_sub, vision_pose2_sub;
ros::Publisher joint_state_pub, mech_call_pub, aruco_call_pub;

sensor_msgs::JointState joint_state;
std::vector<std::string> m_arm_joint_names;
std::vector<std::string> m_gripper_joint_names;

/////////////////////////// Kinova Setting /////////////////////////////////////////////        
int PORT;
int PORT_REAL_TIME;
string ip_address;
string username;
string password;  

float TIME_DURATION = 30.0f; // Duration of the example (seconds)
constexpr auto TIMEOUT_PROMISE_DURATION = std::chrono::seconds{20};

k_api::BaseCyclic::Feedback base_feedback;
k_api::BaseCyclic::Command  base_command;
k_api::BaseCyclic::ActuatorCommand* actuator_command;
k_api::GripperCyclic::MotorCommand* gripper_motor_command;

auto servoing_mode = k_api::Base::ServoingModeInformation();
auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();

/////////////////////////// control Setting ///////////////////////////////////////////// 
int ctrl_flag_;
bool COMMAND_SUCCEESS_;
int total_joint_size;

//variable for gripper example
bool isgripper;

//varialble for vision callback
bool ismech, isaruco, iscallback_mech, iscallback_aruco;
geometry_msgs::Pose mech_pose;

//waypint
std::vector<std::vector<float>> waypointsDefinition_wait_for_MechMind, waypointsDefinition_vision_Home, waypointsDefinition_vision_up, waypointsDefinition_vision_down, waypointsDefinition_vision_place;
auto Wait  = std::vector<std::array<float,6>>();
auto VisionHome  = std::vector<std::array<float,6>>();
auto VisionPlace = std::vector<std::array<float,6>>();

/////////////////////////// Kinova fuction ///////////////////////////////////////////// 
std::function<void(k_api::Base::ActionNotification)> 
    create_event_listener_by_promise(std::promise<k_api::Base::ActionEvent>& finish_promise)
{
    return [&finish_promise] (k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)
        {
        case k_api::Base::ActionEvent::ACTION_END:
        case k_api::Base::ActionEvent::ACTION_ABORT:
            finish_promise.set_value(action_event);
            break;
        default:
            break;
        }
    };
}

std::function<void(k_api::Base::SequenceInfoNotification)> 
    create_sequenc_event_listener_by_promise(std::promise<k_api::Base::EventIdSequenceInfoNotification>& finish_promise)
{
    return [&finish_promise] (k_api::Base::SequenceInfoNotification notification)
    {
        const auto sequence_event = notification.event_identifier();
        switch(sequence_event)
        {
        case k_api::Base::EventIdSequenceInfoNotification::SEQUENCE_COMPLETED:
        case k_api::Base::EventIdSequenceInfoNotification::SEQUENCE_ABORTED:
            finish_promise.set_value(sequence_event);
            break;
        default:
            break;
        }
    };
}

std::function<void(k_api::Base::ActionNotification)>
    create_event_listener_by_ref(k_api::Base::ActionEvent& returnAction)
{
    return [&returnAction](k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)
        {
        case k_api::Base::ActionEvent::ACTION_END:
        case k_api::Base::ActionEvent::ACTION_ABORT:
            returnAction = action_event;
            break;
        default:
            break;
        }
    };
}

void keyboard_event();
bool _kbhit()
{
    termios term;
    tcgetattr(0, &term);

    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);

    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);

    tcsetattr(0, TCSANOW, &term);

    return byteswaiting > 0;
};

double wrapRadiansFromMinusPiToPi(double rad_not_wrapped)
{
    bool properly_wrapped = false;    
    do 
    {
        if (rad_not_wrapped > M_PI)
        {            
            rad_not_wrapped -= 2.0*M_PI;
        }
        else if (rad_not_wrapped < -M_PI)
        {         
            rad_not_wrapped += 2.0*M_PI;
        }
        else
        {
            properly_wrapped = true;
        }
    } while(!properly_wrapped);
    return rad_not_wrapped;
};

/////////////////////////// Execute fuction ///////////////////////////////////////////// 
void publish_cyclic_feedback(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic);
bool set_waypoint_trajectory(k_api::Base::BaseClient* base, std::vector<std::vector<float>> waypointsDefinition, float duration);
bool set_waypoints_angular_trajectory(k_api::Base::BaseClient* base, std::vector<std::array<float,6>> jointPoses);
void populateCartesianCoordinate(k_api::Base::CartesianWaypoint* cartesianCoordinate, std::vector<float> waypointDefinition);
bool set_gripper_command(k_api::Base::BaseClient* base, float position);
void aruco_pose_callback(const geometry_msgs::Pose &msg);
void mechmind_pose_callback(const geometry_msgs::Pose &msg);
void pose_calculation(const geometry_msgs::Pose &msg);    
// void mechmind_pose_callback(const geometry_msgs::Pose &msg, k_api::Base::BaseClient* base);
