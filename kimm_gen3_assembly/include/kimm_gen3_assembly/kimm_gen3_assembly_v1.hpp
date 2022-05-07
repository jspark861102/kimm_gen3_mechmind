#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/DynamixelState.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>

#include <Eigen/Core>

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

#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>

namespace k_api = Kinova::Api;
using namespace Eigen;

/////////////////////////// ROS Setting ////////////////////////////////////////////////    
ros::Publisher test_string_pub_;
ros::Publisher joint_state_pub_;
ros::Publisher ee_state_pub;
ros::Publisher ee_twist_pub;
ros::Publisher external_wrench_pub;

ros::Subscriber vision_pose_sub;
ros::Subscriber dynamixel_state_sub;

ros::ServiceClient dynamixel_client;

int pub_count;
sensor_msgs::JointState joint_state;
geometry_msgs::Twist ee_state;
geometry_msgs::Twist ee_twist;
geometry_msgs::Wrench external_wrench;
std::mutex calculation_mutex_;
std::thread publish_feedback_thread;

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

/////////////////////////// dynamixel setting ///////////////////////////////////////////// 
dynamixel_workbench_msgs::DynamixelState  dynamixel_state;
dynamixel_workbench_msgs::DynamixelStateList dynamixel_state_list_;
dynamixel_workbench_msgs::DynamixelCommand drill;

/////////////////////////// control Setting ///////////////////////////////////////////// 
int ctrl_flag_;
bool COMMAND_SUCCEESS_;

//variable for example_cyclic_torque_control function
bool return_status;
unsigned int actuator_count;
int first_actuator_device_id;
float init_delta_position;
float init_last_torque;
float init_first_torque;
float torque_amplification;

//variable for gripper example
bool isgripper;

//waypint
std::vector<std::vector<float>> waypointsDefinition1, waypointsDefinition2, waypointsDefinition3, waypointsDefinition4;
std::vector<std::vector<float>> waypointsDefinition5, waypointsDefinition6, waypointsDefinition7, waypointsDefinition8;
std::vector<std::vector<float>> waypointsDefinition_Latch_Up, waypointsDefinition_Latch_Down, waypointsDefinition_Bolt_Fit;
std::vector<std::vector<float>> waypointsDefinition_Assemble_Initial, waypointsDefinition_Assemble_Approach, waypointsDefinition_Assemble_Tighten;
std::vector<std::vector<float>> waypointsDefinition_vision;
std::vector<std::vector<float>> waypointsDefinition_Home;

auto jointPoses  = std::vector<std::array<float,6>>();

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
void Thread_publish_cyclinc_feedback(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic);
bool move_to_named_position(k_api::Base::BaseClient* base, string &name); 
bool move_to_named_sequence(k_api::Base::BaseClient* base, string &name); 
bool publish_cyclic_feedback(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic);
bool example_cyclic_torque_control(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic, k_api::ActuatorConfig::ActuatorConfigClient* actuator_config);
bool example_cyclic_torque_control_close(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic, k_api::ActuatorConfig::ActuatorConfigClient* actuator_config);
bool set_waypoint_trajectory(k_api::Base::BaseClient* base, std::vector<std::vector<float>> waypointsDefinition, float duration);
void populateCartesianCoordinate(k_api::Base::CartesianWaypoint* cartesianCoordinate, std::vector<float> waypointDefinition);
bool set_gripper_command(k_api::Base::BaseClient* base, float position);
void vision_pose_callback(const geometry_msgs::Pose &msg);

void dynamixel_callback(const dynamixel_workbench_msgs::DynamixelStateList &msg);
void set_drill_command(int current);
bool set_waypoints_angular_trajectory(k_api::Base::BaseClient* base, std::vector<std::array<float,6>> jointPoses);


