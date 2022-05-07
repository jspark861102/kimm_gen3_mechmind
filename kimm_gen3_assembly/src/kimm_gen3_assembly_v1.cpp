#include "kimm_gen3_assembly/kimm_gen3_assembly_v1.hpp"

using namespace std;
using namespace k_api;

#define SAMPLING_RATE 1000

int main(int argc, char **argv)
{
    /////////////////////////// ROS Setting ////////////////////////////////////////////////    
    ros::init(argc, argv, "kinova_controller");
    ros::NodeHandle n_node;
    ros::Rate loop_rate(SAMPLING_RATE);
    joint_state_pub_ = n_node.advertise<sensor_msgs::JointState>("my_kinova/joint_state", 5);  
    ee_state_pub = n_node.advertise<geometry_msgs::Twist>("my_kinova/ee_state",5); 
    ee_twist_pub = n_node.advertise<geometry_msgs::Twist>("my_kinova/ee_twist",5);  
    external_wrench_pub = n_node.advertise<geometry_msgs::Wrench>("my_kinova/external_wrench",5);  

    vision_pose_sub = n_node.subscribe("my_kinova/vision_pose", 100, vision_pose_callback);
    dynamixel_state_sub = n_node.subscribe("/dynamixel_workbench/dynamixel_state", 100, dynamixel_callback);

    dynamixel_client = n_node.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");             
    ////////////////////////////////////////////////////////////////////////////////////////
    

    /////////////////////////// Kinova Setting /////////////////////////////////////////////        
    n_node.getParam("/kimm_gen3_assembly_v1/port_number", PORT);
    n_node.getParam("/kimm_gen3_assembly_v1/port_realtime_number", PORT_REAL_TIME);
    n_node.getParam("/kimm_gen3_assembly_v1/ip_address", ip_address);
    n_node.getParam("/kimm_gen3_assembly_v1/username", username);
    n_node.getParam("/kimm_gen3_assembly_v1/password", password);    
    
    // Create API objects
    auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };

    std::cout << "Creating transport objects" << std::endl;
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(ip_address, PORT);

    std::cout << "Creating transport real time objects" << std::endl;
    auto transport_real_time = new k_api::TransportClientUdp();
    auto router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(ip_address, PORT_REAL_TIME);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username(username);
    create_session_info.set_password(password);
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating session for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    auto session_manager_real_time = new k_api::SessionManager(router_real_time);
    session_manager_real_time->CreateSession(create_session_info);
    std::cout << "Session created" << std::endl;

    // Create services
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);
    auto actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(router);
    ////////////////////////////////////////////////////////////////////////////////////////

    /////////////////////////// Dyanmixel Setting //////////////////////////////////////////
    // cout << dynamixel_state_list_ << endl;

    ////////////////////////////////////////////////////////////////////////////////////////

    /////////////////////////// Control Setting /////////////////////////////////////////////        
    //Control setting
    ctrl_flag_ = 0;
    COMMAND_SUCCEESS_ = true;     
    
    //realtime control setting
    return_status = true;
    actuator_count = base->GetActuatorCount().count();
    first_actuator_device_id = 1;

    //gripper control setting
    isgripper = false;    

    //variable for feedback publish
    int pub_count;      
    ////////////////////////////////////////////////////////////////////////////////////////

    // std::vector<std::vector<float>> waypointsDefinition1, waypointsDefinition2, waypointsDefinition3, waypointsDefinition4, waypointsDefinition_vision;
    // first
    waypointsDefinition1 = { {0.444,   0.214,  0.13,   0.0f, -180.0,   0.0, 90.0}, //Pose_First_Bolt_Up                            
                        };

    waypointsDefinition2 = { {0.444,   0.214,  0.084,  0.0f, -180.0,   0.0, 90.0}, //Pose_First_Bolt_Down                            
                        };

    waypointsDefinition3 = { {0.444,   0.214,  0.13,   0.0f, -180.0,   0.0, 90.0}, //Pose_First_Bolt_Up
                             {0.443,  -0.018,  0.13,   0.0f, -180.0, -16.5, 90.0}, //Pose_First_Latch_Up_Inclined
                             {0.443,  -0.018,  0.085,  0.0f, -180.0, -16.5, 90.0}, //Pose_First_Latch_Down_Inclined                            
                        };

    waypointsDefinition4 = { {0.443,  -0.018,  0.13,   0.0f, -180.0, -16.5, 90.0}, //Pose_First_Latch_Up_Inclined
                        };

    // second
    waypointsDefinition5 = { {0.444,   0.313,  0.130,  0.0f, -180.0,   0.0, 90.0}, //Pose_Second_Bolt_Up 
                        };

    waypointsDefinition6 = { {0.444,   0.313,  0.084,  0.0f, -180.0,   0.0, 90.0}, //Pose_Second_Bolt_Down 
                        };

    waypointsDefinition7 = { {0.444,   0.313,  0.130,  0.0f, -180.0,   0.0, 90.0}, //Pose_Second_Bolt_Up
                             {0.443,   0.046,  0.130,  0.0f, -180.0,  16.5, 90.0}, //Pose_Second_Latch_Up_Inclined
                             {0.443,   0.046,  0.085,  0.0f, -180.0,  16.5, 90.0}, //Pose_Second_Latch_Down_Inclined
                        };

    waypointsDefinition8 = { {0.443,   0.046,  0.130,  0.0f, -180.0,  16.5, 90.0}, //Pose_Second_Latch_Up_Inclined
                             
                        };

    // Latch up
    waypointsDefinition_Latch_Up = { {0.442,   0.016,  0.130,  0.0f, 0.0,  180.0, 0.0}, //Pose_Latch_Up                             
                        };
    
    waypointsDefinition_Latch_Down = { {0.442,   0.016,  0.085,  0.0f, 0.0,  180.0, 0.0}, //Pose_Latch_Down                             
                        };

    waypointsDefinition_Bolt_Fit = { {0.443,   0.016,  0.080,  0.0f, 0.0,  180.0, 0.0}, //Pose_Bolt_Fit
                        };

    // Latch Assembly
    waypointsDefinition_Assemble_Initial = { {0.444,  -0.06,  0.332,  0.0f, 90.0,  0.0, 0.0}, //Latch Assemble Initial
                        };
    
    waypointsDefinition_Assemble_Approach = { {0.444, -0.150,  0.332,  0.0f, 90.0,  0.0, 0.0}, //Latch Assemble Approach
                        };

    waypointsDefinition_Assemble_Tighten = { {0.444, -0.160,  0.332,  0.0f, 90.0,  0.0, 0.0}, //Latch Assemble tightening
                        };

    // Home
    waypointsDefinition_Home = { {0.575,  -0.005,  0.419,  0.0f, 90.3,  0.0, 89.1}, //Home
                        };

    jointPoses.push_back({ 0.02, 25.81, 264.82, 179.99, 58.08, 270.67});

    
    // waypointsDefinition_vision is obtained from callback
    ////////////////////////////////////////////////////////////////////////////////////////

    int count = 0;
    while (ros::ok()){
        keyboard_event();    

        // feedback from real robot
        base_feedback = base_cyclic->RefreshFeedback();
        
        // publish robot data
        publish_cyclic_feedback(base, base_cyclic); //publish frequency = 50Hz        
        // Thread_publish_cyclinc_feedback(base, base_cyclic);             

        // start assembly
        if (ctrl_flag_ == 1 && !COMMAND_SUCCEESS_){ //move to home position, h
            COMMAND_SUCCEESS_ = true;            
            string name("MyHome");
            move_to_named_position(base, name);
            // set_waypoints_angular_trajectory(base, jointPoses); //MyHome in angluar trajecotry

        }
        if (ctrl_flag_ == 2 && !COMMAND_SUCCEESS_){ //move with a sequence, s
            COMMAND_SUCCEESS_ = true;                        
            string name("Sequence_First_Bolt_Pick_Place_Inclined");
            move_to_named_sequence(base, name);
            name = "Sequence_Second_Bolt_Pick_Place_Inclined";
            move_to_named_sequence(base, name);
            name = "Sequence_Latch_PickUp";
            move_to_named_sequence(base, name);
        }        
        if (ctrl_flag_ == 3){ //admittance ,q
            count = count + 1;
            // ROS_INFO("%d", count);
            if (count < 5000)
            {           
                example_cyclic_torque_control(base, base_cyclic, actuator_config);
            }
            else if (count < 5001)
            {
                example_cyclic_torque_control_close(base, base_cyclic, actuator_config);    
            }
        }          
        if (ctrl_flag_ == 4 && !COMMAND_SUCCEESS_){ //waypoint, w
            COMMAND_SUCCEESS_ = true;                        
            //first
            set_waypoint_trajectory(base, waypointsDefinition1, 0.0); //Pose_First_Bolt_Up       
            set_gripper_command(base, 0.8); //Gripper_Bolt_Realse                               
            set_waypoint_trajectory(base, waypointsDefinition2, 0.0); //Pose_First_Bolt_Down                             
            set_gripper_command(base, 0.98); //Gripper_Bolt_Close         
            set_waypoint_trajectory(base, waypointsDefinition3, 0.0); //Pose_First_Bolt_Latch 
            set_gripper_command(base, 0.94); //Gripper_Bolt_Realse_InLatch                                    
            set_waypoint_trajectory(base, waypointsDefinition4, 0.0); //Pose_First_Bolt_Place                             

            //second
            set_waypoint_trajectory(base, waypointsDefinition5, 0.0); //Pose_Second_Bolt_Up       
            set_gripper_command(base, 0.8); //Gripper_Bolt_Realse                               
            set_waypoint_trajectory(base, waypointsDefinition6, 0.0); //Pose_Second_Bolt_Down                             
            set_gripper_command(base, 0.98); //Gripper_Bolt_Close         
            set_waypoint_trajectory(base, waypointsDefinition7, 0.0); //Pose_Second_Bolt_Latch 
            set_gripper_command(base, 0.94); //Gripper_Bolt_Realse_InLatch                                    
            set_waypoint_trajectory(base, waypointsDefinition8, 0.0); //Pose_Second_Bolt_Place                             

            //Latch up
            set_waypoint_trajectory(base, waypointsDefinition_Latch_Up, 0.0); //Pose_Latch_Up
            set_gripper_command(base, 0.5); //Gripper_Bolt_Realse                               
            set_drill_command(50);
            set_waypoint_trajectory(base, waypointsDefinition_Latch_Down, 3.0); //Pose_Latch_Down
            set_waypoint_trajectory(base, waypointsDefinition_Bolt_Fit, 5.0); //Pose_Bolt_Fit
            set_gripper_command(base, 0.78); //Gripper_Bolt_Close         
            set_drill_command(0);
            set_waypoint_trajectory(base, waypointsDefinition_Latch_Up, 0.0); //Pose_Latch_Up
            
            //Assemble
            set_waypoint_trajectory(base, waypointsDefinition_Home, 0.0); //Home
            set_waypoint_trajectory(base, waypointsDefinition_Assemble_Initial, 0.0); //Latch Assemble Initial
            set_waypoint_trajectory(base, waypointsDefinition_Assemble_Approach, 3.0); //Latch Assemble Approach
            set_drill_command(50);
            set_waypoint_trajectory(base, waypointsDefinition_Assemble_Tighten, 10.0); //Latch Assemble Titghten
            set_drill_command(0);
            set_gripper_command(base, 0.5); //Gripper_release            
            set_waypoint_trajectory(base, waypointsDefinition_Assemble_Initial, 0.0); //Latch Assemble Initial
            
            set_waypoints_angular_trajectory(base, jointPoses); //MyHome in angluar trajecotry

        }           
        if (ctrl_flag_ == 5 && !COMMAND_SUCCEESS_){ //waypoint, e
            COMMAND_SUCCEESS_ = true;                        
            // //first
            // set_waypoint_trajectory(base, waypointsDefinition1, 0.0); //Pose_First_Bolt_Up       
            // set_gripper_command(base, 0.8); //Gripper_Bolt_Realse                               
            // set_waypoint_trajectory(base, waypointsDefinition2, 0.0); //Pose_First_Bolt_Down                             
            // set_gripper_command(base, 0.95); //Gripper_Bolt_Close         
            // set_waypoint_trajectory(base, waypointsDefinition3, 0.0); //Pose_First_Bolt_Latch 
            // set_gripper_command(base, 0.9); //Gripper_Bolt_Realse_InLatch                                    
            // set_waypoint_trajectory(base, waypointsDefinition4, 0.0); //Pose_First_Bolt_Place                             

            // //second
            // set_waypoint_trajectory(base, waypointsDefinition5, 0.0); //Pose_Second_Bolt_Up       
            // set_gripper_command(base, 0.8); //Gripper_Bolt_Realse                               
            // set_waypoint_trajectory(base, waypointsDefinition6, 0.0); //Pose_Second_Bolt_Down                             
            // set_gripper_command(base, 0.95); //Gripper_Bolt_Close         
            // set_waypoint_trajectory(base, waypointsDefinition7, 0.0); //Pose_Second_Bolt_Latch 
            // set_gripper_command(base, 0.90); //Gripper_Bolt_Realse_InLatch                                    
            // set_waypoint_trajectory(base, waypointsDefinition8, 0.0); //Pose_Second_Bolt_Place                             

            // //Latch up
            // set_waypoint_trajectory(base, waypointsDefinition_Latch_Up, 0.0); //Pose_Latch_Up
            // set_gripper_command(base, 0.5); //Gripper_Bolt_Realse                               
            // set_drill_command(50);
            // set_waypoint_trajectory(base, waypointsDefinition_Latch_Down, 3.0); //Pose_Latch_Down
            // set_waypoint_trajectory(base, waypointsDefinition_Bolt_Fit, 5.0); //Pose_Bolt_Fit
            // set_gripper_command(base, 0.76); //Gripper_Bolt_Close         
            // set_drill_command(0);
            // set_waypoint_trajectory(base, waypointsDefinition_Latch_Up, 0.0); //Pose_Latch_Up
            
            //Assemble
            set_waypoint_trajectory(base, waypointsDefinition_Home, 0.0); //Home
            set_waypoint_trajectory(base, waypointsDefinition_Assemble_Initial, 0.0); //Latch Assemble Initial
            set_waypoint_trajectory(base, waypointsDefinition_Assemble_Approach, 3.0); //Latch Assemble Approach
            set_drill_command(50);
            set_waypoint_trajectory(base, waypointsDefinition_Assemble_Tighten, 10.0); //Latch Assemble Titghten
            set_drill_command(0);
            set_gripper_command(base, 0.5); //Gripper_release            
            set_waypoint_trajectory(base, waypointsDefinition_Assemble_Initial, 0.0); //Latch Assemble Initial
            set_waypoint_trajectory(base, waypointsDefinition_Home, 0.0); //Home

        }       
        if (ctrl_flag_ == 99 && !COMMAND_SUCCEESS_) { //gripper        
            COMMAND_SUCCEESS_ = true;                        
            if (!isgripper){
                isgripper = true;  
                set_gripper_command(base, 0.0);                         
            }                
            else{
                isgripper = false;  
                set_gripper_command(base, 0.8);                                  
            }                
        }
        if (ctrl_flag_ == 991 && !COMMAND_SUCCEESS_) { //bolt tightening
            COMMAND_SUCCEESS_ = true; 
            set_drill_command(500);
        }
        if (ctrl_flag_ == 992 && !COMMAND_SUCCEESS_) { //bolt releasing
            COMMAND_SUCCEESS_ = true; 
            set_drill_command(-500);
        }
        if (ctrl_flag_ == 993 && !COMMAND_SUCCEESS_) { //bolt stop        
            COMMAND_SUCCEESS_ = true;                        
            set_drill_command(0);
        }

        if (ctrl_flag_ == 999 && !COMMAND_SUCCEESS_){ // Close API session
            if (publish_feedback_thread.joinable())
            {
                publish_feedback_thread.join();
            }
 
            session_manager->CloseSession();

            router->SetActivationStatus(false);
            transport->disconnect();
            router_real_time->SetActivationStatus(false);
            transport_real_time->disconnect();

            delete base;
            delete base_cyclic;
            delete actuator_config;
            delete session_manager;
            delete session_manager_real_time;
            delete router;
            delete router_real_time;
            delete transport;
            delete transport_real_time;
            break;
        } // Quit

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void keyboard_event(){
    if (_kbhit()){
        int key;
        key = getchar();
        int msg = 0;
        switch (key){
            case 'h': //home
                ctrl_flag_ = 1;
                COMMAND_SUCCEESS_ = false;
                cout << " " << endl;
                cout << "Move to HOME POSITION" << endl;
                cout << " " << endl;
                break;                    
            case 's': //sequence
                ctrl_flag_ = 2;
                COMMAND_SUCCEESS_ = false;
                cout << " " << endl;
                cout << "Move with a sequence" << endl;
                cout << " " << endl;
                break;      
            case 'q': //sequence
                ctrl_flag_ = 3;
                COMMAND_SUCCEESS_ = false;
                cout << " " << endl;
                cout << "low level control" << endl;
                cout << " " << endl;
                break;      
            case 'w': //waypoints
                ctrl_flag_ = 4;
                COMMAND_SUCCEESS_ = false;
                cout << " " << endl;
                cout << "waypoints" << endl;
                cout << " " << endl;
                break;      
            case 'e': //waypoints
                ctrl_flag_ = 5;
                COMMAND_SUCCEESS_ = false;
                cout << " " << endl;
                cout << "waypoints" << endl;
                cout << " " << endl;
                break;      
            case 'z': //gripper
                ctrl_flag_ = 99;                
                COMMAND_SUCCEESS_ = false;
                cout << " " << endl;
                cout << "gripper" << endl;
                cout << " " << endl;
                break;                
            case 'x': //drill tightening
                ctrl_flag_ = 991;                                
                COMMAND_SUCCEESS_ = false;
                cout << " " << endl;
                cout << "drill tightening" << endl;
                cout << " " << endl;
                break;                
            case 'c': //drill releasing
                ctrl_flag_ = 992;                
                COMMAND_SUCCEESS_ = false;
                cout << " " << endl;
                cout << "drill releasing" << endl;
                cout << " " << endl;
                break;                
            case 'v': //drill stop                
                ctrl_flag_ = 993; 
                COMMAND_SUCCEESS_ = false;                               
                cout << " " << endl;
                cout << "drill stop" << endl;
                cout << " " << endl;
                break;                
            case 'p': //quit
                ctrl_flag_ = 999;
                COMMAND_SUCCEESS_ = false;
                cout << " " << endl;
                cout << "Close API Session" << endl;
                cout << " " << endl;
                break;  
        }
    }
}

void Thread_publish_cyclinc_feedback(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
{
    if (calculation_mutex_.try_lock())
    {
        calculation_mutex_.unlock();
        if (publish_feedback_thread.joinable())
        publish_feedback_thread.join();

        publish_feedback_thread = std::thread(publish_cyclic_feedback, base, base_cyclic);
    }
    ros::Rate r(30000);
    for (int i = 0; i < 9; i++)
    {
        r.sleep();
        if (calculation_mutex_.try_lock())
        {
        calculation_mutex_.unlock();
        if (publish_feedback_thread.joinable())
            publish_feedback_thread.join();
        break;
        }
    }  
}

bool publish_cyclic_feedback(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
{
    if (pub_count == 19) // publish frequency = 50Hz
    {    
        // base_feedback = base_cyclic->RefreshFeedback();
        int total_size =  base_feedback.actuators_size() + base_feedback.interconnect().gripper_feedback().motor_size();

        ///////////////////////// joint_state ///////////////////////////
        joint_state.name.resize(total_size);
        joint_state.position.resize(total_size);
        joint_state.velocity.resize(total_size);
        joint_state.effort.resize(total_size);
        joint_state.header.stamp = ros::Time::now();

        joint_state.name = {"shoulder", "bicep", "forearm", "wrist1", "wrist2","wrist3"};
        for (int i = 0; i < base_feedback.actuators_size(); i++) //arm
        {
            joint_state.position[i] = wrapRadiansFromMinusPiToPi(base_feedback.actuators(i).position() * M_PI / 180.0); // [rad]
            joint_state.velocity[i] = base_feedback.actuators(i).velocity() * M_PI / 180.0; // [rad/sec]
            joint_state.effort[i] = base_feedback.actuators(i).torque(); //[Nm]
        }

        if (base_feedback.has_interconnect()) //gripper
        {
            joint_state.name.push_back("gripper");
            joint_state.position[total_size-1] = base_feedback.interconnect().gripper_feedback().motor(0).position(); // 0~100 [%] 
            joint_state.velocity[total_size-1] = base_feedback.interconnect().gripper_feedback().motor(0).velocity(); // 0~100 [%]
            joint_state.effort[total_size-1] = base_feedback.interconnect().gripper_feedback().motor(0).current_motor(); // [mA]
        }

        ///////////////////////// ee_state ///////////////////////////
        ee_state.linear.x = base_feedback.base().tool_pose_x();
        ee_state.linear.y = base_feedback.base().tool_pose_y();
        ee_state.linear.z = base_feedback.base().tool_pose_z();
        ee_state.angular.x = base_feedback.base().tool_pose_theta_x();
        ee_state.angular.y = base_feedback.base().tool_pose_theta_y();
        ee_state.angular.z = base_feedback.base().tool_pose_theta_z();

        ee_twist.linear.x = base_feedback.base().tool_twist_linear_x();
        ee_twist.linear.y = base_feedback.base().tool_twist_linear_y();
        ee_twist.linear.z = base_feedback.base().tool_twist_linear_z();
        ee_twist.angular.x = base_feedback.base().tool_twist_angular_x();
        ee_twist.angular.y = base_feedback.base().tool_twist_angular_y();
        ee_twist.angular.z = base_feedback.base().tool_twist_angular_z();

        external_wrench.force.x = base_feedback.base().tool_external_wrench_force_x();
        external_wrench.force.y = base_feedback.base().tool_external_wrench_force_y();
        external_wrench.force.z = base_feedback.base().tool_external_wrench_force_z();
        external_wrench.torque.x = base_feedback.base().tool_external_wrench_torque_x();
        external_wrench.torque.y = base_feedback.base().tool_external_wrench_torque_y();
        external_wrench.torque.z = base_feedback.base().tool_external_wrench_torque_z();

        ///////////////////////// feedback publish ///////////////////////////
        joint_state_pub_.publish(joint_state);
        ee_state_pub.publish(ee_state);
        ee_twist_pub.publish(ee_twist);
        external_wrench_pub.publish(external_wrench);        

        pub_count = 0;
    }
    else
    {
        pub_count += 1; //need to check topic frequency & topic infomation
    }
    return true;
}

bool move_to_named_position(k_api::Base::BaseClient* base, string &name) 
{
    // Make sure the arm is in Single Level Servoing before executing an Action    
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));            

    // Move arm to ready position
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::UNSPECIFIED_ACTION);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = k_api::Base::ActionHandle();
    action_handle.set_identifier(0);
    for (auto action : action_list.action_list()) 
    {
        // cout << action.name() << endl;
        if (action.name() == name) 
        {
            action_handle = action.handle();      
            std::cout << "Moving the arm with position of " << name << std::endl;    
              
        }
    }

    if (action_handle.identifier() == 0) 
    {
        std::cout << "Can't reach safe position, exiting" << std::endl;
        return false;
    } 
    else 
    {
        // Connect to notification action topic
        std::promise<k_api::Base::ActionEvent> finish_promise;
        auto finish_future = finish_promise.get_future();
        auto promise_notification_handle = base->OnNotificationActionTopic(
            create_event_listener_by_promise(finish_promise),
            k_api::Common::NotificationOptions()
        );

        // Execute action
        base->ExecuteActionFromReference(action_handle);

        // Wait for future value from promise
        const auto status = finish_future.wait_for(TIMEOUT_PROMISE_DURATION);
        base->Unsubscribe(promise_notification_handle);

        if(status != std::future_status::ready)
        {
            std::cout << "Timeout on action notification wait" << std::endl;
        }
        const auto promise_event = finish_future.get();

        std::cout << "Move to Home completed" << std::endl;
        std::cout << "Promise value : " << k_api::Base::ActionEvent_Name(promise_event) << std::endl; 
    }
    return true;
}

bool move_to_named_sequence(k_api::Base::BaseClient* base, string &name) 
{
    // Make sure the arm is in Single Level Servoing before executing an Action    
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));            

    // Move arm to ready position    
    auto sequence_list = base->ReadAllSequences();
    auto sequence_handle = k_api::Base::SequenceHandle();
    sequence_handle.set_identifier(0);
    for (auto sequence : sequence_list.sequence_list()) 
    {
        // cout << sequence.name() << endl;
        if (sequence.name() == name) 
        {
            sequence_handle = sequence.handle();                    
            std::cout << "Moving the arm with sequence of " << name << std::endl;    
        }
    }

    if (sequence_handle.identifier() == 0) 
    {
        std::cout << "Can't identify sequence, exiting" << std::endl;
        return false;
    } 
    else 
    {
        // Connect to notification action topic
        std::promise<k_api::Base::EventIdSequenceInfoNotification> finish_promise;
        auto finish_future = finish_promise.get_future();
        auto promise_notification_handle = base->OnNotificationSequenceInfoTopic(
            create_sequenc_event_listener_by_promise(finish_promise),
            k_api::Common::NotificationOptions()
        );

        // Execute action
        base->PlaySequence(sequence_handle);

        // Wait for future value from promise
        const auto status = finish_future.wait_for(TIMEOUT_PROMISE_DURATION);
        base->Unsubscribe(promise_notification_handle);

        if(status != std::future_status::ready)
        {
            std::cout << "Timeout on sequence notification wait" << std::endl;
        }
        const auto promise_event = finish_future.get();

        std::cout << "Move with a sequence completed" << std::endl;
        std::cout << "Promise value : " << k_api::Base::EventIdSequenceInfoNotification_Name(promise_event) << std::endl; 
    }

    // auto my_sequence = k_api::Base::Sequence();
    // // auto my_handle = k_api::Base::SequenceHandle();    
    // base->CreateSequence(my_sequence);

    return true;
}

bool example_cyclic_torque_control(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic, k_api::ActuatorConfig::ActuatorConfigClient* actuator_config)
{
    if(!COMMAND_SUCCEESS_) {            
        return_status = true;
        
        // Clearing faults
        try
        {
            base->ClearFaults();
        }
        catch(...)
        {
            std::cout << "Unable to clear robot faults" << std::endl;
            return false;
        }

        std::cout << "Initializing the arm for torque control example" << std::endl;

        // Set the base in low-level servoing mode
        servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoing_mode);
        base_feedback = base_cyclic->RefreshFeedback();

        // Initialize each actuator to their current position
        for (unsigned int i = 0; i < actuator_count; i++)
        {
            // Save the current actuator position, to avoid a following error
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
        }

        // Send a first frame
        base_feedback = base_cyclic->Refresh(base_command);
        
        // Set first actuator in torque mode now that the command is equal to measure
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);

        actuator_config->SetControlMode(control_mode_message, first_actuator_device_id);
        // actuator_config->SetControlMode(control_mode_message, 3);

        // Initial delta between first and last actuator
        init_delta_position = base_feedback.actuators(0).position() - base_feedback.actuators(actuator_count - 1).position();

        // Initial first and last actuator torques; avoids unexpected movement due to torque offsets
        init_last_torque = base_feedback.actuators(actuator_count - 1).torque();
        init_first_torque = -base_feedback.actuators(0).torque(); //Torque measure is reversed compared to actuator direction
        torque_amplification = 2.0;

        std::cout << "Running torque control example for " << TIME_DURATION << " seconds" << std::endl;

        COMMAND_SUCCEESS_ = true;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////// realtime ////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////
    try
    {               
        // Real-time loop
        
        // Position command to first actuator is set to measured one to avoid following error to trigger
        // Bonus: When doing this instead of disabling the following error, if communication is lost and first
        //        actuator continues to move under torque command, resulting position error with command will
        //        trigger a following error and switch back the actuator in position command to hold its position
        base_command.mutable_actuators(0)->set_position(base_feedback.actuators(0).position());                        

        // First actuator torque command is set to last actuator torque measure times an amplification
        base_command.mutable_actuators(0)->set_torque_joint(init_first_torque + (torque_amplification * (base_feedback.actuators(actuator_count - 1).torque() - init_last_torque)));

        // First actuator position is sent as a command to last actuator
        base_command.mutable_actuators(actuator_count - 1)->set_position(base_feedback.actuators(0).position() - init_delta_position);


        // base_command.mutable_actuators(2)->set_position(base_feedback.actuators(2).position());        
        // base_command.mutable_actuators(2)->set_torque_joint(11);

        // Incrementing identifier ensures actuators can reject out of time frames
        base_command.set_frame_id(base_command.frame_id() + 1);
        if (base_command.frame_id() > 65535)
            base_command.set_frame_id(0);

        for (int idx = 0; idx < actuator_count; idx++)
        {
            base_command.mutable_actuators(idx)->set_command_id(base_command.frame_id());
        }

        try
        {
            base_feedback = base_cyclic->Refresh(base_command, 0);
        }
        catch (k_api::KDetailedException& ex)
        {
            std::cout << "Kortex exception: " << ex.what() << std::endl;

            std::cout << "Error sub-code: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
        }
        catch (std::runtime_error& ex2)
        {
            std::cout << "runtime error: " << ex2.what() << std::endl;
        }
        catch(...)
        {
            std::cout << "Unknown error." << std::endl;
        }            
    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "API error: " << ex.what() << std::endl;
        return_status = false;
    }
    catch (std::runtime_error& ex2)
    {
        std::cout << "Error: " << ex2.what() << std::endl;
        return_status = false;
    }
    return return_status;
}

bool example_cyclic_torque_control_close(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic, k_api::ActuatorConfig::ActuatorConfigClient* actuator_config)
{
    std::cout << "Torque control example completed" << std::endl;

    // Set first actuator back in position 
    control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
    actuator_config->SetControlMode(control_mode_message, first_actuator_device_id);

    std::cout << "Torque control example clean exit" << std::endl;

    // Set the servoing mode back to Single Level
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);

    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    return true;
}

bool set_waypoint_trajectory(k_api::Base::BaseClient* base, std::vector<std::vector<float>> waypointsDefinition, float duration)
{
    bool success = false;
    // std::vector<std::vector<float>> waypointsDefinition;
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Note : To customize this example for your needs an array of array is used containing the information needed :
    // 1 - x position
    // 2 - y position
    // 3 - z position
    // 4 - blending radius
    // 5 - Theta x
    // 6 - Theta y
    // 7 - Theta z
    // You may overwrite the waypointsDefinition vector by commenting the code bellow and populating it with your own
    // waypoint list information.
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////        
    // waypointsDefinition = { {0.446,   0.214,  0.13,   0.0f, -180.0,   0.0, 90.0}, //Pose_First_Bolt_Up
    //                         //gripper bolt release
    //                         {0.446,   0.214,  0.071,  0.0f, -180.0,   0.0, 90.0}, //Pose_First_Bolt_Down
    //                         //gripper bolt close
    //                         {0.446,   0.214,  0.13,   0.0f, -180.0,   0.0, 90.0}, //Pose_First_Bolt_Up
    //                         {0.443,  -0.016,  0.13,   0.0f, -180.0, -16.5, 90.0}, //Pose_First_Latch_Up_Inclined
    //                         {0.443,  -0.016,  0.085,  0.0f, -180.0, -16.5, 90.0}, //Pose_First_Latch_Down_Inclined
    //                         //gripper bolt release inlatch
    //                         {0.443,  -0.016,  0.13,   0.0f, -180.0, -16.5, 90.0}, //Pose_First_Latch_Up_Inclined
    //                     };

    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Create the trajectory 
    k_api::Base::WaypointList wpts = k_api::Base::WaypointList();
    wpts.set_duration(duration); // as fast as possible
    wpts.set_use_optimal_blending(false);

    // Start waypoint list creation
    int index  = 0;
    for(std::vector<std::vector<float>>::iterator it = waypointsDefinition.begin(); 
        it != waypointsDefinition.end(); ++it, ++index) 
    {
        k_api::Base::Waypoint *wpt = wpts.add_waypoints();
        if(wpt != nullptr)
        {
            wpt->set_name(std::string("waypoint_") + std::to_string(index));
            Kinova::Api::Base::CartesianWaypoint* coordinate = wpt->mutable_cartesian_waypoint();
            if(coordinate != nullptr)
            {
                populateCartesianCoordinate(coordinate, *it);                
            }
        }
    }

    // Connect to notification action topic
    std::promise<k_api::Base::ActionEvent> finish_promise_cart;
    auto finish_future_cart = finish_promise_cart.get_future();
    auto promise_notification_handle_cart_end = base->OnNotificationActionTopic( create_event_listener_by_promise(finish_promise_cart),
                                                                            k_api::Common::NotificationOptions());

    // Verify validity of waypoints
    auto result = base->ValidateWaypointList(wpts);

    if(result.trajectory_error_report().trajectory_error_elements_size() == 0)
    {   
        // Execute action
        try
        {
            // Move arm with waypoints list
            std::cout << "Moving the arm creating a trajectory of " << waypointsDefinition.size() << " cartesian waypoints" << std::endl;
            base->ExecuteWaypointTrajectory(wpts);
        }
        catch(k_api::KDetailedException& ex)
        {
            std::cout << "Try catch error executing normal trajectory" << std::endl;
            // You can print the error informations and error codes
            auto error_info = ex.getErrorInfo().getError();
            std::cout << "KDetailedoption detected what:  " << ex.what() << std::endl;
            
            std::cout << "KError error_code: " << error_info.error_code() << std::endl;
            std::cout << "KError sub_code: " << error_info.error_sub_code() << std::endl;
            std::cout << "KError sub_string: " << error_info.error_sub_string() << std::endl;

            // Error codes by themselves are not very verbose if you don't see their corresponding enum value
            // You can use google::protobuf helpers to get the string enum element for every error code and sub-code 
            std::cout << "Error code string equivalent: " << k_api::ErrorCodes_Name(k_api::ErrorCodes(error_info.error_code())) << std::endl;
            std::cout << "Error sub-code string equivalent: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(error_info.error_sub_code())) << std::endl;
            return false;
        }
        // Wait for future value from promise
        const auto cart_end_status = finish_future_cart.wait_for(TIMEOUT_PROMISE_DURATION);

        base->Unsubscribe(promise_notification_handle_cart_end);

        if(cart_end_status != std::future_status::ready)
        {
            std::cout << "Timeout on action notification wait for cartesian waypoint trajectory" << std::endl;
        }

        const auto cart_promise_event = finish_future_cart.get();
        std::cout << "cartesian waypoint trajectory completed" << std::endl;
        std::cout << "Promise value : " << k_api::Base::ActionEvent_Name(cart_promise_event) << std::endl; 

    }
    else
    {
        std::cout << "Error found in trajectory" << std::endl; 
        result.trajectory_error_report().PrintDebugString();
    }

    return success;
}

// Helper function to populate Cartesian waypoint
void populateCartesianCoordinate(k_api::Base::CartesianWaypoint* cartesianCoordinate, std::vector<float> waypointDefinition)
{    
    static const k_api::Common::CartesianReferenceFrame kReferenceFrame = k_api::Common::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_BASE;
 

    cartesianCoordinate->mutable_pose()->set_x(waypointDefinition[0]);
    cartesianCoordinate->mutable_pose()->set_y(waypointDefinition[1]);
    cartesianCoordinate->mutable_pose()->set_z(waypointDefinition[2]);
    cartesianCoordinate->set_blending_radius(waypointDefinition[3]);

    cartesianCoordinate->mutable_pose()->set_theta_x(waypointDefinition[4]);
    cartesianCoordinate->mutable_pose()->set_theta_y(waypointDefinition[5]);
    cartesianCoordinate->mutable_pose()->set_theta_z(waypointDefinition[6]);    


    cartesianCoordinate->set_reference_frame(kReferenceFrame);
}

bool set_gripper_command(k_api::Base::BaseClient* base, float position)
{
    std::cout << "Gripper position " << position << std::endl;

    k_api::Base::GripperCommand gripper_command;

    gripper_command.set_mode(k_api::Base::GRIPPER_POSITION);

    auto finger = gripper_command.mutable_gripper()->add_finger();
    finger->set_finger_identifier(1);
    
    finger->set_value(position);
    base->SendGripperCommand(gripper_command);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    return true;
}

void vision_pose_callback(const geometry_msgs::Pose &msg)
{
    //need transfomration from quaternion to euler angle
    // geometry_msgs::Twist euler;
    // euler = tf.transformations.euler_from_quaternion({0.0, 0.0, 0.0, .00}, axes='sxyz');

    waypointsDefinition_vision = { {0.443,  -0.016,  0.13,   0.0f, -180.0, -16.5, 90.0}, //Pose_First_Latch_Up_Inclined
                                         };

}

void dynamixel_callback(const dynamixel_workbench_msgs::DynamixelStateList &msg)
{
  dynamixel_state_list_ = msg;
}

void set_drill_command(int current) // current max 1193
{
    std::cout << "drill command " << std::endl;

    drill.request.command = "";
    drill.request.id = 1;
    drill.request.addr_name = "Goal_Current";
    drill.request.value = current;
    dynamixel_client.call(drill);                 

    drill.request.command = "";
    drill.request.id = 2;
    drill.request.addr_name = "Goal_Current";
    drill.request.value = current;
    dynamixel_client.call(drill); 
}

bool set_waypoints_angular_trajectory(k_api::Base::BaseClient* base, std::vector<std::array<float,6>> jointPoses)
{
    bool success = false;

    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Create the trajectory 
    k_api::Base::WaypointList wpts = k_api::Base::WaypointList();

    // Binded to degrees of movement and each degrees correspond to one degree of liberty
    auto actuators = base->GetActuatorCount();
    uint32_t degreesOfFreedom = actuators.count();

    // Move arm with waypoints list            
    // auto jointPoses  = std::vector<std::array<float,6>>();
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Note : To customize this example for your needs an array of array is used containing the information needed :
    // all  values correspond to a joint/motor
    // If you have 6DoF the array will contain 6 positions expressed in degrees in float format.
    // If you have 7DoF the array will contain 7 positions expressed in degrees in float format.
    // You may overwrite the jointPose array for the proper arm
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    // jointPoses.push_back({ 360.0f, 35.6f, 281.8f, 0.8f,   23.8f, 88.9f  });
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    for(auto index = 0; index < jointPoses.size(); ++index)
    {
        k_api::Base::Waypoint *wpt = wpts.add_waypoints();
        if(wpt != nullptr)
        {
            wpt->set_name(std::string("waypoint_") + std::to_string(index));
            k_api::Base::AngularWaypoint *ang = wpt->mutable_angular_waypoint();
            if(ang != nullptr)
            {    
                for(auto angleIndex = 0;angleIndex < degreesOfFreedom; ++angleIndex)
                {
                    ang->add_angles(jointPoses.at(index).at(angleIndex));
                    ang->set_duration(5); 
                }
            }
        }        
    }

    // Connect to notification action topic
    std::promise<k_api::Base::ActionEvent> finish_promise_cart;
    auto finish_future_cart = finish_promise_cart.get_future();
    auto promise_notification_handle_cart = base->OnNotificationActionTopic( create_event_listener_by_promise(finish_promise_cart),
                                                                            k_api::Common::NotificationOptions());

    k_api::Base::WaypointValidationReport result;
    try
    {
        // Verify validity of waypoints
        auto validationResult = base->ValidateWaypointList(wpts);
        result = validationResult;
    }
    catch(k_api::KDetailedException& ex)
    {
        std::cout << "Try catch error on waypoint list" << std::endl;
        // You can print the error informations and error codes
        auto error_info = ex.getErrorInfo().getError();
        std::cout << "KDetailedoption detected what:  " << ex.what() << std::endl;
        
        std::cout << "KError error_code: " << error_info.error_code() << std::endl;
        std::cout << "KError sub_code: " << error_info.error_sub_code() << std::endl;
        std::cout << "KError sub_string: " << error_info.error_sub_string() << std::endl;

        // Error codes by themselves are not very verbose if you don't see their corresponding enum value
        // You can use google::protobuf helpers to get the string enum element for every error code and sub-code 
        std::cout << "Error code string equivalent: " << k_api::ErrorCodes_Name(k_api::ErrorCodes(error_info.error_code())) << std::endl;
        std::cout << "Error sub-code string equivalent: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(error_info.error_sub_code())) << std::endl;
        return false;
    }
    
    // Trajectory error report always exists and we need to make sure no elements are found in order to validate the trajectory
    if(result.trajectory_error_report().trajectory_error_elements_size() == 0)
    {    
        // Execute action
        try
        {
            // Move arm with waypoints list
            std::cout << "Moving the arm creating a trajectory of " << jointPoses.size() << " angular waypoints" << std::endl;
            base->ExecuteWaypointTrajectory(wpts);
        }
        catch(k_api::KDetailedException& ex)
        {
            std::cout << "Try catch error executing normal trajectory" << std::endl;
            // You can print the error informations and error codes
            auto error_info = ex.getErrorInfo().getError();
            std::cout << "KDetailedoption detected what:  " << ex.what() << std::endl;
            
            std::cout << "KError error_code: " << error_info.error_code() << std::endl;
            std::cout << "KError sub_code: " << error_info.error_sub_code() << std::endl;
            std::cout << "KError sub_string: " << error_info.error_sub_string() << std::endl;

            // Error codes by themselves are not very verbose if you don't see their corresponding enum value
            // You can use google::protobuf helpers to get the string enum element for every error code and sub-code 
            std::cout << "Error code string equivalent: " << k_api::ErrorCodes_Name(k_api::ErrorCodes(error_info.error_code())) << std::endl;
            std::cout << "Error sub-code string equivalent: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(error_info.error_sub_code())) << std::endl;
            return false;
        }
        // Wait for future value from promise
        const auto ang_status = finish_future_cart.wait_for(TIMEOUT_PROMISE_DURATION);

        base->Unsubscribe(promise_notification_handle_cart);

        if(ang_status != std::future_status::ready)
        {
            std::cout << "Timeout on action notification wait for angular waypoint trajectory" << std::endl;
        }
        else
        {
            const auto ang_promise_event = finish_future_cart.get();
            std::cout << "Angular waypoint trajectory completed" << std::endl;
            std::cout << "Promise value : " << k_api::Base::ActionEvent_Name(ang_promise_event) << std::endl; 

            success = true;

            // We are now ready to reuse the validation output to test default trajectory generated...
            // Here we need to understand that trajectory using angular waypoint is never optimized.
            // In other words the waypoint list is the same and this is a limitation of Kortex API for now
        }
    }
    else
    {
        std::cout << "Error found in trajectory" << std::endl; 
        result.trajectory_error_report().PrintDebugString();        
    }

    return success;
}
