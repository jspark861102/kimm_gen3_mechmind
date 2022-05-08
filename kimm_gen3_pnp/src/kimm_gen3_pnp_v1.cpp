#include "kimm_gen3_pnp/kimm_gen3_pnp_v1.hpp"

//delete ee state publish
//check lowlevel control : position/velocity/torque
//start with low level pos and vel and then torque
//check the rate of three methods

using namespace std;
using namespace k_api;

#define SAMPLING_RATE 1000

int main(int argc, char **argv)
{
    /////////////////////////// ROS Setting ////////////////////////////////////////////////    
    ros::init(argc, argv, "kinova_controller");
    ros::NodeHandle n_node;
    ros::Rate loop_rate(SAMPLING_RATE);
    
    mech_pose_sub = n_node.subscribe("mechmind_publisher/pose", 100, mechmind_pose_callback);    
    vision_pose_sub = n_node.subscribe("kimm_aruco_publisher/pose", 100, aruco_pose_callback);    
    mech_call_pub = n_node.advertise<std_msgs::String>("kimm_gen3_pnp/mech_call",5);
    aruco_call_pub = n_node.advertise<std_msgs::String>("kimm_gen3_pnp/aruco_call",5);
    joint_state_pub = n_node.advertise<sensor_msgs::JointState>("kimm_gen3_pnp/joint_states", 5);  
    ////////////////////////////////////////////////////////////////////////////////////////    

    /////////////////////////// Kinova Setting /////////////////////////////////////////////        
    n_node.getParam("/kimm_gen3_pnp_v1/port_number", PORT);
    n_node.getParam("/kimm_gen3_pnp_v1/port_realtime_number", PORT_REAL_TIME);
    n_node.getParam("/kimm_gen3_pnp_v1/ip_address", ip_address);
    n_node.getParam("/kimm_gen3_pnp_v1/username", username);
    n_node.getParam("/kimm_gen3_pnp_v1/password", password);    
    
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

    /////////////////////////// Control Setting /////////////////////////////////////////////        
    //Control setting
    ctrl_flag_ = 0;
    COMMAND_SUCCEESS_ = true;           
    
    base_feedback = base_cyclic->RefreshFeedback();  
    total_joint_size =  base_feedback.actuators_size() + base_feedback.interconnect().gripper_feedback().motor_size();
    joint_state.name.resize(total_joint_size);
    joint_state.position.resize(total_joint_size);
    joint_state.velocity.resize(total_joint_size);
    joint_state.effort.resize(total_joint_size);
    ros::param::get("~joint_names", m_arm_joint_names);    
    ros::param::get("~gripper_joint_names", m_gripper_joint_names);

    //gripper control setting
    isgripper = false;    

    //vision callback setting
    ismech = false;
    isaruco = false;
    iscallback_mech = false;
    iscallback_aruco = false;    
    
    ////////////////////////////////////////////////////////////////////////////////////////            
    Wait.push_back({ 271.32, 343.82, 248.61, 180.58, 84.38, 181.31});                    
    waypointsDefinition_wait_for_MechMind =  { {0.007,  0.200,  0.36,  0.0f, 180.0,  0.0, 90.0},}; //initial pose is MyHome pose 
    
    VisionHome.push_back({ 325.65, 5.91, 268.31, 180.58, 81.6, 235.48});                    
    waypointsDefinition_vision_Home =  { {0.296,  0.200,  0.36,  0.0f, 180.0,  0.0, 90.0},}; //initial pose is MyHome pose

    waypointsDefinition_vision_up   =  { {0.296,  0.200,  0.36,  0.0f, 180.0,  0.0, 90.0},};    
    waypointsDefinition_vision_down =  { {0.296,  0.200,  0.36,  0.0f, 180.0,  0.0, 90.0},}; 

    VisionPlace.push_back({ 345.58, 19.51, 263.21, 180.04, 62.75, 256.23});            
    // VisionPlace.push_back({ 345.58, 19.51, 263.21, 180.04, 62.75, 76.22});            
    waypointsDefinition_vision_place =  { {0.408,  0.105,  0.24,  0.0f, 180.0,  0.0, 90.0},}; //initial pose is MyHome pose                        
    ////////////////////////////////////////////////////////////////////////////////////////

    while (ros::ok()){
        keyboard_event();            
        publish_cyclic_feedback(base, base_cyclic);
        
        if (ctrl_flag_ == 1 && !COMMAND_SUCCEESS_){ //move to vision home position, h
            COMMAND_SUCCEESS_ = true; 
            base->ClearFaults();        
            set_waypoints_angular_trajectory(base, Wait);
        }      
        if (ctrl_flag_ == 2 && !COMMAND_SUCCEESS_){ //vision waypoint, v                                             
            if (!ismech) 
            {
                //call obect pose w.r.t marker            
                std_msgs::String mech_msg;
                mech_msg.data = "mech_call";                     
                mech_call_pub.publish(mech_msg);
                ismech = true; //to publish only once during the iteration
            }

            if (iscallback_mech)                   
            {                
                cout << "mechdata is received" << endl;
                set_waypoint_trajectory(base, waypointsDefinition_vision_Home, 0.0);                                        
            }

            if(iscallback_mech && !isaruco) 
            {
                //call marker pose w.r.t. hand camera 
                std_msgs::String aruco_msg;
                aruco_msg.data = "aruco_call";                     
                aruco_call_pub.publish(aruco_msg);
                isaruco = true; //to publish only once during the iteration                
            }                        
            if (iscallback_aruco)
            {                                
                cout << "aruco data is received" << endl;
                set_waypoint_trajectory(base, waypointsDefinition_vision_up, 0.0);        
                set_gripper_command(base, 0.50);                                              
                set_waypoint_trajectory(base, waypointsDefinition_vision_down, 0.0);        
                set_gripper_command(base, 0.90);                                              
                set_waypoint_trajectory(base, waypointsDefinition_vision_up, 0.0);        
                // set_waypoint_trajectory(base, waypointsDefinition_vision_place, 0.0);        
                set_gripper_command(base, 0.5);  
                ROS_WARN("pick and place is complete");
                
                COMMAND_SUCCEESS_ = true;                        
                ismech = false;
                isaruco = false;
                iscallback_mech = false; 
                iscallback_aruco = false;
            }            
        }                
        if (ctrl_flag_ == 99 && !COMMAND_SUCCEESS_) { //gripper, z
            COMMAND_SUCCEESS_ = true;                        
            if (!isgripper){
                isgripper = true;  
                set_gripper_command(base, 0.0);                         
            }                
            else{
                isgripper = false;  
                set_gripper_command(base, 0.99);                                  
            }                
        }
        if (ctrl_flag_ == 999 && !COMMAND_SUCCEESS_){ // Close API session, q             
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
            case 'v': //vision waypoints
                ctrl_flag_ = 2;
                COMMAND_SUCCEESS_ = false;
                cout << " " << endl;
                cout << "vision waypoints" << endl;
                cout << " " << endl;
                break;                        
            case 'z': //gripper
                ctrl_flag_ = 99;                
                COMMAND_SUCCEESS_ = false;
                cout << " " << endl;
                cout << "gripper" << endl;
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

void publish_cyclic_feedback(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
{    
    base_feedback = base_cyclic->RefreshFeedback();    

    ///////////////////////// joint_state ///////////////////////////
    joint_state.header.stamp = ros::Time::now();    
    // joint_state.name = {"Actuator1", "Actuator2", "Actuator3", "Actuator4", "Actuator5","Actuator6"};
    
    for (int i = 0; i < base_feedback.actuators_size(); i++) //arm
    {
        joint_state.name[i] = m_arm_joint_names[i];
        joint_state.position[i] = wrapRadiansFromMinusPiToPi(base_feedback.actuators(i).position() * M_PI / 180.0); // [rad]
        joint_state.velocity[i] = base_feedback.actuators(i).velocity() * M_PI / 180.0; // [rad/sec]
        joint_state.effort[i] = base_feedback.actuators(i).torque(); //[Nm]
    }

    if (base_feedback.has_interconnect()) //gripper
    {
        joint_state.name[total_joint_size-1] = m_gripper_joint_names[0];
        joint_state.position[total_joint_size-1] = base_feedback.interconnect().gripper_feedback().motor(0).position(); // 0~100 [%] 
        joint_state.velocity[total_joint_size-1] = base_feedback.interconnect().gripper_feedback().motor(0).velocity(); // 0~100 [%]
        joint_state.effort[total_joint_size-1] = base_feedback.interconnect().gripper_feedback().motor(0).current_motor(); // [mA]
    }

    ///////////////////////// feedback publish ///////////////////////////
    joint_state_pub.publish(joint_state);   
}

void mechmind_pose_callback(const geometry_msgs::Pose &msg)
{   
    mech_pose = msg;
    iscallback_mech = true;
}


void aruco_pose_callback(const geometry_msgs::Pose &msg)
{   
    pose_calculation(msg);    
}

void pose_calculation(const geometry_msgs::Pose &msg)
{    
    //z축 좌표계 맞춰야 함 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1

    //////// transformation from marker to object ////////
    Vector3d t_obj;
    t_obj(0) = mech_pose.position.x; 
    t_obj(1) = mech_pose.position.y; 
    t_obj(2) = mech_pose.position.z; 
    ///////////////////////////////////////////////////////    

    //////// transformation from "camera_color_optical_frame" to "tool_frame" ////////    
    tf::TransformListener tf_listener;
    tf::StampedTransform echo_transform;    

    // Wait for up to one second for the first transforms to become avaiable. 
    tf_listener.waitForTransform("/camera_color_optical_frame", "/tool_frame", ros::Time(), ros::Duration(1.0));
    tf_listener.lookupTransform("/camera_color_optical_frame",  "/tool_frame", ros::Time(), echo_transform);
    
    Vector3d CtoT;    
    CtoT(0)  = echo_transform.getOrigin().getY();
    CtoT(1)  = echo_transform.getOrigin().getX();
    CtoT(2)  = -echo_transform.getOrigin().getZ() - (0.1628-0.12);  //close length : 0.1628, tf length : 0.12          

    // CtoT(0)  = 0.107;
    // CtoT(1)  = 0.027;
    // CtoT(2)  = -0.11;    
    ///////////////////////////////////////////////////////

    ////////  transfromation from camera to marker //////// 
    Vector3d t;
    t(0) = -msg.position.y; 
    t(1) = -msg.position.x; 
    t(2) =  msg.position.z; 

    Quaterniond q;
    q.x() = msg.orientation.x;
    q.y() = msg.orientation.y;
    q.z() = msg.orientation.z;
    q.w() = msg.orientation.w;
    q.normalize();
    // cout << q.coeffs() << endl;
    ///////////////////////////////////////////////////////

    //it is not exact, but used ,for now, just for yaw angle transformation
    double roll = 0.0, pitch = 0.0, yaw = -1.5708;    
    Quaterniond qrot;
    qrot = AngleAxisd(roll, Vector3d::UnitX())* AngleAxisd(pitch, Vector3d::UnitY())* AngleAxisd(yaw, Vector3d::UnitZ());     
    q = qrot * q; 

    Vector3d rpy = q.toRotationMatrix().eulerAngles(0,1,2)*180.0/M_PI;
    if (fabs(rpy(0)) > 90 ) rpy(2) = 180.0 + rpy(2);
    //
    
    cout << "received data from aruco" << endl;
    cout << "[" << msg.position.x << "  " << msg.position.y << "  " << msg.position.z << "  " << rpy(0,0) << "  " << rpy(1,0) << "  " << rpy(2,0) << "]" << endl;
    cout << " " << endl;

    waypointsDefinition_vision_up.at(0).at(0) = t(0)                    + CtoT(0) + waypointsDefinition_vision_Home.at(0).at(0) + t_obj(0);
    waypointsDefinition_vision_up.at(0).at(1) = t(1)                    + CtoT(1) + waypointsDefinition_vision_Home.at(0).at(1) + t_obj(1);;
    waypointsDefinition_vision_up.at(0).at(2) = t(2)                    + CtoT(2);    
    waypointsDefinition_vision_up.at(0).at(4) = 0.0                     + waypointsDefinition_vision_Home.at(0).at(4); // for now, tilt angle of objecct is not considered 
    waypointsDefinition_vision_up.at(0).at(5) = 0.0                     + waypointsDefinition_vision_Home.at(0).at(5); // for now, tilt angle of objecct is not considered 
    waypointsDefinition_vision_up.at(0).at(6) = (float) rpy(2,0)        + waypointsDefinition_vision_Home.at(0).at(6); //aruco marker tf is rotated with 90degree

    waypointsDefinition_vision_down.at(0).at(0) = t(0)                    + CtoT(0) + waypointsDefinition_vision_Home.at(0).at(0) + t_obj(0);;
    waypointsDefinition_vision_down.at(0).at(1) = t(1)                    + CtoT(1) + waypointsDefinition_vision_Home.at(0).at(1) + t_obj(1);;
    waypointsDefinition_vision_down.at(0).at(2) = 0.0                     + 0.017; //0.013; //down for 1.3cm    
    waypointsDefinition_vision_down.at(0).at(4) = 0.0                     + waypointsDefinition_vision_Home.at(0).at(4); // for now, tilt angle of objecct is not considered 
    waypointsDefinition_vision_down.at(0).at(5) = 0.0                     + waypointsDefinition_vision_Home.at(0).at(5); // for now, tilt angle of objecct is not considered 
    waypointsDefinition_vision_down.at(0).at(6) = (float) rpy(2,0)        + waypointsDefinition_vision_Home.at(0).at(6); //aruco marker tf is rotated with 90degree

    cout << "final waypoint for gen3" << endl;
    cout << "[" << waypointsDefinition_vision_up.at(0).at(0) << "  " << waypointsDefinition_vision_up.at(0).at(1) << "  " << waypointsDefinition_vision_up.at(0).at(2) << "  " << waypointsDefinition_vision_up.at(0).at(4) << "  " << waypointsDefinition_vision_up.at(0).at(5) << "  " << waypointsDefinition_vision_up.at(0).at(6) << "]" << endl;    
    cout << " " << endl;

    // cout <<"z axis" <<ndl;
    // cout << t_obj(2) - ( eCtoT(2) + t(2)) + waypointsDefinition_vision_Home.at(0).at(2)<< endl;

    iscallback_aruco = true;
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
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    return true;
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
