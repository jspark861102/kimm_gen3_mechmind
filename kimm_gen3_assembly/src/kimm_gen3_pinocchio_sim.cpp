#include "kimm_gen3_assembly/kimm_gen3_pinocchio_sim.hpp"

using namespace std;
using namespace Eigen;
using namespace pinocchio;

int main(int argc, char **argv)
{
    /////////////////////////// Setting ////////////////////////////////////////////////    
    ros::init(argc, argv, "kimm_gen3_pinocchio_sim");
    ros::NodeHandle n_node;
    ros::Rate loop_rate(SAMPLING_RATE);

    // Mujoco Subs
    jointState = n_node.subscribe("mujoco_ros/mujoco_ros_interface/joint_states", 5, &JointStateCallback, ros::TransportHints().tcpNoDelay(true));
    mujoco_command_sub = n_node.subscribe("mujoco_ros/mujoco_ros_interface/sim_command_sim2con", 5, &simCommandCallback, ros::TransportHints().tcpNoDelay(true));
    mujoco_time_sub = n_node.subscribe("mujoco_ros/mujoco_ros_interface/sim_time", 1, &simTimeCallback, ros::TransportHints().tcpNoDelay(true));

    // Mujoco Pubs
    mujoco_command_pub_ = n_node.advertise<std_msgs::String>("mujoco_ros/mujoco_ros_interface/sim_command_con2sim", 5);
    robot_command_pub_ = n_node.advertise<mujoco_ros_msgs::JointSet>("mujoco_ros/mujoco_ros_interface/joint_set", 5);
    mujoco_run_pub_ = n_node.advertise<std_msgs::Bool>("mujoco_ros/mujoco_ros_interface/sim_run", 5);

    // Mujoco Msg
    robot_command_msg_.position.resize(GEN3_DOF); 
    robot_command_msg_.torque.resize(GEN3_DOF); 

    // Ros Param
    string urdf_name, urdf_path;
    n_node.getParam("urdf_path", urdf_path);
    n_node.getParam("urdf_name", urdf_name);

    // Pinocchio
    vector<string> package_dirs;
    package_dirs.push_back(urdf_path);
    std::string urdfFileName = package_dirs[0] + "/" + urdf_name;
    robot_ = std::make_shared<RobotWrapper>(urdfFileName, package_dirs, false);  
    model_ = robot_->model();
    Data data(model_);
    data_ = data;

    // Control Variable
    ctrl_mode_ = 0;
    chg_flag_ = false;
    state_.J.resize(6, GEN3_DOF);   

    state_.q_des.setZero();
    state_.q_des_pre.setZero();   
    state_.v_des.setZero();
    state_.ddq_des.setZero();
    state_.tau_des.setZero();    
    ////////////////////////////////////////////////////////////////////////////////////////      

    Home << 0.00, 45.0, 90.0, 0.0, 0.0, 0.0;
    Home_cartesian << 0.575, -0.005, 0.419, 90.3, 0.0, 89.1;
    ////////////////////////////////////////////////////////////////////////////////////////

    Kp << 500., 500., 500., 300., 300., 300.;
    Kd << 2.0*Kp.cwiseSqrt();
    ////////////////////////////////////////////////////////////////////////////////////////

    while (ros::ok()){
        keyboard_event();    
        
        std_msgs::String sim_run_msg_;
        sim_run_msg_.data = true;
        mujoco_run_pub_.publish(sim_run_msg_);

        robot_->computeAllTerms(data_, state_.q, state_.v);

#if GEN3_DOF == 6
        robot_->jacobianWorld(data_, model_.getJointId("Actuator6"), state_.J);
#else
        robot_->jacobianWorld(data_, model_.getJointId("Actuator7"), state_.J);
#endif                

        if (ctrl_mode_== 0){
            state_.q_des.setZero();
            state_.tau_des.setZero();    
        }
        if (ctrl_mode_ == 1){ // home joint
            if (chg_flag_){
                cubic_.stime = time_;
                cubic_.ftime = time_+ 2.0;
                cubic_.q0 = state_.q;
                cubic_.v0 = state_.v;                

                for (int i = 0; i<GEN3_DOF; i++)
                {
                    q_target_(i) = Home(i) * M_PI / 180.;
                }               

                chg_flag_ = false;
            }            
            for (int i=0; i<GEN3_DOF; i++)
            {
                state_.q_des(i) = cubic(time_, cubic_.stime, cubic_.ftime, cubic_.q0(i), q_target_(i), cubic_.v0(i), 0.0);                            

                state_.v_des(i) = (state_.q_des(i) - state_.q_des_pre(i)) * SAMPLING_RATE;
                state_.q_des_pre(i) = state_.q_des(i);
                
                state_.ddq_des(i) = -Kp(i)*(state_.q(i) - state_.q_des(i)) -Kd(i)*(state_.v(i) - state_.v_des(i));
            }
            // cout << state_.q - state_.q_des << endl;            
            // cout << data_.nle << endl;    
            // cout << state_.tau_des << endl;    
            // cout << time_ << endl;    
            // cout << " " << endl;            
        }
        if (ctrl_mode_ == 2){ // home cartesian
            if (chg_flag_){
                cubic_.stime = time_;
                cubic_.ftime = time_+ 2.0;
                cubic_.q0 = state_.q;
                cubic_.v0 = state_.v;                

                for (int i = 0; i<GEN3_DOF; i++)
                {
                    X_target_(i) = Home_cartesian(i);
                }               

                chg_flag_ = false;
            }            
            for (int i=0; i<GEN3_DOF; i++)
            {
                state_.q_des(i) = cubic(time_, cubic_.stime, cubic_.ftime, cubic_.q0(i), q_target_(i), cubic_.v0(i), 0.0);                            
                state_.v_des(i) = (state_.q_des(i) - state_.q_des_pre(i)) * SAMPLING_RATE;
                state_.q_des_pre(i) = state_.q_des(i);
                
                state_.ddq_des(i) = -Kp(i)*(state_.q(i) - state_.q_des(i)) -Kd(i)*(state_.v(i) - state_.v_des(i));
            }            
        }
    
        ////////////////////////////////////////////////////////////////////////////////////////
        state_.tau_des = data_.M * state_.ddq_des + data_.nle;
        robot_command();        
                        
        // cout << data_.M.col(1) << endl;
        // cout << " " << endl;

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
            case 'h': //home joint
                ctrl_mode_ = 1;
                chg_flag_ = true;
                cout << " " << endl;
                cout << "Move to Home Position" << endl;
                cout << " " << endl;
                break;
            case 'a': //home cartesian
                ctrl_mode_= 2;
                chg_flag_ = true;
                cout << " " << endl;
                cout << "Move to Home Position" << endl;
                cout << " " << endl;
                break;  
        }
    }
}

void simCommandCallback(const std_msgs::StringConstPtr &msg){
    std::string buf;
    buf = msg->data;

    if (buf == "RESET")
    {
        std_msgs::String rst_msg_;
        rst_msg_.data = "RESET";
        mujoco_command_pub_.publish(rst_msg_);
    }

    if (buf == "INIT")
    {
        std_msgs::String rst_msg_;
        rst_msg_.data = "INIT";
        mujoco_command_pub_.publish(rst_msg_);
        mujoco_time_ = 0.0;
    }
}

void simTimeCallback(const std_msgs::Float32ConstPtr &msg){
    mujoco_time_ = msg->data;
    time_ = mujoco_time_;
}

void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
  for (int i=0; i<GEN3_DOF; i++){
    state_.q(i) = msg->position[i];
    state_.v(i) = msg->velocity[i];
  }
}

void robot_command()
{
    robot_command_msg_.MODE = 1;
        robot_command_msg_.header.stamp = ros::Time::now();
    robot_command_msg_.time = time_;
    for (int i=0; i<GEN3_DOF; i++)
    {
        // robot_command_msg_.position[i] = state_.q_des(i);
        robot_command_msg_.torque[i] = state_.tau_des(i);
    }
    robot_command_pub_.publish(robot_command_msg_); 
}
