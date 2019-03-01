/*
 * ros_arm_cartesian_control.cpp
 *
 *  Created on: Nov 29, 2012
 *      Author: matthias
 *  Modified on: Mar 1, 2019
 *      Author: Abhishek Padalkar
 */

#include <mcr_arm_cartesian_control/ros_arm_cartesian_control.h>
#include <tf/transform_listener.h>

int main(int argc, char **argv)
{   

    ros::init(argc, argv, "arm_cartesian_control");
    ros::NodeHandle node_handle("~");
    RosArmCartesianControl ros_arm_cc_;
    ros_arm_cc_.tf_listener = new tf::TransformListener();

    double rate = 50;


    //TODO: read from param
    std::string velocity_command_topic = "joint_velocity_command";
    std::string sigma_values_topic = "sigma_values";
    std::string weight_ts_topic = "weight_task_space";
    std::string weight_js_topic = "weight_joint_space";
    std::string joint_state_topic = "/joint_states";
    std::string cart_control_topic = "cartesian_velocity_command";

    std::string tooltip_name = "DEFAULT_CHAIN_TIP";

    node_handle.getParam("use_float_array_msg", ros_arm_cc_.use_float_array_msg);
    node_handle.getParam("joint_state_topic", joint_state_topic);

    if (!node_handle.getParam("root_name", ros_arm_cc_.root_name))
    {
        ROS_ERROR("No parameter for root_name specified");
        return -1;
    }
    ROS_INFO("Using %s as chain root [param: root_name]", ros_arm_cc_.root_name.c_str());

    if (!node_handle.getParam("tip_name", tooltip_name))
    {
        ROS_ERROR("No parameter for tip_name specified");
        return -1;
    }
    ROS_INFO("Using %s as tool tip [param: tip_name]", tooltip_name.c_str());




    //load URDF model
    ROS_URDF_Loader loader;
    loader.loadModel(node_handle, ros_arm_cc_.root_name, tooltip_name, ros_arm_cc_.arm_chain, ros_arm_cc_.joint_limits);
    
    //init
    ros_arm_cc_.nrOfJoints = ros_arm_cc_.arm_chain.getNrOfJoints();
    ros_arm_cc_.joint_positions.resize(ros_arm_cc_.nrOfJoints);
    ros_arm_cc_.joint_positions_initialized.resize(ros_arm_cc_.nrOfJoints, false);
    std_msgs::Float32MultiArray sigma_array;

    ros_arm_cc_.init_ik_solver();

    if (ros_arm_cc_.use_float_array_msg == false){
        ros_arm_cc_.init_joint_msgs();
    }

    //fk_solver = new KDL::ChainFkSolverPos_recursive(arm_chain);
    //jnt2jac = new KDL::ChainJntToJacSolver(arm_chain);

    //sigma values publisher 
    ros_arm_cc_.sigma_publisher = node_handle.advertise<std_msgs::Float32MultiArray>(
                            sigma_values_topic, 1);

    if (ros_arm_cc_.use_float_array_msg == false){
        //register publisher with brics actuator message
        ros_arm_cc_.cmd_vel_publisher = node_handle.advertise<brics_actuator::JointVelocities>(
                            velocity_command_topic, 1);
    }
    else{
        //register publisher with brics float array      
        ros_arm_cc_.cmd_vel_publisher = node_handle.advertise<std_msgs::Float32MultiArray>(
                            velocity_command_topic, 1);
    }
    //register subscriber
    ros::Subscriber sub_joint_states = node_handle.subscribe(joint_state_topic,
                                       1, ros_arm_cc_.jointstateCallback);

    ros::Subscriber sub_wjs = node_handle.subscribe(weight_js_topic, 1,
                             ros_arm_cc_.wjsCallback);
    
    ros::Subscriber sub_wts = node_handle.subscribe(weight_ts_topic, 1,
                             ros_arm_cc_.wtsCallback);

    ros::Subscriber sub_cc = node_handle.subscribe(cart_control_topic, 1,
                             ros_arm_cc_.ccCallback);
    

    arm_cc::Arm_Cartesian_Control control(&ros_arm_cc_.arm_chain, ros_arm_cc_.ik_solver);
    std::vector<double> upper_limits;
    std::vector<double> lower_limits;

    for (unsigned int i = 0; i < ros_arm_cc_.joint_limits.size(); i++)
    {
        upper_limits.push_back(ros_arm_cc_.joint_limits[i]->upper);
        lower_limits.push_back(ros_arm_cc_.joint_limits[i]->lower);
    }
    control.setJointLimits(lower_limits, upper_limits);

    KDL::JntArrayVel cmd_velocities(ros_arm_cc_.nrOfJoints);

    //loop with 50Hz
    ros::Rate loop_rate(rate);

    while (ros::ok())
    {

        ros::spinOnce();

        if (ros_arm_cc_.watchdog())
        {
            control.process(1 / rate, ros_arm_cc_.joint_positions, ros_arm_cc_.targetVelocity, cmd_velocities, ros_arm_cc_.sigma);
            
            sigma_array.data.clear();
            if (ros_arm_cc_.sigma.size() != 0)
            {
                for (int i = 0; i < ros_arm_cc_.nrOfJoints; i++)
                {
                    sigma_array.data.push_back(ros_arm_cc_.sigma[i]);
                }
            }
            
            ros_arm_cc_.sigma_publisher.publish(sigma_array);
            if (ros_arm_cc_.use_float_array_msg == false)
            {
                ros_arm_cc_.publishJointVelocities(cmd_velocities);
            }
            else
            {
                ros_arm_cc_.publishJointVelocities_FA(cmd_velocities);
            }
        }


        loop_rate.sleep();
    }


    return 0;
}
