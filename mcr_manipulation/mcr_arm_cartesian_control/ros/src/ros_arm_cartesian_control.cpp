/*
 * ros_arm_cartesian_control.cpp
 *
 *  Created on: Nov 29, 2012
 *      Author: matthias
 *  Modified on: Mar 1, 2019
 *      Author: Abhishek Padalkar
 */

#include <mcr_arm_cartesian_control/ros_arm_cartesian_control.h>

RosArmCartesianControl::RosArmCartesianControl()
{
    ros::Subscriber sub_joint_states = node_handle.subscribe(joint_state_topic,
                                       1, ros_arm_cc_.jointstateCallback);

    ros::Subscriber sub_wjs = node_handle.subscribe(weight_js_topic, 1,
                             ros_arm_cc_.wjsCallback);
    
    ros::Subscriber sub_wts = node_handle.subscribe(weight_ts_topic, 1,
                             ros_arm_cc_.wtsCallback);

    ros::Subscriber sub_cc = node_handle.subscribe(cart_control_topic, 1,
                             ros_arm_cc_.ccCallback);
}

void RosArmCartesianControl::jointstateCallback(sensor_msgs::JointStateConstPtr joints)
{

    for (unsigned i = 0; i < joints->position.size(); i++)
    {

        const char* joint_uri = joints->name[i].c_str();

        for (unsigned int j = 0; j < arm_chain.getNrOfJoints(); j++)
        {
            const char* chainjoint =
                arm_chain.getSegment(j).getJoint().getName().c_str();

            if (chainjoint != 0 && strcmp(chainjoint, joint_uri) == 0)
            {
                joint_positions.data[j] = joints->position[i];
                joint_positions_initialized[j] = true;
            }
        }
    }
}

void RosArmCartesianControl::wtsCallback(std_msgs::Float32MultiArray weights)
{
    weight_ts.resize(6, 6);
    weight_ts.setIdentity();
    weight_ts(0, 0) = weights.data[0];
    weight_ts(1, 1) = weights.data[1];
    weight_ts(2, 2) = weights.data[2];
    weight_ts(3, 3) = weights.data[3];
    weight_ts(4, 4) = weights.data[4];
    weight_ts(5, 5) = weights.data[5];
    ((KDL::ChainIkSolverVel_wdls*) ik_solver)->setWeightTS(weight_ts);
}

void RosArmCartesianControl::wjsCallback(std_msgs::Float32MultiArray weights)
{
    weight_js.resize(nrOfJoints, nrOfJoints);
    weight_js.setIdentity();
    for (int i = 0; i < nrOfJoints; i++)
    {
        weight_js(i,i) = weights.data[i];
    }
    ((KDL::ChainIkSolverVel_wdls*) ik_solver)->setWeightJS(weight_js);
}

void RosArmCartesianControl::ccCallback(geometry_msgs::TwistStampedConstPtr desiredVelocity)
{

    for (size_t i = 0; i < joint_positions_initialized.size(); i++)
    {
        if (!joint_positions_initialized[i])
        {
            std::cout << "joints not initialized" << std::endl;
            return;
        }
    }

    if (!tf_listener) return;

    geometry_msgs::Vector3Stamped linear_in;
    geometry_msgs::Vector3Stamped linear_out;
    linear_in.header = desiredVelocity->header;
    linear_in.vector = desiredVelocity->twist.linear;

    try
    {
        tf_listener->transformVector(root_name, linear_in, linear_out);
    }
    catch (...)
    {
        ROS_ERROR("Could not transform frames %s -> %s for linear transformation", desiredVelocity->header.frame_id.c_str(), root_name.c_str());
        return;
    }
    geometry_msgs::Vector3Stamped angular_in;
    geometry_msgs::Vector3Stamped angular_out;
    angular_in.header = desiredVelocity->header;
    angular_in.vector = desiredVelocity->twist.angular;

    try
    {
        tf_listener->transformVector(root_name, angular_in, angular_out);
    }
    catch (...)
    {
        ROS_ERROR("Could not transform frames %s -> %s for angular transformation", desiredVelocity->header.frame_id.c_str(), root_name.c_str());
        return;
    }
    targetVelocity.vel.data[0] = linear_out.vector.x;
    targetVelocity.vel.data[1] = linear_out.vector.y;
    targetVelocity.vel.data[2] = linear_out.vector.z;

    targetVelocity.rot.data[0] = angular_out.vector.x;
    targetVelocity.rot.data[1] = angular_out.vector.y;
    targetVelocity.rot.data[2] = angular_out.vector.z;

    t_last_command = ros::Time::now();

    active = true;
}


void RosArmCartesianControl::init_ik_solver()
{

    if (ik_solver != 0)
    {
        return;
    }

    ik_solver = new KDL::ChainIkSolverVel_wdls(arm_chain);
    weight_ts.resize(6, 6);
    weight_ts.setIdentity();

    weight_ts(0, 0) = 1;
    weight_ts(1, 1) = 1;
    weight_ts(2, 2) = 1;
    weight_ts(3, 3) = 0.4;
    weight_ts(4, 4) = 0.4;
    weight_ts(5, 5) = 0.4;
    ((KDL::ChainIkSolverVel_wdls*) ik_solver)->setWeightTS(weight_ts);

    weight_js = (Eigen::MatrixXd::Identity(arm_chain.getNrOfJoints(),
                                           arm_chain.getNrOfJoints()));
    //weight_js(0, 0) = 0.5;
    //weight_js(1,1) = 1;
    //weight_js(2,2) = 1;
    //weight_js(3,3) = 1;
    //weight_js(4,4) = 0.1;
    //((KDL::ChainIkSolverVel_wdls*) ik_solver)->setWeightJS(weight_js);


    ((KDL::ChainIkSolverVel_wdls*) ik_solver)->setLambda(10000.0);
}

void RosArmCartesianControl::init_joint_msgs()
{
    joint_positions_initialized.resize(arm_chain.getNrOfJoints(), false);
    jointMsg.velocities.resize(arm_chain.getNrOfJoints());
    for (unsigned int i = 0; i < arm_chain.getNrOfSegments(); i++)
    {
        jointMsg.velocities[i].joint_uri =
            arm_chain.getSegment(i).getJoint().getName();
        jointMsg.velocities[i].unit = "s^-1 rad";
    }
}

void RosArmCartesianControl::publishJointVelocities(KDL::JntArrayVel& joint_velocities)
{

    for (unsigned int i = 0; i < joint_velocities.qdot.rows(); i++)
    {
        jointMsg.velocities[i].value = joint_velocities.qdot(i);
        ROS_DEBUG("%s: %.5f %s", jointMsg.velocities[i].joint_uri.c_str(),
                  jointMsg.velocities[i].value, jointMsg.velocities[i].unit.c_str());
        if (isnan(jointMsg.velocities[i].value))
        {
            ROS_ERROR("invalid joint velocity: nan");
            return;
        }
        if (fabs(jointMsg.velocities[i].value) > 1.0)
        {
            ROS_ERROR("invalid joint velocity: too fast");
            return;
        }
    }
    cmd_vel_publisher.publish(jointMsg);
}


void RosArmCartesianControl::publishJointVelocities_FA(KDL::JntArrayVel& joint_velocities)
{
    std_msgs::Float32MultiArray joint_velocitiy_array;
    joint_velocitiy_array.data.clear();
    for (unsigned int i = 0; i < joint_velocities.qdot.rows(); i++)
    {
        joint_velocitiy_array.data.push_back(joint_velocities.qdot(i));
        if (isnan(joint_velocities.qdot(i)))
        {
            ROS_ERROR("invalid joint velocity: nan");
            return;
        }
        if (fabs(joint_velocities.qdot(i)) > 1.0)
        {
            ROS_ERROR("invalid joint velocity: too fast");
            return;
        }
    }
    cmd_vel_publisher.publish(joint_velocitiy_array);
}


void RosArmCartesianControl::stopMotion()
{
    if (use_float_array_msg == false){
        for (unsigned int i = 0; i < jointMsg.velocities.size(); i++)
        {
            jointMsg.velocities[i].value = 0.0;

        }
        cmd_vel_publisher.publish(jointMsg);
    }
    else{
        std_msgs::Float32MultiArray joint_velocitiy_array;
        for (unsigned int i = 0; i < nrOfJoints; i++)
        {
            joint_velocitiy_array.data.push_back(0.0);
        }
        cmd_vel_publisher.publish(joint_velocitiy_array);
    }
}


bool RosArmCartesianControl::watchdog()
{

    double watchdog_time = 0.3;
    if (active == false)
    {
        return false;
    }

    ros::Time now = ros::Time::now();

    ros::Duration time = (now - t_last_command);

    if (time > ros::Duration(watchdog_time))
    {
        active = false;
        stopMotion();
        return false;
    }

    return true;
}

