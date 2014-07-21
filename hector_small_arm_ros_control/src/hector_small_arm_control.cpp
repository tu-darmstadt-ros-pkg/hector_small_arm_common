/*********************************************************************
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Christian Rose
 *                      Team Hector,
 *                      Technische Universität Darmstadt
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Technische Universität Darmstadt nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <hector_small_arm_ros_control/hector_small_arm_control.hpp>

#include <controller_manager/controller_manager.h>

namespace hector_small_arm_control
{

Hector_Small_Arm_Control::Hector_Small_Arm_Control()
{
    joint_name_vector_.push_back("joint_0");
    joint_name_vector_.push_back("joint_1");
    joint_name_vector_.push_back("joint_2");
    joint_name_vector_.push_back("joint_3");
    joint_name_vector_.push_back("joint_4");
    joint_name_vector_.push_back("joint_5");

    joint_offset["joint_0"] = -0.06135923151542565;
    joint_offset["joint_1"] = -1.5288675185926892;
    joint_offset["joint_2"] = 1.5135277107138327;
    joint_offset["joint_3"] = 0.5011003907093095;
    joint_offset["joint_4"] = 0.43974115919388385;
    joint_offset["joint_5"] = 0;

  for(unsigned int i=0; i<joint_name_vector_.size(); i++)
    {
        joint_positions_[joint_name_vector_[i]] = 0.0;
        joint_velocitys_[joint_name_vector_[i]] = 0.0;
        joint_efforts_[joint_name_vector_[i]] = 0.0;

        joint_cmd_pubs_[joint_name_vector_[i]] = nh_.advertise<std_msgs::Float64>("/" + joint_name_vector_[i] + "/command", 5);

        ros::Subscriber sub = nh_.subscribe("/" + joint_name_vector_[i] + "/state", 1, &Hector_Small_Arm_Control::jointStateCallback, this);
        joint_state_subs_[joint_name_vector_[i]] = sub;

        nh_.setCallbackQueue(&subscriber_queue_);

        hardware_interface::JointStateHandle state_handle(joint_name_vector_[i], &joint_positions_[joint_name_vector_[i]], &joint_velocitys_[joint_name_vector_[i]], &joint_efforts_[joint_name_vector_[i]]);
        joint_state_interface_.registerHandle(state_handle);

        hardware_interface::JointHandle pos_handle(joint_state_interface_.getHandle(joint_name_vector_[i]), &joint_pos_cmds_[joint_name_vector_[i]]);
        position_joint_interface_.registerHandle(pos_handle);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);

    subscriber_spinner_.reset(new ros::AsyncSpinner(1, &subscriber_queue_));
    subscriber_spinner_->start();

    _fake_dof_value = 0.0;
}

void Hector_Small_Arm_Control::cleanup()
{
    subscriber_spinner_->stop();
}

void Hector_Small_Arm_Control::read(ros::Time time, ros::Duration period)
{
    if(received_joint_states_.size()<5){
        return;
    }
    for(unsigned int i=0; i<joint_name_vector_.size()-1; i++)
    {
        joint_positions_[joint_name_vector_[i]] = received_joint_states_[joint_name_vector_[i]]->current_pos - joint_offset[joint_name_vector_[i]];
    }

    //take last dof as fake
    joint_positions_[joint_name_vector_[joint_name_vector_.size()-1]] = _fake_dof_value;
}

void Hector_Small_Arm_Control::write(ros::Time time, ros::Duration period)
{
    for(unsigned int i=0; i<joint_name_vector_.size(); i++)
      {
        std_msgs::Float64 msg;
        msg.data = joint_pos_cmds_[joint_name_vector_[i]] + joint_offset[joint_name_vector_[i]];
        joint_cmd_pubs_[joint_name_vector_[i]].publish(msg);
    }

   //remember last joint value for fake 6. dof
   _fake_dof_value = joint_pos_cmds_[joint_name_vector_[joint_name_vector_.size()-1]];
}

void Hector_Small_Arm_Control::jointStateCallback(const dynamixel_msgs::JointStateConstPtr& dyn_joint_state)
{
   received_joint_states_[dyn_joint_state->name] = dyn_joint_state;
}

}

int main(int argc, char** argv){

try{
        ROS_INFO("starting");
        ros::init(argc, argv, "hector_small_arm_control");

        hector_small_arm_control::Hector_Small_Arm_Control hector_small_arm;

        controller_manager::ControllerManager cm(&hector_small_arm);

        ros::AsyncSpinner spinner(4);
        spinner.start();

        ros::Rate loop_rate(50);

        ros::Time last_time = ros::Time::now();

        while (ros::ok())
                {
                    //ROS_INFO("in main loop");
                    loop_rate.sleep();

                    ros::Time current_time = ros::Time::now();
                    ros::Duration elapsed_time = current_time - last_time;
                    last_time = current_time;

                    //ROS_INFO("before read");
                    hector_small_arm.read(current_time, elapsed_time);
                    //ROS_INFO("after read");

                    //ROS_INFO("before cm.update");
                    cm.update(current_time, elapsed_time);
                    //ROS_INFO("after cm.update");

                    //ROS_INFO("before write");
                    hector_small_arm.write(current_time, elapsed_time);
                    //ROS_INFO("after write");
                }

        hector_small_arm.cleanup();
    }
    catch(...)
    {
        ROS_ERROR("Unhandled exception!");
        return -1;
    }

    return 0;
}
