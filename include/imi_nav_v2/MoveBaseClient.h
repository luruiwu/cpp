//
// Created by robot on 18-9-3.
//

#ifndef IMI_NAV_MOVEBASECLIENT_H
#define IMI_NAV_MOVEBASECLIENT_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <tf/transform_listener.h>
#include "Common.h"

class CMoveBaseClient {
public:
    CMoveBaseClient(const std::string& name) : m_ac(name, true) {
        ROS_INFO("Waiting for action server to start.");
        m_ac.waitForServer();
        ROS_INFO("Action server started, sending goal.");
    }

    void send(const SPose& pose) {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x =  pose.x;
        goal.target_pose.pose.position.y =  pose.y;
        goal.target_pose.pose.position.z =  0;

        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose.w);
        m_ac.sendGoal(goal);
    }

    actionlib::SimpleClientGoalState sendAndWait(const SPose& pose) {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x =  pose.x;
        goal.target_pose.pose.position.y =  pose.y;
        goal.target_pose.pose.position.z =  0;

        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose.w);
        return m_ac.sendGoalAndWait(goal);
    }

    actionlib::SimpleClientGoalState waitForDone() {
        m_ac.waitForResult();
        return m_ac.getState();
    }

    void cancel(){
        m_ac.cancelAllGoals();
    }

    actionlib::SimpleClientGoalState getState(){
        return m_ac.getState();
    }
private:
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> m_ac;
};

#endif //IMI_NAV_MOVEBASECLIENT_H
