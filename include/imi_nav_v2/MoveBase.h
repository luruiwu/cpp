//
// Created by robot on 18-9-4.
//

#ifndef IMI_NAV_MOVEBASE_H
#define IMI_NAV_MOVEBASE_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "Common.h"

class CMoveBase {
public:
    enum STATE {
        STATE_NONE = -1,
        STATE_INIT = 0,
        STATE_GO,
        STATE_DONE,
        STATE_BACK_INIT,
        STATE_BACK_GO,
        STATE_BACK_DONE,
        STATE_TURN_INIT,
        STATE_TURN_GO,
        STATE_TURN_DONE,
    };

    CMoveBase(ros::Publisher &pub) : m_pubCmdVel(pub), m_eState(STATE_NONE), m_dTurnMin(0.05), m_dTurnSpeed(0.6),
                                     m_dGoMin(0.2), m_dGoSpeed(0.2), m_dDevThreash(0.0175){
    }

    void send(const SPose &pose) {
        m_GoalPose = pose;
        m_eState = STATE_INIT;
    }

    void setGoal(const SPose &pose) {
        m_GoalPose = pose;
    }

    void sendBack(const SPose &pose) {
        m_GoalPose = pose;
        m_eState = STATE_BACK_INIT;
    }

    void sendTurn(const SPose &pose) {
        m_GoalPose = pose;
        m_eState = STATE_TURN_INIT;
    }

    void cancel() {
        m_eState = STATE_DONE;
    }

    STATE getState() {
        return m_eState;
    }

    void setState(STATE state){
      this->m_eState=state;
    }

    void spinOnce(const SPose &currentPose) {
        m_currentPose = currentPose;

        switch (m_eState) {
            case STATE_NONE: {
                handleNone();
                break;
            }
            case STATE_INIT: {
                handleInit();
                break;
            }
            case STATE_GO: {
                handleGo();
                break;
            }
            case STATE_DONE: {
                handleDone();
                break;
            }
            case STATE_BACK_INIT: {
                handleBackInit();
                break;
            }
            case STATE_BACK_GO: {
                handleBackGo();
                break;
            }
            case STATE_BACK_DONE: {
                handleBackDone();
                break;
            }
            case STATE_TURN_INIT: {
                handleTurnInit();
                break;
            }
            case STATE_TURN_GO: {
                handleTurnGo();
                break;
            }
            case STATE_TURN_DONE: {
                handleTurnDone();
                break;
            }
            default: {
                ROS_ERROR("should not happen");
                break;
            }
        }
    }

private:
    void handleNone() {
        move(0, 0);
    }

    void handleInit() {
        m_eState = STATE_GO;
    }

    void handleGo() {
      double dTurnYaw = atan2(m_GoalPose.y - m_currentPose.y, m_GoalPose.x - m_currentPose.x);
      dTurnYaw = dTurnYaw - m_GoalPose.w;
      dTurnYaw = CUtil::normalizeYaw(dTurnYaw);
      if (fabs(dTurnYaw) > PI / 2.0) {
        m_eState = STATE_DONE;
        return;
      }
      double dDevYaw=m_currentPose.w-m_GoalPose.w;
      dDevYaw = CUtil::normalizeYaw(dDevYaw);
      double A=tan(m_GoalPose.w);
      double B=-1.0f;
      double C=m_GoalPose.y-A*m_GoalPose.x;
      double d=fabs(A*m_currentPose.x+B*m_currentPose.y+C)/sqrt(A*A+B*B);
      if(dDevYaw>0.0175){
        move(m_dGoSpeed,-0.2);
      }else if(dDevYaw<-0.0175){
        move(m_dGoSpeed,0.2);
      }else{
        move(m_dGoSpeed,0);
      }
//      double A=tan(m_GoalPose.w);
//      double B=-1.0f;
//      double C=m_GoalPose.y-A*m_GoalPose.x;
//      double d=fabs(A*m_currentPose.x+B*m_currentPose.y+C)/sqrt(A*A+B*B);
//      if(d>0.1)
//      {
//        ROS_INFO("d=%f",d);
//        double xcg=m_GoalPose.x-m_currentPose.x;
//        double ycg=m_GoalPose.y-m_currentPose.y;
//        double costh=(xcg*sin(m_GoalPose.w)+ycg*cos(m_GoalPose.w))/
//            (sqrt(xcg*xcg+ycg*ycg)*sqrt(sin(m_GoalPose.w)*sin(m_GoalPose.w)+cos(m_GoalPose.w)*cos(m_GoalPose.w)));
//        ROS_INFO("costh=%f",costh);
//        if(costh>0.00001)
//          move(m_dGoSpeed,0.2f);
//        if(costh<-0.00001)
//          move(m_dGoSpeed,-0.2f);
//      }else{
//        move(m_dGoSpeed,0);
//      }
    }

    void handleDone() {
        move(0, 0);
    }

    void handleBackInit() {
        m_eState = STATE_BACK_GO;
    }

    void handleBackGo() {
        double dTurnYaw = atan2(m_GoalPose.y - m_currentPose.y, m_GoalPose.x - m_currentPose.x);
        dTurnYaw = dTurnYaw - m_GoalPose.w;
        dTurnYaw = CUtil::normalizeYaw(dTurnYaw);
        if (fabs(dTurnYaw) < PI / 2.0) {
            m_eState = STATE_BACK_DONE;
            return;
        }
        move(-m_dGoSpeed / 2.0f, 0);
    }

    void handleBackDone() {
        m_eState = STATE_DONE;
    }

    void handleTurnInit() {
        m_eState = STATE_TURN_GO;
    }

    void handleTurnGo() {
        double dDeltaYaw = m_GoalPose.w - m_currentPose.w;
        dDeltaYaw = CUtil::normalizeYaw(dDeltaYaw);
        if (dDeltaYaw > m_dTurnMin) {
            move(0, m_dTurnSpeed);
        } else if (dDeltaYaw < -m_dTurnMin) {
            move(0, -m_dTurnSpeed);
        } else {

            m_eState = STATE_TURN_DONE;
        }
    }

    void handleTurnDone() {
        m_eState = STATE_DONE;
    }

    void move(double x, double w) {
        geometry_msgs::Twist base_cmd;
        base_cmd.linear.x = x;
        base_cmd.linear.y = 0;
        base_cmd.linear.z = 0;
        base_cmd.angular.x = 0;
        base_cmd.angular.y = 0;
        base_cmd.angular.z = w;
        m_pubCmdVel.publish(base_cmd);
    }

private:
    ros::Publisher &m_pubCmdVel;
    STATE m_eState;
    SPose m_currentPose;
    SPose m_GoalPose;
    double m_dTurnMin;
    double m_dTurnSpeed;
    double m_dGoMin;
    double m_dGoSpeed;
    double m_dDevThreash;
};

#endif //IMI_NAV_MOVEBASE_H
