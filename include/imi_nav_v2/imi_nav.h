#ifndef __nav_H__V2__
#define __nav_H__V2__

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <boost/thread/thread.hpp>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map>
#include <iostream>
#include "MoveBaseClient.h"
#include "MoveBase.h"
#include "astar.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "gmapping/gridfastslam/gridslamprocessor.h"
#include "gmapping/sensor/sensor_base/sensor.h"
#include "gmapping/sensor/sensor_range/rangesensor.h"
#include "gmapping/sensor/sensor_odometry/odometrysensor.h"

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class CNavigation {
    enum STATE{
        SINIT = 0,
        SGOSTRAIGHT,
        SGOSTRAIGHT_DOING,
        SGOBACK,
        SGOBACK_DOING,
        SGOAHEAD,
        SGOAHEAD_DOING,
        STURN,
        STURN_DOING,
        SSIDEWALK,
        SSIDEWALK_DOING,
        STURN_SECOND,
        STRUN_SECOND_DOING,
        SREGIONSTOP,
        SGONEXTREGION,
        SGONEXTREGION_DOING,
        SSTOP
    };
public:
    CNavigation();

    ~CNavigation();

    void StartService();

private:
    void onMap(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void onLaser(const sensor_msgs::LaserScanConstPtr &laser);

    bool getOdomPose(SPose &pose,tf::StampedTransform &transform);

    void handleInit();
    void handleGoStraight();
    void handleGoStraightDoing();
    void handleGoBack();
    void handleGoBackDoing();
    void handleGoAhead();
    void handleGoAheadDoing();
    void handleTurn();//First turn, calculate the turn angle according to the clean direction
    void handleTurnDoing();
    void handleSideWalk();
    void handleSideWalkDoing();
    void handleTurnSecond();//Second turn,calculate the turn angle according to the first turn angle
    void handleTurnSecondDoing();
    void handleRegionStop();
    void handleGoNextRegion();
    void handleGoNextRegionDoing();
    void handleStop();

    void calTurnAngle();
    bool transformWorld2Robot(const GMapping::Point &wPose,SPose &rPose);
    void updateCleanMap();
    bool isRegionLeft();


private:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    tf::TransformListener m_listener;
    ros::Subscriber m_mapSub;
    ros::Publisher m_pubCmdVel;
    ros::Subscriber m_laserSub;
    ros::Publisher marker_pub;
    string m_strOdomFrame;
    string m_strBaseFrame;
    nav_msgs::OccupancyGrid m_map;
    CMoveBase* m_pClient;
    STATE m_eState;
    SPose m_LastPose;
    SPose m_GoalPose;
    SPose m_currentPose;
    double m_dFrontGap;
    double m_dBackGap;
    double m_dTurnGap;
    double m_dTurnAngle;
    double m_dSecondTurnAngle;
    double m_dObstacleGap;
    bool m_bIsLeft;
    bool m_bBlock;
    bool m_bNeedTurn;
    bool m_bNeedBack;

    GMapping::ScanMatcherMap *m_pSmap;
    bool m_bMapBlock;
    bool m_bRegionLeft;

    SPose m_InitPose;
    tf::StampedTransform transform;
    double m_dMinBlock;
    double m_dAdjustAngle;
    vector<SPose> m_vBlock;
    SPose m_NearestObstale;
    vector<vector<int>> m_vCleanMap;
    vector<SPose> m_vNonCleanStartPoint;
    SPose m_NextRegionPose;
};


#endif //__nav_H__V2__
