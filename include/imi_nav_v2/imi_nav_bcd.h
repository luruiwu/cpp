#ifndef IMI_NAV_BCD_H
#define IMI_NAV_BCD_H

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
#include <iostream>
#include "MoveBaseClient.h"
#include "MoveBase.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <imi_nav_v2/astar.h>
#include <imi_nav_v2/tsp.h>

#include "gmapping/gridfastslam/gridslamprocessor.h"
#include "gmapping/sensor/sensor_base/sensor.h"
#include "gmapping/sensor/sensor_range/rangesensor.h"
#include "gmapping/sensor/sensor_odometry/odometrysensor.h"

#include <room_exploration/cell_decompositon.h>
#include <building_navigation/a_star_pathplanner.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class CNav {
    enum STATE{
        SINIT = 0,
        SGOSTRAIGHT,
        SGOSTRAIGHT_DOING,
        SGOBACK,
        SGOBACK_DOING,
        STURN,
        STURN_DOING,
        SSIDEWALK,
        SSIDEWALK_DOING,
        STURN_SECOND,
        STRUN_SECOND_DOING,
        SCELLSTOP,
        SGONEXTCELL,
        SCELLTURN,
        SCELLTURNDOING,
        SSTOP
    };

    enum TURNDIRECTION{
      DLEFT = 0,
      DRIGHT
    };

public:
    CNav(const vector<CellPolygon> & cells);

    ~CNav();

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
    void handleTurn();//First turn, calculate the turn angle according to the clean direction
    void handleTurnDoing();
    void handleSideWalk();
    void handleSideWalkDoing();
    void handleTurnSecond();//Second turn
    void handleTurnSecondDoing();
    void handleCellStop();
    void handleGoNextCell();
    void handleCellTurn();
    void handleCellTurnDoing();
    void handleStop();
    void calTurnAngle();
    bool transformWorld2Robot(const GMapping::Point &wPose,SPose &rPose);
    bool getNextCell(const SPose &rPose);

private:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    vector<CellPolygon> cells;
    GMapping::ScanMatcherMap *m_pSmap;
    tf::TransformListener m_listener;
    ros::Subscriber m_mapSub;
    ros::Publisher m_pubCmdVel;
    ros::Subscriber m_laserSub;
    string m_strOdomFrame;
    string m_strBaseFrame;
    nav_msgs::OccupancyGrid m_map;
    vector<SPose> m_vBlock;//these block poses are in robot_frame,not map_frame
    CMoveBase* m_pClient;
    STATE m_eState;
    SPose m_InitPose;
    SPose m_LastPose;
    SPose m_GoalPose;
    SPose m_CurrentPose;
    SPose m_NearestObstale;
    SPose m_NextCellInitPose;
    tf::StampedTransform transform;
    int m_iNextCellInx;
    int m_iNextCellCornerInx;
    int m_iGoStraightCount;
    double m_dFrontGap;
    double m_dBackGap;
    double m_dTurnGap;
    double m_dTurnAngle;
    double m_dMinBlock;
    double m_dAdjustAngle;
    double m_dNextCellInitAngle;
    bool m_bIsLeft;
    bool m_bBlock;
    bool m_bMapBlock;
    bool m_bNextCellTurnLeft;
    bool m_bSideBack;
};


#endif // IMI_NAV_BCD_H
