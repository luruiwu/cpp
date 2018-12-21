#include<imi_nav_v2/imi_nav.h>
#include<iostream>
#include<sstream>

#include <eigen3/Eigen/Dense>
#include <visualization_msgs/Marker.h>

#define N 999

//using namespace std;

CNavigation::CNavigation() :
  private_nh("~"), m_pClient(NULL), m_eState(SINIT), m_dFrontGap(0.0f), m_dBackGap(0.0f),m_dTurnGap(0.0f),
  m_dTurnAngle(0.0f), m_dSecondTurnAngle(0.0f), m_dObstacleGap(0.0f),m_dMinBlock(10.0f),m_dAdjustAngle(0.0f),m_InitPose(),
  m_bIsLeft(false),m_bBlock(false), m_bNeedTurn(false), m_bMapBlock(false),m_bRegionLeft(false),m_bNeedBack(false){
  private_nh.param<string>("odom_frame", m_strOdomFrame, "odom");
  private_nh.param<string>("base_frame", m_strBaseFrame, "base_link");
  private_nh.param<double>("front_gap", m_dFrontGap, 5.0f);
//  private_nh.param<double>("min_backGap", m_dBackGap, 0.0f);
  private_nh.param<double>("min_turnGap", m_dTurnGap, 0.38f);
//  private_nh.param<double>("max_randGap", m_dRandGap, 0.02f);


  private_nh.param<double>("min_turnAngle", m_dTurnAngle, 1.5708f);
  private_nh.param<double>("min_secondTurnAngle", m_dSecondTurnAngle, 3.1416f);
//  private_nh.param<double>("max_randAngle", m_dRandAngle, 0.1745f);
  private_nh.param<double>("obstacle_gap",m_dObstacleGap, 0.2f);
  private_nh.param<double>("adjust_angle",m_dAdjustAngle,0.05f);

  m_pSmap = new GMapping::ScanMatcherMap(GMapping::Point(0.0,0.0), -1.0, -1.0, 1.0, 1.0,
                                         0.1);
  m_mapSub = nh.subscribe<nav_msgs::OccupancyGrid>("map", 10, &CNavigation::onMap, this);
  m_pubCmdVel = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 20);
  m_laserSub = nh.subscribe<sensor_msgs::LaserScan>("scan", 100, &CNavigation::onLaser, this);

//  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

}

CNavigation::~CNavigation() {
  if (m_pClient != NULL) {
    delete m_pClient;
    m_pClient = NULL;
  }

  if(m_pSmap!=NULL){
    delete m_pSmap;
    m_pSmap=NULL;
  }
}

void CNavigation::onLaser(const sensor_msgs::LaserScanConstPtr &laser) {
  for (vector<float>::const_iterator it = laser->ranges.begin(); it != laser->ranges.end(); it++) {
    if (*it > 0.17 && *it < 0.2) {
      m_bBlock = true;
      ROS_INFO("Block!");
      break;
    }
  }
}

void CNavigation::onMap(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
  m_map = *msg.get();
  if(!m_bMapBlock){
    if (m_map.info.width != (unsigned int) (*m_pSmap).getMapSizeX() ||
        m_map.info.height != (unsigned int) (*m_pSmap).getMapSizeY()){
      double xmin_ = m_map.info.origin.position.x;
      double ymin_ = m_map.info.origin.position.y;
      double xmax_ = m_map.info.width * m_map.info.resolution + xmin_;
      double ymax_ = m_map.info.height * m_map.info.resolution + ymin_;
      GMapping::Point center;
      center.x = (xmin_ + xmax_) / 2.0;
      center.y = (ymin_ + ymax_) / 2.0;
      m_pSmap = new GMapping::ScanMatcherMap(center, xmin_, ymin_, xmax_, ymax_,
                                             0.1);
      for(int x=0;x<(unsigned int) (*m_pSmap).getMapSizeX();++x){
        vector<int> vec;
        for(int y=0;y<(unsigned int)(*m_pSmap).getMapSizeY();++y){
          vec.push_back(0);
        }
        m_vCleanMap.push_back(vec);
      }

      ROS_INFO("Map Changed!");
//      ROS_INFO("min_x=%f,max_x=%f,min_y=%f,max_y=%f",xmin_,xmax_,ymin_,ymax_);
//      GMapping::Point wp=(*m_pSmap).map2world(168,64);
//      ROS_INFO("wp:x=%f,y=%f",wp.x,wp.y);
    }

    m_vBlock.clear();
    m_dMinBlock=10.0f;
    m_NearestObstale.x=0;
    m_NearestObstale.y=0;
    SPose pose;
    if(getOdomPose(pose,transform)){
      //left,right,up-right,up-left in robot frame
      SPose p1(0,0.178,0);
      SPose p2(0,-0.178,0);
      SPose p3(0.3,-0.178,0);
      SPose p4(0.3,0.178,0);
      //transform to world frame
      tf::Vector3 wp1=transform*tf::Vector3(0,0.178,0);
      tf::Vector3 wp2=transform*tf::Vector3(0,-0.178,0);
      tf::Vector3 wp3=transform*tf::Vector3(0.3,-0.178,0);
      tf::Vector3 wp4=transform*tf::Vector3(0.3,0.178,0);
      //map to 2D grid map
      GMapping::IntPoint poseInMap1 = (*m_pSmap).world2map(GMapping::Point(wp1.getX(),wp1.getY()));
      GMapping::IntPoint poseInMap2 = (*m_pSmap).world2map(GMapping::Point(wp2.getX(),wp2.getY()));
      GMapping::IntPoint poseInMap3=(*m_pSmap).world2map(GMapping::Point(wp3.getX(),wp3.getY()));
      GMapping::IntPoint poseInMap4=(*m_pSmap).world2map(GMapping::Point(wp4.getX(),wp4.getY()));

      int frontXmin,frontXmax,frontYmin,frontYmax;
      if(CUtil::getMinMax(poseInMap1.x,poseInMap2.x,poseInMap3.x,poseInMap4.x,frontXmin,frontXmax)&&
         CUtil::getMinMax(poseInMap1.y,poseInMap2.y,poseInMap3.y,poseInMap4.y,frontYmin,frontYmax)){
        for(int x=frontXmin;x<frontXmax;x++){
          for(int y=frontYmin;y<frontYmax;y++){
            if( m_map.data[MAP_IDX(m_map.info.width,x,y)]>0){
              GMapping::Point wPose = (*m_pSmap).map2world(x,y);
              tf::Vector3 vp=transform.inverse()*tf::Vector3(wPose.x,wPose.y,0);
              SPose rPose(vp.getX(),vp.getY(),0);
              if(CUtil::isPointInRect(p1,p2,p3,p4,rPose)){
                m_vBlock.push_back(rPose);
                double d=rPose.x-sqrt(0.178*0.178-pow(rPose.y,2));
                if(d<m_dMinBlock){
                  m_dMinBlock=d;
                  m_NearestObstale=rPose;
                }
              }
            }
          }
        }
      }
      if(m_vBlock.size()>0){
        ROS_INFO("map block!");
        m_bMapBlock=true;
      }
    }
  }
}

bool CNavigation::getOdomPose(SPose &pose,tf::StampedTransform &transform) {
  try {
    m_listener.lookupTransform(m_strOdomFrame, m_strBaseFrame,
                               ros::Time(0), transform);
  }
  catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }
  pose.x = transform.getOrigin().x();
  pose.y = transform.getOrigin().y();
  pose.w = tf::getYaw(transform.getRotation());
  return true;
}

void CNavigation::handleInit() {
  m_eState = SGOSTRAIGHT;
}

void CNavigation::handleGoStraight() {
  if (m_pClient == NULL) {
    ROS_ERROR("client null, should not happen!");
    return;
  }
  m_LastPose = m_currentPose;

  if(fabs(m_currentPose.w)<PI/2){
    m_bIsLeft=true;
  }else{
    m_bIsLeft=false;
  }

  double theta=PI;
  if(m_bIsLeft){
    theta=0;
  }
  theta-=0.05;
  m_GoalPose.x = m_currentPose.x + cos(theta) * m_dFrontGap;
  m_GoalPose.y = m_currentPose.y + sin(theta) * m_dFrontGap;
  m_GoalPose.w = theta;
  m_pClient->send(m_GoalPose);
  m_eState = SGOSTRAIGHT_DOING;

  ROS_INFO("Go Straight!");
}

void CNavigation::handleGoStraightDoing() {
  if (m_pClient == NULL) {
    ROS_ERROR("client null, should not happen!");
    return;
  }

  if (m_pClient->getState() == CMoveBase::STATE_DONE) {
    m_eState =SGOSTRAIGHT;
    return;
  }
  if (m_bBlock) {
    m_eState = SGOBACK;
    m_dBackGap=0.8;
//    m_bBlock=false;
//    m_bMapBlock=false;
    return;
  }
  if(!m_bBlock && m_bMapBlock){
    ROS_INFO("minBlock=%f",m_dMinBlock);
    if(m_dMinBlock<0.5){
      m_eState=SGOBACK;
      m_dBackGap=0.18;
    }else{
      m_eState=STURN;
    }
    return;
  }

  double dDist = pow(m_currentPose.x - m_LastPose.x, 2) + pow(m_currentPose.y - m_LastPose.y, 2);
  if (dDist > pow(m_dFrontGap / 2.0f, 2)) {
    m_LastPose = m_currentPose;
    m_GoalPose.x = m_currentPose.x + cos(m_currentPose.w) * m_dFrontGap;
    m_GoalPose.y = m_currentPose.y + sin(m_currentPose.w) * m_dFrontGap;
    m_GoalPose.w = m_currentPose.w;
    m_pClient->setGoal(m_GoalPose);
  }
}

void CNavigation::handleGoBack(){
  if (m_pClient == NULL) {
    ROS_ERROR("client null, should not happen!");
    return;
  }
  srand(time(NULL));
  m_LastPose = m_currentPose;
  m_GoalPose.x = m_currentPose.x - cos(m_currentPose.w) * m_dBackGap;
  m_GoalPose.y = m_currentPose.y - sin(m_currentPose.w) * m_dBackGap;
  m_GoalPose.w = m_currentPose.w;
  m_pClient->sendBack(m_GoalPose);
  m_eState = SGOBACK_DOING;
  ROS_INFO("Go Back");
}

void CNavigation::handleGoBackDoing(){
  if (m_pClient == NULL) {
    ROS_ERROR("client null, should not happen!");
    return;
  }

  if (m_pClient->getState() != CMoveBase::STATE_DONE) {
    return;
  }

//  if(/*m_bNeedBack*/){
//    m_bNeedBack=false;
//    m_eState=SREGIONSTOP;
//    return;
//  }

  m_eState = STURN;
}

void CNavigation::handleGoAhead(){
  if (m_pClient == NULL) {
    ROS_ERROR("client null, should not happen!");
    return;
  }

//  ROS_INFO("MinBlock=%f",m_dMinBlock);
  m_LastPose = m_currentPose;
  m_GoalPose.x = m_currentPose.x + cos(m_currentPose.w) * (m_dMinBlock-0.2);
  m_GoalPose.y = m_currentPose.y + sin(m_currentPose.w) * (m_dMinBlock-0.2);
  m_GoalPose.w = m_currentPose.w;
  m_pClient->setGoal(m_GoalPose);
  m_eState = SGOAHEAD_DOING;
  ROS_INFO("Go Ahead,MinBlock=%f",m_dMinBlock);
}

void CNavigation::handleGoAheadDoing(){
  if (m_pClient == NULL) {
    ROS_ERROR("client null, should not happen!");
    return;
  }

  if (m_pClient->getState() != CMoveBase::STATE_DONE) {
    return;
  }

  m_eState = STURN;

}

void CNavigation::handleTurn() {
  if (m_pClient == NULL) {
    ROS_ERROR("client null, should not happen!");
    return;
  }


  if(m_bBlock){
    m_bBlock=false;
    m_dTurnAngle=PI/2;
  }
  else if(m_bMapBlock)
    calTurnAngle();

  double angle=m_dTurnAngle+m_dAdjustAngle;
  m_LastPose = m_currentPose;
  m_GoalPose.x = m_currentPose.x;
  m_GoalPose.y = m_currentPose.y;
  m_GoalPose.w = m_currentPose.w + (m_bIsLeft?angle:-angle);
  m_GoalPose.w = CUtil::normalizeYaw(m_GoalPose.w);
  m_pClient->sendTurn(m_GoalPose);
  m_eState =STURN_DOING;

  ROS_INFO("Turn,CurrentAngle=%f,Angle=%f,GoalAngle=%f",m_currentPose.w,angle,m_GoalPose.w);
}

void CNavigation::handleTurnDoing() {
  if (m_pClient == NULL) {
    ROS_ERROR("client null, should not happen!");
    return;
  }
  if (m_pClient->getState() != CMoveBase::STATE_DONE) {
    return;
  }

  m_eState = SSIDEWALK;

}

void CNavigation::handleSideWalk() {
  if (m_pClient == NULL) {
    ROS_ERROR("client null, should not happen!");
    return;
  }
  srand(time(NULL));
  double gap=0.6/sin(m_dTurnAngle);

  SPose pose;
  if(getOdomPose(pose,transform)){
    SPose p1(0.178,0.178,0);
    SPose p2(0.178,-0.178,0);
    SPose p3(0.178+gap,-0.178,0);
    SPose p4(0.178+gap,0.178,0);
    //transform to world frame
    tf::Vector3 wp1=transform*tf::Vector3(0.178,0.178,0);
    tf::Vector3 wp2=transform*tf::Vector3(0.178,-0.178,0);
    tf::Vector3 wp3=transform*tf::Vector3(0.178+gap,-0.178,0);
    tf::Vector3 wp4=transform*tf::Vector3(0.178+gap,0.178,0);
    //map to 2D grid map
    GMapping::IntPoint poseInMap1 = (*m_pSmap).world2map(GMapping::Point(wp1.getX(),wp1.getY()));
    GMapping::IntPoint poseInMap2 = (*m_pSmap).world2map(GMapping::Point(wp2.getX(),wp2.getY()));
    GMapping::IntPoint poseInMap3=(*m_pSmap).world2map(GMapping::Point(wp3.getX(),wp3.getY()));
    GMapping::IntPoint poseInMap4=(*m_pSmap).world2map(GMapping::Point(wp4.getX(),wp4.getY()));
    double minBlock=10.0;
    double minClean=10.0;
    int frontXmin,frontXmax,frontYmin,frontYmax;
    if(CUtil::getMinMax(poseInMap1.x,poseInMap2.x,poseInMap3.x,poseInMap4.x,frontXmin,frontXmax)&&
       CUtil::getMinMax(poseInMap1.y,poseInMap2.y,poseInMap3.y,poseInMap4.y,frontYmin,frontYmax)){
      for(int x=frontXmin;x<frontXmax;x++){
        for(int y=frontYmin;y<frontYmax;y++){
          //block or not
          if(m_map.data[MAP_IDX(m_map.info.width,x,y)]>0||m_vCleanMap[x][y]==1){
            GMapping::Point wPose = (*m_pSmap).map2world(x,y);
            tf::Vector3 vp=transform.inverse()*tf::Vector3(wPose.x,wPose.y,0);
            SPose rPose(vp.getX(),vp.getY(),0);
            if(CUtil::isPointInRect(p1,p2,p3,p4,rPose)){
              double d=rPose.x-sqrt(0.178*0.178-pow(rPose.y,2));
              if(m_map.data[MAP_IDX(m_map.info.width,x,y)]>0){
                if(d<minBlock){
                  minBlock=d;
                }
              }
              if(m_vCleanMap[x][y]==1){
                if(d<minClean){
                  minClean=d;
                }
              }
            }
          }
        }
      }

      ROS_INFO("minBlock=%f,minClean=%f",minBlock,minClean);
      if(fabs(10.0-minBlock)>0.0001){
        if(minBlock<0.3){
          m_eState=SSTOP;
          return;
        }
      }else if(fabs(10.0-minClean)>0.0001){
        if(minClean<0.5){
          m_eState=SSTOP;
          return;
        }
      }
    }
  }

  gap=0.18/*/sin(m_dTurnAngle)*/;
  m_LastPose = m_currentPose;
  m_GoalPose.x = m_currentPose.x + cos(m_currentPose.w) * gap;
  m_GoalPose.y = m_currentPose.y + sin(m_currentPose.w) * gap;
  m_GoalPose.w = m_currentPose.w;
  m_pClient->send(m_GoalPose);
  m_eState = SSIDEWALK_DOING;
  ROS_INFO("SideWalk,CurrentAngle=%f,gap=%f",m_currentPose.w, gap);
}

void CNavigation::handleSideWalkDoing() {
  if (m_pClient == NULL) {
    ROS_ERROR("client null, should not happen!");
    return;
  }

  if (m_pClient->getState() != CMoveBase::STATE_DONE) {
    return;
  }

  if(m_bBlock){
//    m_bNeedBack=true;
    m_eState=SSTOP;
    return;
  }

  m_eState = STURN_SECOND;

}

void CNavigation::handleTurnSecond() {
  if (m_pClient == NULL) {
    ROS_ERROR("client null, should not happen!");
    return;
  }

  m_LastPose = m_currentPose;
  m_GoalPose.x = m_currentPose.x;
  m_GoalPose.y = m_currentPose.y;
//  double angle=PI+m_dAdjustAngle;
  m_GoalPose.w = m_bIsLeft?(PI-0.1):(0.1);
  m_GoalPose.w = CUtil::normalizeYaw(m_GoalPose.w);
  m_pClient->sendTurn(m_GoalPose);

  m_eState =STRUN_SECOND_DOING;
//  ROS_INFO("Turn Second,TurnAngle=%f,isLeft=%d,currentPose=%f,goalPoseAngle=%f",angle,m_bIsLeft,m_currentPose.w,m_GoalPose.w);
}

void CNavigation::handleTurnSecondDoing() {
  if (m_pClient == NULL) {
    ROS_ERROR("client null, should not happen!");
    return;
  }

  if (m_pClient->getState() != CMoveBase::STATE_DONE) {
    return;
  }

//  ROS_INFO("CurrentAngle=%f",m_currentPose.w);

  m_bMapBlock=false;

  m_eState = SGOSTRAIGHT;

}

void CNavigation::handleRegionStop() {
  if(isRegionLeft())
    m_eState=SGONEXTREGION;
  else
    m_eState=SSTOP;
  ROS_INFO("Region Stop");
}

void CNavigation::handleGoNextRegion() {
  if (m_pClient == NULL) {
    ROS_ERROR("client null, should not happen!");
    return;
  }

  ROS_INFO("Go next region");
  //path-planning to the next region.A* algorithm.
  m_LastPose=m_currentPose;

  double minDistance=100.0;
  for(int i=0;i<m_vNonCleanStartPoint.size();++i){
    double d=pow(m_currentPose.x-m_vNonCleanStartPoint[i].x,2)+pow(m_currentPose.y-m_vNonCleanStartPoint[i].y,2);
    if(d<minDistance){
      minDistance=d;
      m_NextRegionPose=m_vNonCleanStartPoint[i];
    }
  }

  ROS_INFO("nextPose:x=%f,y=%f",m_NextRegionPose.x,m_NextRegionPose.y);

  MoveBaseClient ac("move_base",true);
  if(!ac.waitForServer(ros::Duration(60))) {
    ROS_INFO("Can't connected to move base server");
    return;
  }
  ROS_INFO("Connected to move base server");
  ROS_INFO("Starting navigation test");
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x=m_NextRegionPose.x;
  goal.target_pose.pose.position.y=m_NextRegionPose.y;
  goal.target_pose.pose.position.z=0;
  goal.target_pose.pose.orientation.x=0;
  goal.target_pose.pose.orientation.y=0;
  goal.target_pose.pose.orientation.z=0;
  goal.target_pose.pose.orientation.w=1;
  ac.sendGoal(goal);
  ROS_INFO("sending move_base goal");
  bool finished_within_time = ac.waitForResult();
  if(!finished_within_time) {
    ac.cancelGoal();
    ROS_INFO("Timed out achieving goal");
  } else {
    //We made it!
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      m_eState=SGOSTRAIGHT;
      ROS_INFO("Goal succeeded!");
    } else {
      m_eState=SSTOP;
      ROS_INFO("The base failed for some reason");
    }
  }
}

void CNavigation::handleGoNextRegionDoing() {
//  ac.waitForResult();
//  if(ac.getState()== actionlib::SimpleClientGoalState::SUCCEEDED){
//    ROS_INFO("move_base goal successful");
//    m_eState=SGOSTRAIGHT;
//  }else{
//    ROS_ERROR("move_base goal failed");
//    m_eState=SSTOP;
//  }
}

void CNavigation::handleStop() {
  m_pClient->setState(CMoveBase::STATE_DONE);
  ROS_INFO("Stop");
}

void CNavigation::StartService() {
  ROS_INFO("init");
  if (m_pClient == NULL) {
    m_pClient = new CMoveBase(m_pubCmdVel);
  }

  while(1){
    if(getOdomPose(m_InitPose,transform)){
      break;
    }
  }

  ros::Rate loopRate(20);
  while (ros::ok()) {
    ros::spinOnce();

//    double a=CUtil::normalizeYaw(PI/2+PI/3);//left
//    Eigen::Vector3d v1(1,0,0);
//    Eigen::AngleAxisd n1(a, Eigen::Vector3d(0, 0, 1));
//    Eigen::Vector3d rotated_v1 = n1*v1;
//    cout << "(1, 0, 0)旋转PI/2后:" << endl << rotated_v1.transpose() << endl;
//    Eigen::Quaterniond q(n1);

//    visualization_msgs::Marker marker;
//    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
//    marker.header.frame_id = "/map";
//    marker.header.stamp = ros::Time::now();
//    // Set the namespace and id for this marker.  This serves to create a unique ID
//    // Any marker sent with the same namespace and id will overwrite the old one
//    marker.ns = "basic_shapes";
//    marker.id = 0;
//    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
//    marker.type = visualization_msgs::Marker::ARROW;
//    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
//    marker.action = visualization_msgs::Marker::ADD;
//    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
//    marker.pose.position.x = 0;
//    marker.pose.position.y = 0;
//    marker.pose.position.z = 0;
//    marker.pose.orientation.x = q.x();
//    marker.pose.orientation.y = q.y();
//    marker.pose.orientation.z = q.z();
//    marker.pose.orientation.w = q.w();
//    // Set the scale of the marker -- 1x1x1 here means 1m on a side
//    marker.scale.x = 1.0;
//    marker.scale.y = 1.0;
//    marker.scale.z = 1.0;
//    // Set the color -- be sure to set alpha to something non-zero!
//    marker.color.r = 0.0f;
//    marker.color.g = 1.0f;
//    marker.color.b = 0.0f;
//    marker.color.a = 1.0;
//    marker.lifetime = ros::Duration();
//    marker_pub.publish(marker);
////    ROS_INFO("publish arrow success!x=%f,y=%f,z=%f,w=%f",q.x(),q.y(),q.z(),q.w());

    if (!getOdomPose(m_currentPose,transform)) {
      ROS_ERROR("get pose failed");
      loopRate.sleep();
      continue;
    }

//    updateCleanMap();
    m_pClient->spinOnce(m_currentPose);
    switch (m_eState) {
      case SINIT: {
        handleInit();
        break;
      }
      case SGOSTRAIGHT: {
        handleGoStraight();
        break;
      }
      case SGOSTRAIGHT_DOING: {
        handleGoStraightDoing();
        break;
      }
      case SGOBACK:{
        handleGoBack();
        break;
      }
      case SGOBACK_DOING:{
        handleGoBackDoing();
        break;
      }
      case SGOAHEAD:{
        handleGoAhead();
        break;
      }
      case SGOAHEAD_DOING:{
        handleGoAheadDoing();
        break;
      }
      case STURN: {
        handleTurn();
        break;
      }
      case STURN_DOING: {
        handleTurnDoing();
        break;
      }
      case SSIDEWALK: {
        handleSideWalk();
        break;
      }
      case SSIDEWALK_DOING: {
        handleSideWalkDoing();
        break;
      }
      case STURN_SECOND: {
        handleTurnSecond();
        break;
      }
      case STRUN_SECOND_DOING: {
        handleTurnSecondDoing();
        break;
      }
      case SREGIONSTOP: {
        handleRegionStop();
        break;
      }
      case SGONEXTREGION:{
        handleGoNextRegion();
        break;
      }
      case SGONEXTREGION_DOING:{
        handleGoNextRegionDoing();
        break;
      }
      case SSTOP: {
        handleStop();
        break;
      }
      default: {
        ROS_ERROR("do nothing, should not happen!");
        break;
      }
    }
    loopRate.sleep();
  }
}

void CNavigation::calTurnAngle(){
  //calculate the obstacles in the left of the NearestObstacle
//  ROS_INFO("nearest obstacle:x=%f,y=%f",m_NearestObstale.x,m_NearestObstale.y);
  vector<SPose> vObstacle;
  for(vector<SPose>::iterator it=m_vBlock.begin();it!=m_vBlock.end();++it){
    SPose p=*it;
    if((m_bIsLeft && p.y>m_NearestObstale.y)||(!m_bIsLeft && p.y<m_NearestObstale.y)){
      vObstacle.push_back(p);
    }
  }
  SPose ppair=m_NearestObstale;
  for(vector<SPose>::iterator it=vObstacle.begin();it!=vObstacle.end();++it){
    SPose p=*it;
    double slope=fabs((p.x-m_NearestObstale.x)/(p.y-m_NearestObstale.y));
    double offset=p.x-slope*fabs(p.y);//>0
    vector<SPose>::iterator iter=vObstacle.begin();
    for(;iter!=vObstacle.end();++iter){
      if(iter!=it){
        SPose p1=*iter;
        double outValue=slope*fabs(p1.y)+offset;
        if(p1.x<outValue){
          break;
        }
      }
    }
    if(iter==vObstacle.end()){
      ppair=p;
      break;
    }
  }

  //calculate the turn angle
  double a=m_NearestObstale.y-ppair.y;
  double b=ppair.x-m_NearestObstale.x;
  if(fabs(b)<0.0001){
    m_dTurnAngle=PI/2;
  }else {
    double t=a/b;
    if(t>0.0001){
      if(m_bIsLeft)
        m_dTurnAngle=PI-atan(t);
      else
        m_dTurnAngle=atan(t);
    }else if(t<-0.0001){
      if(m_bIsLeft)
        m_dTurnAngle=atan(-t);
      else
        m_dTurnAngle=PI-atan(-t);
    }
  }
}

bool CNavigation::transformWorld2Robot(const GMapping::Point &wPose, SPose &rPose){
  geometry_msgs::PointStamped wtfPose;
  wtfPose.header.stamp=ros::Time::now();
  wtfPose.header.frame_id=m_strOdomFrame;
  wtfPose.point.x=wPose.x;
  wtfPose.point.y=wPose.y;
  wtfPose.point.z=0;
  geometry_msgs::PointStamped rtfPose;
  try{
    m_listener.transformPoint(m_strBaseFrame,wtfPose,rtfPose);
  }catch(tf::TransformException &ex){
    ROS_ERROR("Failed to compute pose in base_link,%s",ex.what());
    return false;
  }
  rPose.x=rtfPose.point.x;
  rPose.y=rtfPose.point.y;
  return true;
}

void CNavigation::updateCleanMap(){
  //calculate the robot_base scope in world-frame
  tf::Vector3 wp1=transform*tf::Vector3(-0.178,0.178,0);
  tf::Vector3 wp2=transform*tf::Vector3(-0.178,-0.178,0);
  tf::Vector3 wp3=transform*tf::Vector3(0.178,-0.178,0);
  tf::Vector3 wp4=transform*tf::Vector3(0.178,0.178,0);
  //map to 2D grid map
  GMapping::IntPoint poseInMap1 = (*m_pSmap).world2map(GMapping::Point(wp1.getX(),wp1.getY()));
  GMapping::IntPoint poseInMap2 = (*m_pSmap).world2map(GMapping::Point(wp2.getX(),wp2.getY()));
  GMapping::IntPoint poseInMap3 = (*m_pSmap).world2map(GMapping::Point(wp3.getX(),wp3.getY()));
  GMapping::IntPoint poseInMap4 = (*m_pSmap).world2map(GMapping::Point(wp4.getX(),wp4.getY()));
  int frontXmin,frontXmax,frontYmin,frontYmax;
  if(CUtil::getMinMax(poseInMap1.x,poseInMap2.x,poseInMap3.x,poseInMap4.x,frontXmin,frontXmax)&&
     CUtil::getMinMax(poseInMap1.y,poseInMap2.y,poseInMap3.y,poseInMap4.y,frontYmin,frontYmax)){
    for(int x=frontXmin;x<frontXmax;++x){
      for(int y=frontYmin;y<frontYmax;++y){
        if(m_vCleanMap[x][y]!=1){
          m_vCleanMap[x][y]=1;
        }
      }
    }
  }
}

bool CNavigation::isRegionLeft(){
  vector<SPose>().swap(m_vNonCleanStartPoint);
  bool cleanFlag=false;
  for(int x=0;x<m_vCleanMap.size()-10;++x){
    for(int y=0;y<m_vCleanMap[0].size()-10;++y){
      for(int i=x;i<x+10;++i){
        for(int j=y;j<y+10;++j){
          if(m_vCleanMap[i][j]==1||m_map.data[MAP_IDX(m_map.info.width,i,j)]>0){
            break;
            cleanFlag=true;
          }
        }
        if(cleanFlag)
          break;
      }
      if(!cleanFlag){
        GMapping::Point wPose=(*m_pSmap).map2world(x,y);
        tf::Vector3 vp=transform.inverse()*tf::Vector3(wPose.x,wPose.y,0);
        SPose rPose(vp.getX(),vp.getY(),0);
        m_vNonCleanStartPoint.push_back(rPose);
      }
      cleanFlag=false;
    }
  }
  if(!m_vNonCleanStartPoint.empty())
    return true;
  return false;
}
