#include <iostream>
#include <sstream>

#include <eigen3/Eigen/Dense>

#include <imi_nav_v2/imi_nav_bcd.h>

#define N 999

//using namespace std;

CNav::CNav(const std::vector<CellPolygon>& cells) :
  private_nh("~"), m_pClient(NULL), m_eState(SINIT), m_dFrontGap(0.0f),m_dBackGap(0.0f),m_dTurnGap(0.0f),m_dAdjustAngle(0.0f),
  m_dTurnAngle(0.0f),m_dMinBlock(0.0f),m_iNextCellInx(-1),m_iNextCellCornerInx(-1),m_iGoStraightCount(0),m_bIsLeft(true),
  m_bMapBlock(false),m_bBlock(false), m_bNextCellTurnLeft(true),m_bSideBack(false)
{
  private_nh.param<string>("odom_frame", m_strOdomFrame, "odom");
  private_nh.param<string>("base_frame", m_strBaseFrame, "base_link");
  private_nh.param<double>("front_gap", m_dFrontGap, 20.0f);
  private_nh.param<double>("back_gap",m_dBackGap,0.35f);
  private_nh.param<double>("turn_gap", m_dTurnGap, 0.35f);
  private_nh.param<double>("adjust_angle",m_dAdjustAngle,0.05f);

  this->cells=cells;

  m_pSmap = new GMapping::ScanMatcherMap(GMapping::Point(0.0,0.0), -1.0, -1.0, 1.0, 1.0,
                                         0.1);
  m_mapSub = nh.subscribe<nav_msgs::OccupancyGrid>("map", 10, &CNav::onMap, this);
  m_pubCmdVel = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 20);
  m_laserSub = nh.subscribe<sensor_msgs::LaserScan>("scan", 100, &CNav::onLaser, this);
}

CNav::~CNav()
{
  if (m_pClient != NULL)
  {
    delete m_pClient;
    m_pClient = NULL;
  }

  if(m_pSmap!=NULL)
  {
    delete m_pSmap;
    m_pSmap=NULL;
  }
}

void CNav::onLaser(const sensor_msgs::LaserScanConstPtr &laser)
{
  for (vector<float>::const_iterator it = laser->ranges.begin(); it != laser->ranges.end(); it++)
  {
    if (*it > 0.17 && *it < 0.2)
    {
      m_bBlock = true;
      ROS_INFO("Block!");
      break;
    }
  }
}

void CNav::onMap(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  m_map = *msg.get();

  if(!m_bMapBlock)
  {
    if (m_map.info.width != (unsigned int) (*m_pSmap).getMapSizeX() ||
        m_map.info.height != (unsigned int) (*m_pSmap).getMapSizeY())
    {
      double xmin_ = m_map.info.origin.position.x;
      double ymin_ = m_map.info.origin.position.y;
      double xmax_ = m_map.info.width * m_map.info.resolution + xmin_;
      double ymax_ = m_map.info.height * m_map.info.resolution + ymin_;
      //      ROS_INFO("min_x=%f,max_x=%f,min_y=%f,max_y=%f",xmin_,xmax_,ymin_,ymax_);
      GMapping::Point center;
      center.x = (xmin_ + xmax_) / 2.0;
      center.y = (ymin_ + ymax_) / 2.0;
      m_pSmap = new GMapping::ScanMatcherMap(center, xmin_, ymin_, xmax_, ymax_,
                                             0.1);
      ROS_INFO("Map Changed!");
    }

    m_vBlock.clear();
    m_dMinBlock=10.0f;
    m_NearestObstale.x=0;
    m_NearestObstale.y=0;
    SPose pose;
    if(getOdomPose(pose,transform))
    {
      //left,right,up-right,up-left in robot frame
      SPose p1(0,0.178,0);
      SPose p2(0,-0.178,0);
      SPose p3(0.3,-0.178,0);
      SPose p4(0.3,0.178,0);
      //transform from robot_frame to world_frame(map)
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
         CUtil::getMinMax(poseInMap1.y,poseInMap2.y,poseInMap3.y,poseInMap4.y,frontYmin,frontYmax))
      {
        for(int x=frontXmin;x<frontXmax;x++)
        {
          for(int y=frontYmin;y<frontYmax;y++)
          {
            cv::Point p(x,m_map.info.height-y);
            vector<cv::Point>::iterator iter = find(cells[m_iNextCellInx].vertices_.begin(),cells[m_iNextCellInx].vertices_.end(),p);
            if( m_map.data[MAP_IDX(m_map.info.width,x,y)]>0 || iter!=cells[m_iNextCellInx].vertices_.end())
            {
              GMapping::Point wPose = (*m_pSmap).map2world(x,y);
              tf::Vector3 vp=transform.inverse()*tf::Vector3(wPose.x,wPose.y,0);//transform from world_frame to robot_frame
              SPose rPose(vp.getX(),vp.getY(),0);
              if(CUtil::isPointInRect(p1,p2,p3,p4,rPose))
              {
                m_vBlock.push_back(rPose);
                double d=rPose.x-sqrt(0.178*0.178-pow(rPose.y,2));
                if(d<m_dMinBlock)
                {
                  m_dMinBlock=d;
                  m_NearestObstale=rPose;
                }
              }
            }
          }
        }
      }
      if(m_vBlock.size()>0)
      {
        ROS_INFO("map block!");
        m_bMapBlock=true;
      }
    }
  }
}

bool CNav::getOdomPose(SPose &pose,tf::StampedTransform &transform)
{
  try
  {
    m_listener.lookupTransform(m_strOdomFrame, m_strBaseFrame,
                               ros::Time(0), transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
  }
  pose.x = transform.getOrigin().x();
  pose.y = transform.getOrigin().y();
  pose.w = tf::getYaw(transform.getRotation());
  return true;
}

void CNav::handleInit()
{
  //get the first cell according to the distance to the original pose
  if(getNextCell(m_CurrentPose))
    m_eState=SGONEXTCELL;
  else
    m_eState=SSTOP;
}

void CNav::handleGoStraight()
{
  if (m_pClient == NULL) {
    ROS_ERROR("client null, should not happen!");
    return;
  }

  m_LastPose = m_CurrentPose;
  double angle;
  if((++m_iGoStraightCount)%2==1)
  {
    m_bIsLeft=m_bNextCellTurnLeft;
    angle=m_dNextCellInitAngle;
  }
  else
  {
    m_bIsLeft=!m_bNextCellTurnLeft;
    angle=m_dNextCellInitAngle+PI;
  }
//  theta-=0.05;
//  turn_angle=CUtil::normalizeYaw(turn_angle);
  m_GoalPose.w=CUtil::normalizeYaw(angle);
  m_GoalPose.x = m_CurrentPose.x + cos(m_GoalPose.w) * m_dFrontGap;
  m_GoalPose.y = m_CurrentPose.y + sin(m_GoalPose.w) * m_dFrontGap;
  m_pClient->send(m_GoalPose);
  m_eState = SGOSTRAIGHT_DOING;

  ROS_INFO("Go Straight!");
}

void CNav::handleGoStraightDoing()
{
  if (m_pClient == NULL)
  {
    ROS_ERROR("client null, should not happen!");
    return;
  }

  if (m_pClient->getState() == CMoveBase::STATE_DONE)
  {
    m_eState =SGOSTRAIGHT;
    return;
  }
  if (m_bBlock)
  {
    m_eState = SGOBACK;
    m_dBackGap=0.8;
    m_bBlock=false;
    return;
  }
  else if(m_bMapBlock)
  {
    ROS_INFO("minBlock=%f",m_dMinBlock);
    if(m_dMinBlock<0.5)
    {
      m_eState=SGOBACK;
      m_dBackGap=0.18;
    }
    else
    {
      m_eState=STURN;
    }
    return;
  }

  double dDist = pow(m_CurrentPose.x - m_LastPose.x, 2) + pow(m_CurrentPose.y - m_LastPose.y, 2);
  if (dDist > pow(m_dFrontGap / 2.0f, 2))
  {
    m_LastPose = m_CurrentPose;
    m_GoalPose.x = m_CurrentPose.x + cos(m_CurrentPose.w) * m_dFrontGap;
    m_GoalPose.y = m_CurrentPose.y + sin(m_CurrentPose.w) * m_dFrontGap;
    m_GoalPose.w = m_CurrentPose.w;
    m_pClient->setGoal(m_GoalPose);
  }
}

void CNav::handleGoBack()
{
  if (m_pClient == NULL)
  {
    ROS_ERROR("client null, should not happen!");
    return;
  }

  srand(time(NULL));
  m_LastPose = m_CurrentPose;
  m_GoalPose.x = m_CurrentPose.x - cos(m_CurrentPose.w) * m_dBackGap;
  m_GoalPose.y = m_CurrentPose.y - sin(m_CurrentPose.w) * m_dBackGap;
  m_GoalPose.w = m_CurrentPose.w;
  m_pClient->sendBack(m_GoalPose);
  m_eState = SGOBACK_DOING;
  ROS_INFO("Go Back");
}

void CNav::handleGoBackDoing()
{
  if (m_pClient == NULL)
  {
    ROS_ERROR("client null, should not happen!");
    return;
  }

  if (m_pClient->getState() != CMoveBase::STATE_DONE)
    return;

  if(m_bSideBack)
  {
    m_eState=STURN_SECOND;
    m_bSideBack=false;
  }
  else
    m_eState = STURN;
}

void CNav::handleTurn()
{
  if (m_pClient == NULL)
  {
    ROS_ERROR("client null, should not happen!");
    return;
  }

  if(m_bBlock)
    m_dTurnAngle=PI/2;
  else if(m_bMapBlock)
    calTurnAngle();

  double angle=m_dTurnAngle+m_dAdjustAngle;
  m_LastPose = m_CurrentPose;
  m_GoalPose.x = m_CurrentPose.x;
  m_GoalPose.y = m_CurrentPose.y;
  m_GoalPose.w = m_CurrentPose.w + (m_bIsLeft?angle:-angle);
  m_GoalPose.w = CUtil::normalizeYaw(m_GoalPose.w);
  m_pClient->sendTurn(m_GoalPose);
  m_eState =STURN_DOING;

//  ROS_INFO("Turn,CurrentAngle=%f,Angle=%f,GoalAngle=%f",m_CurrentPose.w,angle,m_GoalPose.w);
}

void CNav::handleTurnDoing()
{
  if (m_pClient == NULL)
  {
    ROS_ERROR("client null, should not happen!");
    return;
  }
  if (m_pClient->getState() != CMoveBase::STATE_DONE) {
    return;
  }

  m_eState = SSIDEWALK;

}

void CNav::handleSideWalk()
{
  if (m_pClient == NULL)
  {
    ROS_ERROR("client null, should not happen!");
    return;
  }

  m_LastPose = m_CurrentPose;
  srand(time(NULL));
  double gap=0.35;
  double gap_test=0.5;
  SPose pose;
  if(getOdomPose(pose,transform))
  {
    SPose p1(0.178,0.178,0);
    SPose p2(0.178,-0.178,0);
    SPose p3(0.178+gap_test,-0.178,0);
    SPose p4(0.178+gap_test,0.178,0);
    //transform to world frame
    tf::Vector3 wp1=transform*tf::Vector3(0.178,0.178,0);
    tf::Vector3 wp2=transform*tf::Vector3(0.178,-0.178,0);
    tf::Vector3 wp3=transform*tf::Vector3(0.178+gap_test,-0.178,0);
    tf::Vector3 wp4=transform*tf::Vector3(0.178+gap_test,0.178,0);
    //map to 2D grid map
    GMapping::IntPoint poseInMap1 = (*m_pSmap).world2map(GMapping::Point(wp1.getX(),wp1.getY()));
    GMapping::IntPoint poseInMap2 = (*m_pSmap).world2map(GMapping::Point(wp2.getX(),wp2.getY()));
    GMapping::IntPoint poseInMap3=(*m_pSmap).world2map(GMapping::Point(wp3.getX(),wp3.getY()));
    GMapping::IntPoint poseInMap4=(*m_pSmap).world2map(GMapping::Point(wp4.getX(),wp4.getY()));
    double minBlock=10.0;
    int frontXmin,frontXmax,frontYmin,frontYmax;
    if(CUtil::getMinMax(poseInMap1.x,poseInMap2.x,poseInMap3.x,poseInMap4.x,frontXmin,frontXmax)&&
       CUtil::getMinMax(poseInMap1.y,poseInMap2.y,poseInMap3.y,poseInMap4.y,frontYmin,frontYmax))
    {
      for(int x=frontXmin;x<frontXmax;x++)
      {
        for(int y=frontYmin;y<frontYmax;y++)
        {
          //block or not,cell contour or not.
          cv::Point p(x,m_map.info.height-y);
          vector<cv::Point>::iterator iter=find(cells[m_iNextCellInx].vertices_.begin(),cells[m_iNextCellInx].vertices_.end(),p);
          if(m_map.data[MAP_IDX(m_map.info.width,x,y)]>0 || iter!=cells[m_iNextCellInx].vertices_.end())
          {
            GMapping::Point wPose = (*m_pSmap).map2world(x,y);
            tf::Vector3 vp=transform.inverse()*tf::Vector3(wPose.x,wPose.y,0);
            SPose rPose(vp.getX(),vp.getY(),0);
            if(CUtil::isPointInRect(p1,p2,p3,p4,rPose))
            {
              double d=rPose.x-sqrt(0.178*0.178-pow(rPose.y,2));
              if(m_map.data[MAP_IDX(m_map.info.width,x,y)]>0)
              {
                if(d<minBlock)
                {
                  minBlock=d;
                }
              }
            }
          }
        }
      }

      ROS_INFO("minBlock=%f",minBlock);
      if(fabs(10.0-minBlock)>0.00001)
      {
        if(minBlock<0.1)
        {
          m_eState=SCELLSTOP;
          return;
        }
        else if(minBlock<0.35)
          gap=minBlock;
      }
    }
  }
  gap=gap/sin(m_dTurnAngle);
  m_GoalPose.x = m_CurrentPose.x + cos(m_CurrentPose.w) * gap;
  m_GoalPose.y = m_CurrentPose.y + sin(m_CurrentPose.w) * gap;
  m_GoalPose.w = m_CurrentPose.w;
  m_pClient->send(m_GoalPose);
  m_eState = SSIDEWALK_DOING;
  ROS_INFO("SideWalk,CurrentAngle=%f,gap=%f",m_CurrentPose.w, gap);
}

void CNav::handleSideWalkDoing()
{
  if (m_pClient == NULL)
  {
    ROS_ERROR("client null, should not happen!");
    return;
  }

  if (m_pClient->getState() != CMoveBase::STATE_DONE)
  {
    return;
  }

  if(m_bBlock)
  {
    m_bSideBack=true;
    m_bBlock=false;
    m_pClient->sendBack(m_LastPose);
    m_eState=SGOBACK_DOING;
    return;
  }

  m_eState = STURN_SECOND;

}

void CNav::handleTurnSecond()
{
  if (m_pClient == NULL)
  {
    ROS_ERROR("client null, should not happen!");
    return;
  }

  m_LastPose = m_CurrentPose;
  m_GoalPose.x = m_CurrentPose.x;
  m_GoalPose.y = m_CurrentPose.y;
  double angle;
  if(m_iGoStraightCount%2==1)
    angle=m_dNextCellInitAngle+PI;
  else
    angle=m_dNextCellInitAngle;
  m_GoalPose.w = angle+m_dAdjustAngle;
  m_GoalPose.w = CUtil::normalizeYaw(m_GoalPose.w);
  m_pClient->sendTurn(m_GoalPose);

  m_eState =STRUN_SECOND_DOING;
//  ROS_INFO("Turn Second,TurnAngle=%f,isLeft=%d,currentPose=%f,goalPoseAngle=%f",angle,m_bIsLeft,m_CurrentPose.w,m_GoalPose.w);
}

void CNav::handleTurnSecondDoing()
{
  if (m_pClient == NULL)
  {
    ROS_ERROR("client null, should not happen!");
    return;
  }

  if (m_pClient->getState() != CMoveBase::STATE_DONE)
  {
    return;
  }

  m_bMapBlock=false;
  m_eState = SGOSTRAIGHT;
}

void CNav::handleCellStop()
{
  if (m_pClient == NULL)
  {
    ROS_ERROR("client null, should not happen!");
    return;
  }

  vector<CellPolygon>::iterator it=cells.begin();
  for(;it!=cells.end();)
  {
    if(it->vertices_ == cells[m_iNextCellInx].vertices_)
    {
      it=cells.erase(it);
      m_iNextCellInx=-1;
      m_iNextCellCornerInx=-1;
    }
    else
      it++;
  }

  if(getNextCell(m_CurrentPose))
    m_eState=SGONEXTCELL;
  else
    m_eState=SSTOP;
}

void CNav::handleGoNextCell()
{
  if (m_pClient == NULL)
  {
    ROS_ERROR("client null, should not happen!");
    return;
  }

  ROS_INFO("Go next region");
  GMapping::Point worldPose=(*m_pSmap).map2world(m_NextCellInitPose.x,m_NextCellInitPose.y);
  ROS_INFO("nextPose:x=%f,y=%f",worldPose.x,worldPose.y);
  MoveBaseClient ac("move_base",true);
  if(!ac.waitForServer(ros::Duration(60)))
  {
    ROS_INFO("Can't connected to move base server");
    return;
  }
  ROS_INFO("Connected to move base server");
  ROS_INFO("Starting navigation test");
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x=worldPose.x;
  goal.target_pose.pose.position.y=worldPose.y;
  goal.target_pose.pose.position.z=0;
  goal.target_pose.pose.orientation.x=0;
  goal.target_pose.pose.orientation.y=0;
  goal.target_pose.pose.orientation.z=0;
  goal.target_pose.pose.orientation.w=1;
  ac.sendGoal(goal);
  ROS_INFO("sending move_base goal");
  bool finished_within_time = ac.waitForResult();
  if(!finished_within_time)
  {
    ac.cancelGoal();
    ROS_INFO("Timed out achieving goal");
  }
  else
  {
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      m_eState=SCELLTURN;
      ROS_INFO("Goal succeeded!");
    }
    else
    {
      m_eState=SSTOP;
      ROS_INFO("The base failed for some reason");
    }
  }
}

void CNav::handleCellTurn()
{
  if(m_pClient == NULL)
  {
    ROS_INFO("client null,should not happen!");
    return;
  }

  m_dNextCellInitAngle=cells[m_iNextCellInx].rotation_angle_;
  if(m_iNextCellCornerInx%2!=0)
    m_dNextCellInitAngle=PI+m_dNextCellInitAngle;
  double angle=m_dNextCellInitAngle+m_dAdjustAngle;
  m_LastPose = m_CurrentPose;
  m_GoalPose.x=m_CurrentPose.x;
  m_GoalPose.y=m_CurrentPose.y;
  m_GoalPose.w=/*m_CurrentPose.w + */angle;
  m_GoalPose.w=CUtil::normalizeYaw(m_GoalPose.w);

  double a=CUtil::normalizeYaw(m_GoalPose.w+PI/2);//left
  Eigen::AngleAxisd angle_axisd(a, Eigen::Vector3d(0, 0, 1));
  Eigen::Vector3d vector(1,0,0);
  Eigen::Vector3d n1=angle_axisd*vector;
  //cout << "(1, 0, 0)绕z轴旋转angle后:" << endl << n1.transpose() << endl;

  int inx;
  if(m_iNextCellCornerInx==0)
    inx=3;
  else if(m_iNextCellCornerInx==1)
    inx=2;
  else if(m_iNextCellCornerInx==2)
    inx=1;
  else
    inx=0;
  cv::Point p1=cells[m_iNextCellInx].out_corners_[m_iNextCellCornerInx];
  cv::Point p2=cells[m_iNextCellInx].out_corners_[inx];

  //  Eigen::Vector3d n2;
  //  n2<<p2.x-p1.x,p2.y-p1.y<<0;
  double d=n1(0,0)*(p2.x-p1.x)+n1(1,0)*(p2.y-p1.y);
  if(d>0)//turn left
    m_bNextCellTurnLeft=true;
  else
    m_bNextCellTurnLeft=false;

  m_pClient->sendTurn(m_GoalPose);
  m_eState = SCELLTURNDOING;
}

void CNav::handleCellTurnDoing()
{
  if(m_pClient == NULL)
  {
    ROS_INFO("client null,should not happen!");
    return;
  }

  if(m_pClient->getState()!=CMoveBase::STATE_DONE)
  {
    return;
  }
  m_iGoStraightCount=0;
  m_eState=SGOSTRAIGHT;
}

void CNav::handleStop()
{
  m_pClient->setState(CMoveBase::STATE_DONE);
  ROS_INFO("Stop");
}

void CNav::StartService()
{
  ROS_INFO("init");
  if (m_pClient == NULL)
  {
    m_pClient = new CMoveBase(m_pubCmdVel);
  }

  ros::Rate loopRate(20);
  while (ros::ok())
  {
    ros::spinOnce();
    if (!getOdomPose(m_CurrentPose,transform))
    {
      ROS_ERROR("get pose failed");
      loopRate.sleep();
      continue;
    }
    m_pClient->spinOnce(m_CurrentPose);
    switch (m_eState)
    {
      case SINIT:
      {
        handleInit();
        break;
      }
      case SGOSTRAIGHT:
      {
        handleGoStraight();
        break;
      }
      case SGOSTRAIGHT_DOING:
      {
        handleGoStraightDoing();
        break;
      }
      case SGOBACK:
      {
        handleGoBack();
        break;
      }
      case SGOBACK_DOING:
      {
        handleGoBackDoing();
        break;
      }
      case STURN:
      {
        handleTurn();
        break;
      }
      case STURN_DOING:
      {
        handleTurnDoing();
        break;
      }
      case SSIDEWALK:
      {
        handleSideWalk();
        break;
      }
      case SSIDEWALK_DOING:
      {
        handleSideWalkDoing();
        break;
      }
      case STURN_SECOND:
      {
        handleTurnSecond();
        break;
      }
      case STRUN_SECOND_DOING:
      {
        handleTurnSecondDoing();
        break;
      }
      case SCELLSTOP:
      {
        handleCellStop();
        break;
      }
      case SGONEXTCELL:
      {
        handleGoNextCell();
        break;
      }
      case SCELLTURN:
      {
        handleCellTurn();
        break;
      }
      case SCELLTURNDOING:
      {
        handleCellTurnDoing();
        break;
      }
      case SSTOP:
      {
        handleStop();
        break;
      }
      default:
      {
        ROS_ERROR("do nothing, should not happen!");
        break;
      }
    }
    loopRate.sleep();
  }
}

void CNav::calTurnAngle()
{
  //calculate the obstacles in the left of the NearestObstacle
//  ROS_INFO("nearest obstacle:x=%f,y=%f",m_NearestObstale.x,m_NearestObstale.y);
  vector<SPose> vObstacle;
  for(vector<SPose>::iterator it=m_vBlock.begin();it!=m_vBlock.end();++it)
  {
    SPose p=*it;
    if((m_bIsLeft && p.y>m_NearestObstale.y)||(!m_bIsLeft && p.y<m_NearestObstale.y))
    {
      vObstacle.push_back(p);
    }
  }
  SPose ppair=m_NearestObstale;
  for(vector<SPose>::iterator it=vObstacle.begin();it!=vObstacle.end();++it)
  {
    SPose p=*it;
    double slope=fabs((p.x-m_NearestObstale.x)/(p.y-m_NearestObstale.y));
    double offset=p.x-slope*fabs(p.y);//>0
    vector<SPose>::iterator iter=vObstacle.begin();
    for(;iter!=vObstacle.end();++iter)
    {
      if(iter!=it)
      {
        SPose p1=*iter;
        double outValue=slope*fabs(p1.y)+offset;
        if(p1.x<outValue)
        {
          break;
        }
      }
    }
    if(iter==vObstacle.end())
    {
      ppair=p;
      break;
    }
  }

  //calculate the turn angle
  double a=m_NearestObstale.y-ppair.y;
  double b=ppair.x-m_NearestObstale.x;
  if(fabs(b)<0.0001)
  {
    m_dTurnAngle=PI/2;
  }
  else
  {
    double t=a/b;
    if(t>0.0001)
    {
      if(m_bIsLeft)
        m_dTurnAngle=PI-atan(t);
      else
        m_dTurnAngle=atan(t);
    }
    else if(t<-0.0001)
    {
      if(m_bIsLeft)
        m_dTurnAngle=atan(-t);
      else
        m_dTurnAngle=PI-atan(-t);
    }
  }
}

bool CNav::transformWorld2Robot(const GMapping::Point &wPose, SPose &rPose)
{
  geometry_msgs::PointStamped wtfPose;
  wtfPose.header.stamp=ros::Time::now();
  wtfPose.header.frame_id=m_strOdomFrame;
  wtfPose.point.x=wPose.x;
  wtfPose.point.y=wPose.y;
  wtfPose.point.z=0;
  geometry_msgs::PointStamped rtfPose;
  try
  {
    m_listener.transformPoint(m_strBaseFrame,wtfPose,rtfPose);
  }
  catch(tf::TransformException &ex)
  {
    ROS_ERROR("Failed to compute pose in base_link,%s",ex.what());
    return false;
  }
  rPose.x=rtfPose.point.x;
  rPose.y=rtfPose.point.y;
  return true;
}

bool CNav::getNextCell(const SPose &rPose)
{
  //calculate the next cell according to the distance to current robot pose
  if(cells.size()==0)
  {
    ROS_INFO("No cell left,clean is over!");
    return false;
  }

  GMapping::IntPoint mapPose=(*m_pSmap).world2map(GMapping::Point(rPose.x,rPose.y));
  cv::Point imgPose(mapPose.x,m_map.info.height-mapPose.y);
//  m_iNextCellInx=-1;
//  corner_inx=-1;
  double min_distance=0x7fffff;
  for(size_t i=0;i<cells.size();++i)
  {
    CellPolygon cell=cells[i];
    cv::Point* cell_corners=cell.out_corners_;
    for(size_t j=0;j<4;++j)
    {
      double d=sqrt(pow(cell_corners[j].x-imgPose.x,2)+pow(cell_corners[j].y-imgPose.y,2));
      if(d<min_distance)
      {
        m_iNextCellInx=i;
        m_iNextCellCornerInx=j;
      }
    }
  }

  if(m_iNextCellInx!=-1 && m_iNextCellCornerInx!=-1)
  {
    m_NextCellInitPose.x=cells[m_iNextCellInx].out_corners_[m_iNextCellCornerInx].x;
    m_NextCellInitPose.y=m_map.info.height-cells[m_iNextCellInx].out_corners_[m_iNextCellCornerInx].y;
    return true;
  }
  return false;
}


