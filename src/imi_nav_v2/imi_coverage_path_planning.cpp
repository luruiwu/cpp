#include <imi_nav_v2/imi_coverage_path_planning.h>

ImiCoveragePathPlanning::ImiCoveragePathPlanning(ros::NodeHandle nh):
private_nh_("~"),map_path_(""),min_cell_area_(0.0f),path_eps_(0.0f),grid_obstacle_offset_(0.0f),map_resolution_(0.0f),
robot_radius_(0.0f),coverage_radius_(0.0f),grid_spacing_in_meter_(0.0f),use_dyn_goal_eps_(true),goal_eps_(0.0f),
print_path_(true),navigate_path_(true),base_frame_(""),odom_frame_(""){

  private_nh_.param<std::string>("map_path",map_path_,"/home/exbot/.ros/map.pgm");
  private_nh_.param<double>("min_cell_area",min_cell_area_,10.0f);
  private_nh_.param<double>("path_eps",path_eps_,2.0f);
  private_nh_.param<double>("grid_obstacle_offset",grid_obstacle_offset_,0.0f);
  private_nh_.param<float>("map_resolution",map_resolution_,0.1);
  private_nh_.param<float>("robot_radius",robot_radius_,0.178);
  private_nh_.param<double>("coverage_radius",coverage_radius_,0.18);
  private_nh_.param<bool>("use_dyn_goal_eps",use_dyn_goal_eps_,true);
  private_nh_.param<double>("goal_eps",goal_eps_,0.35);
  private_nh_.param<bool>("print_path",print_path_,true);
  private_nh_.param<bool>("navigate_path",navigate_path_,true);
  private_nh_.param<std::string>("base_frame",base_frame_,"base_link");
  private_nh_.param<std::string>("odom_frame",odom_frame_,"odom");
  grid_spacing_in_meter_=coverage_radius_*std::sqrt(2);
  map_origin_.position.x=0;
  map_origin_.position.y=0;
  map_origin_.position.z=0;
  starting_position_.x=180;
  starting_position_.y=170;

  path_pub_=nh.advertise<nav_msgs::Path>("coverage_path",2);
}

ImiCoveragePathPlanning::~ImiCoveragePathPlanning(){}

bool ImiCoveragePathPlanning::removeUnconnectRoomParts(cv::Mat &room_map){
  cv::Mat room_map_int(room_map.rows, room_map.cols, CV_32SC1);
  for (int v=0; v<room_map.rows; ++v)
  {
    for (int u=0; u<room_map.cols; ++u)
    {
      if (room_map.at<uchar>(v,u) == 255)
        room_map_int.at<int32_t>(v,u) = -100;
      else
        room_map_int.at<int32_t>(v,u) = 0;
    }
  }

  std::map<int, int> area_to_label_map;
  int label = 1;
  for (int v=0; v<room_map_int.rows; ++v)
  {
    for (int u=0; u<room_map_int.cols; ++u)
    {
      if (room_map_int.at<int32_t>(v,u) == -100)
      {
        const int area = cv::floodFill(room_map_int, cv::Point(u,v), cv::Scalar(label), 0, 0, 0, 8 | cv::FLOODFILL_FIXED_RANGE);
        area_to_label_map[area] = label;
        ++label;
      }
    }
  }
  if (area_to_label_map.size() == 0)
    return false;

  const int label_of_biggest_room = area_to_label_map.rbegin()->second;
//  int area_of_biggest_room=area_to_label_map.rbegin()->first;
//  std::cout << "area_of_biggest_room=" << area_of_biggest_room << std::endl;

  std::cout << "label_of_biggest_room=" << label_of_biggest_room << std::endl;
  for (int v=0; v<room_map.rows; ++v)
    for (int u=0; u<room_map.cols; ++u)
      if (room_map_int.at<int32_t>(v,u) != label_of_biggest_room)
        room_map.at<uchar>(v,u) = 0;

  return true;
}

void ImiCoveragePathPlanning::navigateExplorationPath(const std::vector<geometry_msgs::Pose2D> &exploration_path,const double &grid_spacing_in_pixel){
  std::vector<geometry_msgs::Pose2D> robot_poses;
  geometry_msgs::Pose2D last_pose;
  geometry_msgs::Pose2D pose;
  for(size_t map_oriented_pose=0;map_oriented_pose<exploration_path.size();++map_oriented_pose){
    pose=exploration_path[map_oriented_pose];
    double temp_goal_eps=0;

    if(use_dyn_goal_eps_){
      if(map_oriented_pose!=0){
        double delta_theta=fabs(last_pose.theta-pose.theta);
        if(delta_theta > M_PI*0.5){
          delta_theta=M_PI*0.5;
          temp_goal_eps=(M_PI*0.5-delta_theta)/(M_PI*0.5)*goal_eps_;
        }
      }
    }else{
      temp_goal_eps=goal_eps_;
    }
    publishNavigationGoal(pose,robot_poses,temp_goal_eps);
    last_pose=pose;
  }
  std::cout<<"All navigation goals have been published."<<std::endl;
}

bool ImiCoveragePathPlanning::publishNavigationGoal(const geometry_msgs::Pose2D &nav_goal,std::vector<geometry_msgs::Pose2D> &robot_poses, const double &eps){
  MoveBaseClient mv_base_client("/move_base",true);

  while(mv_base_client.waitForServer(ros::Duration(5.0))==false){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  geometry_msgs::Pose2D map_oriented_pose;
  map_oriented_pose.x=nav_goal.x;
  map_oriented_pose.y=nav_goal.y;
  map_oriented_pose.theta=nav_goal.theta;

  move_base_msgs::MoveBaseGoal move_base_goal;
  move_base_goal.target_pose.header.frame_id="map";
  move_base_goal.target_pose.header.stamp=ros::Time::now();
  move_base_goal.target_pose.pose.position.x = map_oriented_pose.x;
  move_base_goal.target_pose.pose.position.y = map_oriented_pose.y;
  move_base_goal.target_pose.pose.orientation.z = sin(map_oriented_pose.theta/2);
  move_base_goal.target_pose.pose.orientation.w = cos(map_oriented_pose.theta/2);

  // send goal to the move_base sever, when one is found
  ROS_INFO_STREAM("Sending goal with eps " << eps);
  mv_base_client.sendGoal(move_base_goal);
  // wait until goal is reached or the goal is aborted
//	ros::Duration sleep_rate(0.1);
  tf::TransformListener listener;
  tf::StampedTransform transform;
  ros::Duration sleep_duration(0.15); // TODO: param
  bool near_pos;
  do
  {
    near_pos = false;
    double roll, pitch, yaw;
    // try to get the transformation from map_frame to base_frame, wait max. 2 seconds for this transform to come up
    try
    {
      ros::Time time = ros::Time(0);
      listener.waitForTransform(odom_frame_, base_frame_, time, ros::Duration(2.0)); // 5.0
      listener.lookupTransform(odom_frame_, base_frame_, time, transform);

      sleep_duration.sleep();

      // save the current pose if a transform could be found
      geometry_msgs::Pose2D current_pose;

      current_pose.x = transform.getOrigin().x();
      current_pose.y = transform.getOrigin().y();
      transform.getBasis().getRPY(roll, pitch, yaw);
      current_pose.theta = yaw;

      if((current_pose.x-map_oriented_pose.x)*(current_pose.x-map_oriented_pose.x) + (current_pose.y-map_oriented_pose.y)*(current_pose.y-map_oriented_pose.y) <= eps*eps)
        near_pos = true;

      robot_poses.push_back(current_pose);
    }
    catch(tf::TransformException &ex)
    {
      ROS_WARN_STREAM("Couldn't get transform from " << odom_frame_ << " to " << base_frame_ << "!");// %s", ex.what());
    }

  }while(mv_base_client.getState() != actionlib::SimpleClientGoalState::ABORTED && mv_base_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED
      && near_pos == false);

  // check if point could be reached or not
  if(mv_base_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED || near_pos == true)
  {
    ROS_INFO("current goal could be reached.");
    return true;
  }else{
    ROS_INFO("current goal could not be reached.");
    return false;
  }
}

void ImiCoveragePathPlanning::printCoveragePath(const cv::Mat &map,const std::vector<geometry_msgs::Pose2D> &exploration_path,const cv::Point2d &map_origin){
  cv::Mat fov_path_map;
  std::vector<geometry_msgs::PoseStamped> exploration_path_pose_stamped(exploration_path.size());
  std_msgs::Header header;
  header.stamp=ros::Time::now();
  header.frame_id="map";
  for(size_t step=0;step<exploration_path.size();++step){
    if(step>0){
      fov_path_map=map.clone();
      cv::resize(fov_path_map,fov_path_map,cv::Size(),2,2,cv::INTER_LINEAR);
      if(exploration_path.size()>0){
        cv::circle(fov_path_map,2*cv::Point((exploration_path[0].x-map_origin.x)/map_resolution_, (exploration_path[0].y-map_origin.y)/map_resolution_), 2, cv::Scalar(150), CV_FILLED);
      }
      for(size_t i=1;i<=step;++i){
        cv::Point p1((exploration_path[i-1].x-map_origin.x)/map_resolution_, (exploration_path[i-1].y-map_origin.y)/map_resolution_);
        cv::Point p2((exploration_path[i].x-map_origin.x)/map_resolution_, (exploration_path[i].y-map_origin.y)/map_resolution_);
        cv::circle(fov_path_map, 2*p2, 2, cv::Scalar(200), CV_FILLED);//elarge the original picture twice
        cv::line(fov_path_map, 2*p1, 2*p2, cv::Scalar(150), 1);
        cv::Point p3(p2.x+5*cos(exploration_path[i].theta), p2.y+5*sin(exploration_path[i].theta));
        if (i==step)
        {
          cv::circle(fov_path_map, 2*p2, 2, cv::Scalar(80), CV_FILLED);
          cv::line(fov_path_map, 2*p1, 2*p2, cv::Scalar(150), 1);
          cv::line(fov_path_map, 2*p2, 2*p3, cv::Scalar(50), 1);
        }
      }
    }
    //path pose
    exploration_path_pose_stamped[step].header=header;
    exploration_path_pose_stamped[step].header.seq=step;
    exploration_path_pose_stamped[step].pose.position.x=exploration_path[step].x;
    exploration_path_pose_stamped[step].pose.position.y=exploration_path[step].y;
    exploration_path_pose_stamped[step].pose.position.z=0.0;
    Eigen::Quaterniond quaternion;
    quaternion = Eigen::AngleAxisd((double)exploration_path[step].theta, Eigen::Vector3d::UnitZ());
    tf::quaternionEigenToMsg(quaternion,exploration_path_pose_stamped[step].pose.orientation);
  }
  cv::imshow("CoveragePath",fov_path_map);
  cv::waitKey(0);
}

void ImiCoveragePathPlanning::start(){
  cv::Mat grayMap=cv::imread(map_path_,0);
  cv::Mat map;
  cv::threshold(grayMap,map,250,255,cv::THRESH_BINARY);

  cv::Mat temp;
  cv::erode(map, temp, cv::Mat(), cv::Point(-1, -1), 2);
  cv::dilate(temp, map, cv::Mat(), cv::Point(-1, -1), 2);

  bool room_not_empty = removeUnconnectRoomParts(map);
  if(!room_not_empty){
    std::cout<<"The given room is too small to deal."<<std::endl;
    return;
  }

  std::vector<geometry_msgs::Pose2D> exploration_path;
  double grid_spacing_in_pixel=grid_spacing_in_meter_/map_resolution_;
  cv::Point2d map_origin(0,0);
  Eigen::Matrix<float,2,1> zero_vector;
  zero_vector<<0,0;
  boustrophedon_explorer_.getExplorationPath(map,exploration_path,map_resolution_,starting_position_,map_origin,grid_spacing_in_pixel,
                                             grid_obstacle_offset_,path_eps_,true,zero_vector,min_cell_area_);

  if(print_path_){
    printCoveragePath(map,exploration_path,map_origin);
    std::cout<<"Coverage Path has been printed."<<std::endl;
  }

  if(navigate_path_){
    navigateExplorationPath(exploration_path,grid_spacing_in_pixel);
  }

}
