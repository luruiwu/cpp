#ifndef IMI_COVERAGE_PATH_PLANNING_H
#define IMI_COVERAGE_PATH_PLANNING_H

#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/Path.h>
// Eigen library
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
//MoveBase
#include "MoveBaseClient.h"
#include "MoveBase.h"
#include <move_base_msgs/MoveBaseAction.h>
//boustrophedon
#include <room_exploration/boustrophedon.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class ImiCoveragePathPlanning
{
public:
  ImiCoveragePathPlanning(ros::NodeHandle nh);
  ~ImiCoveragePathPlanning();

private:
  bool removeUnconnectRoomParts(cv::Mat &room_map);
  bool publishNavigationGoal(const geometry_msgs::Pose2D & nav_goal,std::vector<geometry_msgs::Pose2D> &robot_poses,const double &eps);
  void navigateExplorationPath(const std::vector<geometry_msgs::Pose2D> &exploration_path,const double &grid_spacing_in_pixel);
  void printCoveragePath(const cv::Mat &map,const std::vector<geometry_msgs::Pose2D> &exploration_path,const cv::Point2d &map_origin);

public:
  void start();

private:
  ros::NodeHandle private_nh_;
  ros::Publisher path_pub_;
  BoustrophedonExplorer boustrophedon_explorer_;
  std::string map_path_;
  double min_cell_area_;
  double path_eps_;
  double grid_obstacle_offset_;
  geometry_msgs::Pose map_origin_;
  float map_resolution_;
  float robot_radius_;
  cv::Point starting_position_;
  double coverage_radius_;
  double grid_spacing_in_meter_;
  bool use_dyn_goal_eps_;
  double goal_eps_;
  bool print_path_;
  bool navigate_path_;
  std::string base_frame_;
  std::string odom_frame_;

};

#endif // IMI_COVERAGE_PATH_PLANNING_H
