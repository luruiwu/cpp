#ifndef CELL_DECOMPOSITON_H
#define CELL_DECOMPOSITON_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <string>

#include <Eigen/Dense>
#include <room_exploration/meanshift2d.h>
#include <room_exploration/fov_to_robot_mapper.h>
#include <room_exploration/room_rotator.h>
#include <room_exploration/grid.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>

#define PI 3.14159265359

class CellPolygon
{
public:
  // vertexes
  std::vector<cv::Point> vertices_;

  // min/max coordinates
  int max_x_,min_x_,max_y_,min_y_;

  //four out_corners
  cv::Point out_corners_[4];

  //center point
  cv::Point center_;

  //rotation angle
  double rotation_angle_;

public:
  // constructor
  CellPolygon(const std::vector<cv::Point>& vertices,const double map_resolution)
  {
    // save given vertexes
    vertices_ = vertices;

    max_x_=0;
    min_x_=100000;
    max_y_=0;
    min_y_=100000;
    for(size_t point=0; point<vertices_.size(); ++point)
    {
      if(vertices_[point].x > max_x_)
        max_x_ = vertices_[point].x;
      if(vertices_[point].y > max_y_)
        max_y_ = vertices_[point].y;
      if(vertices_[point].x < min_x_)
        min_x_ = vertices_[point].x;
      if(vertices_[point].y < min_y_)
        min_y_ = vertices_[point].y;
    }

    //compute visible center
    MeanShift2D ms;
    cv::Mat room=cv::Mat::zeros(max_y_+10, max_x_+10, CV_8UC1);
    cv::drawContours(room,std::vector<std::vector<cv::Point> >(1,vertices),-1,cv::Scalar(255),CV_FILLED);
    cv::Mat distance_map;
    cv::distanceTransform(room,distance_map,CV_DIST_L2,5);
    double min_val=0.,max_val=0.;
    cv::minMaxLoc(distance_map,&min_val,&max_val);
    std::vector<cv::Vec2d> room_cells;
    for(int v=0;v<distance_map.rows;++v)
    {
      for(int u=0;u<distance_map.cols;++u)
      {
        if(distance_map.at<float>(v,u) > max_val*0.95f)
          room_cells.push_back(cv::Vec2d(u,v));
      }
    }
    cv::Vec2d room_center = ms.findRoomCenter(room,room_cells,map_resolution);
    center_.x=room_center[0];
    center_.y=room_center[1];
  }

  std::vector<cv::Point> getVertices() const
  {
    return vertices_;
  }

  cv::Point getCenter() const
  {
    return center_;
  }

  void drawPolygon(cv::Mat& image, const cv::Scalar& color) const
  {
    // draw polygon in an black image with necessary size
    cv::Mat black_image = cv::Mat(/*max_y_+10*/224, /*max_x_+10*/224, CV_8UC1, cv::Scalar(0));
    cv::drawContours(black_image, std::vector<std::vector<cv::Point> >(1,vertices_), -1, color, 1);

    // assign drawn map
    image = black_image.clone();
  }
};

class CellDecomposition
{
protected:

  static void computeCellDecompositionWithRotation(const cv::Mat& room_map, const float map_resolution, const double min_cell_area,
      const double rotation_offset, cv::Mat& R, cv::Rect& bbox, cv::Mat& rotated_room_map,
      std::vector<CellPolygon>& cell_polygons);

  static void computeCellDecomposition(const cv::Mat& room_map, const float map_resolution, const double min_cell_area,
      std::vector<CellPolygon>& cell_polygons);

  static void correctThinWalls(cv::Mat& room_map);

public:
  // constructor
  CellDecomposition();

  static void decomposition(const cv::Mat& room_map, const float map_resolution,
        const double grid_spacing_in_pixel, const double grid_obstacle_offset, const double path_eps,
        const double min_cell_area, std::vector<CellPolygon>& cell_polygons);

};

#endif // CELL_DECOMPOSITON_H
