#ifndef MEANSHIFT2D_H
#define MEANSHIFT2D_H
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


class MeanShift2D
{
public:
  MeanShift2D(void) {}

  void filter(const std::vector<cv::Vec2d>& data, std::vector<cv::Vec2d>& filtered_data, const double bandwidth, const int maximumIterations=100);

  void computeConvergencePoints(const std::vector<cv::Vec2d>& filtered_data, std::vector<cv::Vec2d>& convergence_points, std::vector< std::vector<int> >& convergence_sets, const double sensitivity);

  // map_resolution in [m/cell]
  cv::Vec2d findRoomCenter(const cv::Mat& room_image, const std::vector<cv::Vec2d>& room_cells, double map_resolution);
};

#endif // MEANSHIFT2D_H
