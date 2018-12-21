#include<imi_nav_v2/imi_nav_bcd.h>

bool removeUnconnectRoomParts(cv::Mat &room_map){
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

int main(int argc,char** argv)
{
  ros::init(argc,argv,"imi_nav_node_v2");
  ros::NodeHandle nh("~");

  cv::Mat grayMap=cv::imread("/home/exbot/.ros/map.pgm",0);//0 respects grayscale image
  cv::Mat map;
  cv::threshold(grayMap,map,250,255,cv::THRESH_BINARY);

//  cv::imshow("map",map);
//  cv::waitKey();

  cv::Mat temp;
  cv::erode(map, temp, cv::Mat(), cv::Point(-1, -1), 2);
  cv::dilate(temp, map, cv::Mat(), cv::Point(-1, -1), 2);

  bool room_not_empty = removeUnconnectRoomParts(map);
  if(!room_not_empty){
    std::cout<<"The given room is too small to deal."<<std::endl;
    return 0;
  }

  std::vector<CellPolygon> cells;
  double coverage_radius_=0.18;
  double grid_spacing_in_meter_=coverage_radius_*sqrt(2);
  float map_resolution_=0.1;
  double grid_spacing_in_pixel=grid_spacing_in_meter_/map_resolution_;
  double grid_obstacle_offset_=0.2;
  double path_eps_=2.0;
  double min_cell_area_=10.0;

  CellDecomposition::decomposition(map,map_resolution_,grid_spacing_in_pixel,grid_obstacle_offset_,path_eps_,min_cell_area_,cells);

  CellPolygon cell=cells[0];

  ROS_INFO("rotation_angle:%f",cell.rotation_angle_);

//  cv::Mat blackImage;

//  cell.drawPolygon(blackImage,cv::Scalar(255));
//  cv::imshow("cell0",blackImage);
//  cv::waitKey();

  CNav nav(cells);

  nav.StartService();

}
