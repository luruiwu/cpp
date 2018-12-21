#include <room_exploration/cell_decompositon.h>

//#define DEBUG_VISUALIZATION

// Constructor
CellDecomposition::CellDecomposition()
{

}
void CellDecomposition::decomposition(const cv::Mat& room_map,const float map_resolution,
    const double grid_spacing_in_pixel, const double grid_obstacle_offset, const double path_eps,
    const double min_cell_area, std::vector<CellPolygon>& cell_polygons)
{

  cv::Mat R;
  cv::Rect bbox;
  cv::Mat rotated_room_map;
  computeCellDecompositionWithRotation(room_map, map_resolution, min_cell_area, 0., R, bbox, rotated_room_map, cell_polygons);

  ROS_INFO("Found the cells in the given map.cell_size: %d",cell_polygons.size());
  int grid_spacing_as_int = (int)std::floor(grid_spacing_in_pixel);
  int half_grid_spacing_as_int=(int)std::floor(0.5*grid_spacing_in_pixel);

  cv::Mat R_inv;
  cv::invertAffineTransform(R, R_inv);

  for(size_t i=0; i<cell_polygons.size(); ++i)
  {
    CellPolygon cell=cell_polygons[i];
    cv::Mat cell_map;
    cell.drawPolygon(cell_map ,cv::Scalar(255));
    cv::Point cell_center=cell.getCenter();
    cv::Mat R_cell;
    cv::Rect cell_bbox;
    cv::Mat rotated_cell_map;
    RoomRotator cell_rotation;
    double rotation_angle = cell_rotation.computeRoomRotationMatrix(cell_map,R_cell,cell_bbox,map_resolution, &cell_center);
    cell_rotation.rotateRoom(cell_map,rotated_cell_map,R_cell,cell_bbox);
    cv::Mat inflated_room_map, rotated_inflated_room_map;  
    cv::erode(rotated_room_map, inflated_room_map, cv::Mat(), cv::Point(-1, -1), half_grid_spacing_as_int+grid_obstacle_offset);
    cell_rotation.rotateRoom(inflated_room_map, rotated_inflated_room_map, R_cell, cell_bbox);
    cv::Mat rotated_inflated_cell_map = rotated_cell_map.clone();
    for (int v=0; v<rotated_inflated_cell_map.rows; ++v)
      for (int u=0; u<rotated_inflated_cell_map.cols; ++u)
        if (rotated_inflated_cell_map.at<uchar>(v,u)!=0 && rotated_inflated_room_map.at<uchar>(v,u)==0)
          rotated_inflated_cell_map.at<uchar>(v,u) = 128;
    BoustrophedonGrid grid_lines;
    GridGenerator::generateBoustrophedonGrid(rotated_cell_map, rotated_inflated_cell_map, -1, grid_lines, cv::Vec4i(0, 0, 0, 0), //cv::Vec4i(min_x, max_x, min_y, max_y),
        grid_spacing_as_int, half_grid_spacing_as_int, path_eps/*grid_spacing_as_int*/, grid_obstacle_offset);		//1);
    if(grid_lines.size()==0)
    {
      ROS_INFO("no Boustrophedon lines found!");
      continue;
    }

    //get the corners of each cell
    std::vector<cv::Point> corners(4);
    corners[0] = grid_lines[0].upper_line[0];		// upper left corner
    corners[1] = grid_lines[0].upper_line.back();	// upper right corner
    corners[2] = grid_lines.back().upper_line[0];	// lower left corner
    corners[3] = grid_lines.back().upper_line.back();	// lower right corner

    cv::Mat R_cell_inv;
    cv::invertAffineTransform(R_cell, R_cell_inv);	// invert the rotation matrix to remap the determined points to the original cell
    cv::transform(corners, corners, R_cell_inv); //cell transform
    cv::transform(corners,corners,R_inv);//map transform
    for(int j=0;j<4;j++)
    {
      cell_polygons[i].out_corners_[j].x=corners[j].x;
      cell_polygons[i].out_corners_[j].y=corners[j].y;
    }

    //vertices transform
    std::vector<cv::Point> vertices=cell.vertices_;
    cv::transform(vertices,vertices,R_inv);//map_transform
    cell_polygons[i].vertices_=vertices;

    //set the rotation angle of each cell
    cell_polygons[i].rotation_angle_=rotation_angle;
  }

#ifdef DEBUG_VISUALIZATION
  cv::Mat corners_map=room_map.clone();
  for(size_t c=0;c<cell_polygons.size();++c)
  {
     CellPolygon cell=cell_polygons[c];
     for(int i=1;i<4;++i)
     {
       cv::circle(corners_map,cell.out_corners_[i],1,cv::Scalar(50),CV_FILLED);
       cv::line(corners_map,cell.out_corners_[i-1],cell.out_corners_[i],cv::Scalar(150),1);
     }
  }
  cv::imshow("corners_map",corners_map);
  cv::waitKey();
#endif
}

void CellDecomposition::computeCellDecompositionWithRotation(const cv::Mat& room_map, const float map_resolution, const double min_cell_area,
    const double rotation_offset, cv::Mat& R, cv::Rect& bbox, cv::Mat& rotated_room_map,
    std::vector<CellPolygon>& cell_polygons)
{
  RoomRotator room_rotation;
  room_rotation.computeRoomRotationMatrix(room_map, R, bbox, map_resolution, 0, rotation_offset);
  room_rotation.rotateRoom(room_map, rotated_room_map, R, bbox);
  computeCellDecomposition(rotated_room_map, map_resolution, min_cell_area, cell_polygons);
}

void CellDecomposition::computeCellDecomposition(const cv::Mat& room_map, const float map_resolution, const double min_cell_area,
    std::vector<CellPolygon>& cell_polygons)
{
  // create a map copy to mark the cell boundaries
  cv::Mat cell_map = room_map.clone();

  // find smallest y-value for that a white pixel occurs, to set initial y value and find initial number of segments
  size_t y_start = 0;
  int n_start = 0;
  bool found = false, obstacle = false;
  for(size_t y=0; y<room_map.rows; ++y)
  {
    for(size_t x=0; x<room_map.cols; ++x)
    {
      if(room_map.at<uchar>(y,x) == 255 && found == false)
      {
        y_start = y;
        found = true;
      }
      else if(found == true && obstacle == false && room_map.at<uchar>(y,x) == 0)
      {
        ++n_start;
        obstacle = true;
      }
      else if(found == true && obstacle == true && room_map.at<uchar>(y,x) == 255)
      {
        obstacle = false;
      }
    }

    if(found == true)
      break;
  }

  // swipe trough the map and detect critical points
  int previous_number_of_segments = n_start;
  for(size_t y=y_start+1; y<room_map.rows; ++y) // start at y_start+1 because we know number of segments at y_start
  {
    int number_of_segments = 0; // int to count how many segments at the current slice are
    bool obstacle_hit = false; // bool to check if the line currently hit an obstacle, s.t. not all black pixels trigger an event
    bool hit_white_pixel = false; // bool to check if a white pixel has been hit at the current slice, to start the slice at the first white pixel

    // count number of segments within this row
    for(size_t x=0; x<room_map.cols; ++x)
    {
      if(room_map.at<uchar>(y,x) == 255 && hit_white_pixel == false)
        hit_white_pixel = true;
      else if(hit_white_pixel == true)
      {
        if(obstacle_hit == false && room_map.at<uchar>(y,x) == 0) // check for obstacle
        {
          ++number_of_segments;
          obstacle_hit = true;
        }
        else if(obstacle_hit == true && room_map.at<uchar>(y,x) == 255) // check for leaving obstacle
        {
          obstacle_hit = false;
        }
      }
    }

    // reset hit_white_pixel to use this Boolean later
    hit_white_pixel = false;

    // check if number of segments has changed --> event occurred
    if(previous_number_of_segments < number_of_segments) // IN event
    {
      // check the current slice again for critical points
      for(int x=0; x<room_map.cols; ++x)
      {
        if(room_map.at<uchar>(y,x) == 255 && hit_white_pixel == false)
          hit_white_pixel = true;
        else if(hit_white_pixel == true && room_map.at<uchar>(y,x) == 0)
        {
          // check over black pixel for other black pixels, if none occur a critical point is found
          bool critical_point = true;
          for(int dx=-1; dx<=1; ++dx)
            if(room_map.at<uchar>(y-1,x+dx) == 0)
              critical_point = false;

          // if a critical point is found mark the separation, note that this algorithm goes left and right
          // starting at the critical point until an obstacle is hit, because this prevents unnecessary cells
          // behind other obstacles on the same y-value as the critical point
          if(critical_point == true)
          {
            // to the left until a black pixel is hit
            for(int dx=-1; x+dx>0; --dx)
            {
              if(cell_map.at<uchar>(y,x+dx) == 255)
                cell_map.at<uchar>(y,x+dx) = 0;
              else if(cell_map.at<uchar>(y,x+dx) == 0)
                break;
            }

            // to the right until a black pixel is hit
            for(int dx=1; x+dx<room_map.cols; ++dx)
            {
              if(cell_map.at<uchar>(y,x+dx) == 255)
                cell_map.at<uchar>(y,x+dx) = 0;
              else if(cell_map.at<uchar>(y,x+dx) == 0)
                break;
            }
          }
        }
      }
    }
    else if(previous_number_of_segments > number_of_segments) // OUT event
    {
      // check the previous slice again for critical points --> y-1
      for(int x=0; x<room_map.cols; ++x)
      {
        if(room_map.at<uchar>(y-1,x) == 255 && hit_white_pixel == false)
          hit_white_pixel = true;
        else if(hit_white_pixel == true && room_map.at<uchar>(y-1,x) == 0)
        {
          // check over black pixel for other black pixels, if none occur a critical point is found
          bool critical_point = true;
          for(int dx=-1; dx<=1; ++dx)
            if(room_map.at<uchar>(y,std::min(x+dx, room_map.cols-1)) == 0) // check at side after obstacle
              critical_point = false;

          // if a critical point is found mark the separation, note that this algorithm goes left and right
          // starting at the critical point until an obstacle is hit, because this prevents unnecessary cells
          // behind other obstacles on the same y-value as the critical point
          if(critical_point == true)
          {
            // to the left until a black pixel is hit
            for(int dx=-1; x+dx>0; --dx)
            {
              if(cell_map.at<uchar>(y-1,x+dx) == 255)
                cell_map.at<uchar>(y-1,x+dx) = 0;
              else if(cell_map.at<uchar>(y-1,x+dx) == 0)
                break;
            }

            // to the right until a black pixel is hit
            for(int dx=1; x+dx<room_map.cols; ++dx)
            {
              if(cell_map.at<uchar>(y-1,x+dx) == 255)
                cell_map.at<uchar>(y-1,x+dx) = 0;
              else if(cell_map.at<uchar>(y-1,x+dx) == 0)
                break;
            }
          }
        }
      }
    }

    // save the found number of segments
    previous_number_of_segments = number_of_segments;
  }

#ifdef DEBUG_VISUALIZATION
  cv::imshow("cell_map", cell_map);
#endif


  std::vector<std::vector<cv::Point> > cells;
  cv::Mat cell_copy = cell_map.clone();
  correctThinWalls(cell_copy);	// just adds a few obstacle pixels to avoid merging independent segments
  cv::findContours(cell_copy, cells, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
#ifdef DEBUG_VISUALIZATION
//	 testing
//	cv::Mat black_map = cv::Mat(cell_map.rows, cell_map.cols, cell_map.type(), cv::Scalar(0));
//	for(size_t i=0; i<cells.size(); ++i)
//	{
//		cv::drawContours(black_map, cells, i, cv::Scalar(127), CV_FILLED);
//		cv::imshow("contours", black_map);
//		cv::waitKey();
//	}
#endif

  // create generalized Polygons out of the contours to handle the cells
  for(size_t cell=0; cell<cells.size(); ++cell)
  {
    if(cv::contourArea(cells[cell])>=min_cell_area)
    {
      CellPolygon current_cell(cells[cell], map_resolution);
      cell_polygons.push_back(current_cell);
    }
  }
}

void CellDecomposition::correctThinWalls(cv::Mat& room_map)
{
  for (int v=1; v<room_map.rows; ++v)
  {
    for (int u=1; u<room_map.cols; ++u)
    {
      if (room_map.at<uchar>(v-1,u-1)==255 && room_map.at<uchar>(v-1,u)==0 && room_map.at<uchar>(v,u-1)==0 && room_map.at<uchar>(v,u)==255)
        room_map.at<uchar>(v,u)=0;
      else if (room_map.at<uchar>(v-1,u-1)==0 && room_map.at<uchar>(v-1,u)==255 && room_map.at<uchar>(v,u-1)==255 && room_map.at<uchar>(v,u)==0)
        room_map.at<uchar>(v,u-1)=0;
    }
  }
}
