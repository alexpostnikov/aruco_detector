#include <iostream>
#include "aruco_marker_classes.h"
#include "aruco_parameters.h"
#include "aruco_utils.h"
//#include <opencv2/aruco.hpp>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>
#include "std_msgs/String.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <math.h>
#include <opencv2/calib3d.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <ros/console.h>
#include <ros/package.h>

using namespace std;
using namespace cv;

#define SAVE_TVEC_STATIC_MARKER_ID_0 0

void reallocate_corners(std::vector<Point2f> &corners);

cv::Point3d MarkerStatic::getAveragedStaticMarkerCenter(void)
{
  cv::Mat mean_mat;
  cv::reduce(centers_, mean_mat, 1, CV_REDUCE_AVG);
  cv::Point3f mean(mean_mat.at<double>(0, 0), mean_mat.at<double>(0, 1), mean_mat.at<double>(0, 2));
  return mean;
}

void MarkerStatic::calcCenter(cv::Mat cameraMatrix, cv::Mat distCoeffs)
{
  if (corners_.size() > 0)
  {
    std::vector<cv::Vec3d> rvec_vector, tvec_vector;
    std::vector<std::vector<cv::Point2f>> centers_vector;
    centers_vector.push_back(corners_);
    cv::aruco::estimatePoseSingleMarkers(centers_vector, marker_size_, cameraMatrix, distCoeffs, rvec_vector,
                                         tvec_vector);

    if (SAVE_TVEC_STATIC_MARKER_ID_0)
    {
      if (id_ == 0)
      {
        ofstream fileTvec;
        fileTvec.open(ros::package::getPath("aruco_detector") + "/src/test_data/tvec_1.txt",
                      fstream::app | fstream::out);
        fileTvec << tvec_vector[0][0] << " " << tvec_vector[0][1] << " " << tvec_vector[0][2] << endl;
        fileTvec.close();
      }
    }
    tvec_ = tvec_vector[0];
    rvec_ = rvec_vector[0];

    centers_.push_back(cv::Point3d(tvec_vector[0]));
    if (centers_.size() > max_len_of_centers_)
    {
      centers_.erase(centers_.begin());
    }
  }
}

MarkerCube::MarkerCube()
{
  ;
}

bool MarkerCube::isMineMarkerId(int id)
{
  for (uint8_t i = 0; i < (cube_side_qr_id_.size()); i++)
  {
    if (cube_side_qr_id_[i] == id)
    {
      return true;
    }
  }
  return false;
}

// calculates ONE average center from number of centers calculated by every seen marker
bool MarkerCube::calcCenter(cv::Mat cameraMatrix, cv::Mat distCoeffs, cv::Mat new_transform_from_cam_to_map,
                            ArucoDetectorParameters params)
{
  if (markers_.size() == 0)
    return false;

  Mat average_position;
  for (auto &marker : markers_)
  {
    std::vector<Point2f> marker_undist_corners;
    undistortPoints(marker.corners_, marker_undist_corners, cameraMatrix, distCoeffs, cv::noArray(), cameraMatrix);

    // TODO: constants? from param server?
    std::vector<Mat> corners_in_map_coords;
    std::cout << params.markers_heights[std::to_string(marker.id_)]<< std::endl;
    float z_goal = params.markers_heights[std::to_string(marker.id_)] - params.camera_position[2];
    float z_goal_bot = z_goal - params.width_marker;
    // float z_goal_sber_robots = 0.421 - params.camera_position[2];
    // float z_goal_enemy_robots = 0.507 - params.camera_position[2];
    // float z_goal_top_side_markers = 0.35 - params.camera_position[2];
    // float z_goal_top_side_enemys = 0.28 + 8 - params.camera_position[2];
    // float z_goal = 0, z_goal_bot = 0;
    // // SET Z_GOAL FOR CALCULUS (DEPENDING ON ENEMY\SBER_cube\TOP_SIDE )
    // // if enemy marker
    // if ((std::find(params.marker_ids["big_enemy_ids"].begin(), params.marker_ids["big_enemy_ids"].end(), marker.id_) !=
    //      params.marker_ids["big_enemy_ids"].end()) ||
    //     ((std::find(params.marker_ids["small_enemy_ids"].begin(), params.marker_ids["small_enemy_ids"].end(),
    //                 marker.id_) != params.marker_ids["small_enemy_ids"].end())))
    // {
    //   z_goal = z_goal_enemy_robots;
    //   // std::cout<< "z_goal "<< z_goal << " id " <<  marker.id_<<std::endl;
    //   z_goal_bot = z_goal - params.len_of_cube_markers;
    // }
    // // ------------------------------------------------------------------------------------

    // // if sber cube
    // if ((std::find(params.marker_ids["small_sber_cube_ids"].begin(), params.marker_ids["small_sber_cube_ids"].end(),
    //                marker.id_) != params.marker_ids["small_sber_cube_ids"].end()) ||
    //     ((std::find(params.marker_ids["big_sber_cube_ids"].begin(), params.marker_ids["big_sber_cube_ids"].end(),
    //                 marker.id_) != params.marker_ids["big_sber_cube_ids"].end())))
    // {
    //   z_goal = z_goal_sber_robots;
    //   z_goal_bot = z_goal - params.width_marker;
    // }
    // // ------------------------------------------------------------------------------------


    // //  top side markers
    // if ((std::find(params.marker_ids["small_top_side_ids"].begin(), params.marker_ids["small_top_side_ids"].end(),
    //                marker.id_) != params.marker_ids["small_top_side_ids"].end()) ||
    //     ((std::find(params.marker_ids["big_top_side_ids"].begin(), params.marker_ids["big_top_side_ids"].end(),
    //                 marker.id_) != params.marker_ids["big_top_side_ids"].end())))
    // {
    //   z_goal = z_goal_top_side_markers;
    // }
    // // ------------------------------------------------------------------------------------

    
    std::vector<int> top_corners_ids_to_use = { 0, 1 };
    std::vector<int> bottom_corners_ids_to_use = { 3, 2 };

    // 

    // set corners id to be used for calculus of position
    if (std::find(params.markers_orientation["clockwise"].begin(), params.markers_orientation["clockwise"].end(),
                  marker.id_) != params.markers_orientation["clockwise"].end())
    {
      top_corners_ids_to_use = { 1, 2 };
      bottom_corners_ids_to_use = { 0, 3 };
    }
    if (std::find(params.markers_orientation["upSide"].begin(), params.markers_orientation["upSide"].end(),
                  marker.id_) != params.markers_orientation["upSide"].end())
    {
      top_corners_ids_to_use = { 2, 3 };
      bottom_corners_ids_to_use = { 1, 0 };
    }
    if (std::find(params.markers_orientation["counterClockwise"].begin(),
                  params.markers_orientation["counterClockwise"].end(),
                  marker.id_) != params.markers_orientation["counterClockwise"].end())
    {
      top_corners_ids_to_use = { 3, 0 };
      bottom_corners_ids_to_use = { 2, 1 };
    }
    // ---------------------------------------------------------------

    
    if (params.use_top_line_marker)
    {
      for (auto corner_id : top_corners_ids_to_use)
      {
        Mat corner_mat =
            (Mat_<double>(3, 1) << marker_undist_corners[corner_id].x, marker_undist_corners[corner_id].y, 1);
        corner_mat = cameraMatrix.inv() * corner_mat;
        move_and_norm_vector(corner_mat, new_transform_from_cam_to_map, params.camera_position, z_goal);
        corners_in_map_coords.push_back(corner_mat);
      }
    }
    if (params.use_bottom_line_marker)
    {
      for (auto corner_id : bottom_corners_ids_to_use)
      {
        Mat corner_mat =
            (Mat_<double>(3, 1) << marker_undist_corners[corner_id].x, marker_undist_corners[corner_id].y, 1);
        corner_mat = cameraMatrix.inv() * corner_mat;
        move_and_norm_vector(corner_mat, new_transform_from_cam_to_map, params.camera_position, z_goal_bot);
        corners_in_map_coords.push_back(corner_mat);
      }
    }

    // calc average from all corners to get real marker center
    float angle = 0;
  
    if (corners_in_map_coords.size() > 2)
    {
      average_position =
          (Mat_<double>(2, 1) << (corners_in_map_coords[0].at<double>(0, 0) +
                                  corners_in_map_coords[1].at<double>(0, 0)) /
                                      2,
            (corners_in_map_coords[0].at<double>(1, 0) + corners_in_map_coords[1].at<double>(1, 0)) / 2);
      cv::Vec3d first_tvec = cv::Vec3d(average_position.at<double>(0, 0), average_position.at<double>(1, 0), 0);
      // marker.setTvec(cv::Vec3d(average_position.at<double>(0, 0), average_position.at<double>(1, 0), 0));
      float dot = corners_in_map_coords[0].at<double>(0, 0) - corners_in_map_coords[1].at<double>(0, 0);
      float det = corners_in_map_coords[0].at<double>(1, 0) - corners_in_map_coords[1].at<double>(1, 0);
      float first_angle = atan2(dot, det);

      average_position =
          (Mat_<double>(2, 1) << (corners_in_map_coords[2].at<double>(0, 0) +
                                  corners_in_map_coords[3].at<double>(0, 0)) /
                                      2,
            (corners_in_map_coords[2].at<double>(1, 0) + corners_in_map_coords[3].at<double>(1, 0)) / 2);
      cv::Vec3d second_tvec = cv::Vec3d(average_position.at<double>(0, 0), average_position.at<double>(1, 0), 0);
      
      dot = corners_in_map_coords[2].at<double>(0, 0) - corners_in_map_coords[3].at<double>(0, 0);
      det = corners_in_map_coords[2].at<double>(1, 0) - corners_in_map_coords[3].at<double>(1, 0);
      float second_angle = atan2(dot, det);
      cv::Vec3d result_tvec = cv::Vec3d((first_tvec[0]+second_tvec[0])/2.0, (first_tvec[1]+second_tvec[1])/2.0, 0);
      marker.setTvec(result_tvec);
      angle = (first_angle + second_angle)/2.0;
    }

    if (corners_in_map_coords.size() == 2)
    {
      average_position = (Mat_<double>(2, 1) << (corners_in_map_coords[0].at<double>(0, 0) +
                                                 corners_in_map_coords[1].at<double>(0, 0)) /
                                                    2,
                          (corners_in_map_coords[0].at<double>(1, 0) + corners_in_map_coords[1].at<double>(1, 0)) / 2);
      marker.setTvec(cv::Vec3d(average_position.at<double>(0, 0), average_position.at<double>(1, 0), 0));
      float dot = corners_in_map_coords[0].at<double>(0, 0) - corners_in_map_coords[1].at<double>(0, 0);
      float det = corners_in_map_coords[0].at<double>(1, 0) - corners_in_map_coords[1].at<double>(1, 0);
      angle = atan2(dot, det);
    }
    
    Vec3d angle_vec(0.0, 0.0, -(angle - 1.57));
    angle_vec[2] += params.markers_poses[std::to_string(marker.id_)][2];
    marker.setRvec(angle_vec);
    center_ = calcCenterOfRobot_by_disp(angle_vec, marker.getTvec(), params.markers_poses[std::to_string(marker.id_)]);
    ;
    // --------------------------------------------------------------------- //
    // if ((marker.id_ == 2))
    // {
    //   // Vec3d angle_vec(0.0, 0.0, -(angle - 3.14));
    //   // marker.setRvec(angle_vec);
    //   center_ = calcCenterOfRobot_by_disp(angle_vec, marker.getTvec(), Vec3d(-0.03, -0.07, 0));
    // }

    // else if ((marker.id_ == 8))
    // {
    //   Vec3d angle_vec(0.0, 0.0, -(angle - 3.14));
    //   marker.setRvec(angle_vec);
    //   center_ = calcCenterOfRobot_by_disp(angle_vec, marker.getTvec(), Vec3d(0.02, 0.07, 0));
    // }

    // else if ((marker.id_ == 6))
    // {
    //   Vec3d angle_vec(0.0, 0.0, -(angle - 3.14));
    //   marker.setRvec(angle_vec);
    //   center_ = calcCenterOfRobot_by_disp(angle_vec, marker.getTvec(), Vec3d(0.02, -0.07, 0));
    // }

    // else if ((marker.id_ == 1))
    // {
    //   Vec3d angle_vec(0.0, 0.0, (angle - 3.14));
    //   marker.setRvec(angle_vec);
    //   center_ = calcCenterOfRobot_by_disp(angle_vec, marker.getTvec(), Vec3d(0.07, -0.02, 0));
    // }

    // else if (marker.id_ == 3)
    // {
    //   Vec3d angle_vec(0.0, 0.0, -(angle - 3.14));
    //   marker.setRvec(angle_vec);
    //   center_ = calcCenterOfRobot_by_disp(angle_vec, marker.getTvec(), Vec3d(0.09, 0, 0));
    // }
    // else if (marker.id_ == 4)

    // {
    //   Vec3d angle_vec(0.0, 0.0, -(angle - 3.14));

    //   marker.setRvec(angle_vec);

    //   center_ = calcCenterOfRobot_by_disp(angle_vec, marker.getTvec(), Vec3d(0.03, 0., 0));
    // }

    // else if (marker.id_ == 45)
    // {
    //   Vec3d angle_vec(0.0, 0.0, (angle + 3.14 / 2.0));

    //   marker.setRvec(angle_vec);

    //   center_ = calcCenterOfRobot_by_disp(angle_vec, marker.getTvec(), Vec3d(0.03, 0., 0));
    // }

    // else if (marker.id_ == 41)
    // {
    //   Vec3d angle_vec(0.0, 0.0, (angle + 3.14 / 2.0));

    //   marker.setRvec(angle_vec);

    //   center_ = calcCenterOfRobot_by_disp(angle_vec, marker.getTvec(), Vec3d(0.03, 0., 0));
    // }

    // else if (marker.id_ == 31)
    // {
    //   Vec3d angle_vec(0.0, 0.0, (angle + 3.14 / 2.0));

    //   marker.setRvec(angle_vec);

    //   center_ = calcCenterOfRobot_by_disp(angle_vec, marker.getTvec(), Vec3d(0.03, 0., 0));
    // }

    // else
    // {
    //   center_ = calcCenterOfCube(angle_vec, marker.getTvec());
    // }
    // --------------------------------------------------------------- //
  }
  return true;
}
