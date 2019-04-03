#include <ros/console.h>
#include <ros/package.h>

#include "aruco_parameters.h"
#include "aruco_scene.h"
#include "aruco_utils.h"
#include "ros/ros.h"
#include "ros/ros.h"
#include "stdint.h"

#include <tf/transform_broadcaster.h>
#include <chrono>
#include <string>

namespace enc = sensor_msgs::image_encodings;

SceneHolder::SceneHolder()
{
  throw(std::unexpected);
}

SceneHolder::SceneHolder(ArucoDetectorParameters arucoParams)
{
  params_ = arucoParams;
  // read from parameters number of staticMarkers_
  for (uint8_t i = 0; i < params_.number_of_static_markers; i++)
  {
    MarkerStatic static_marker(params_.static_markers_ids[i], params_.static_marker_sizes);
    static_markers_.push_back(static_marker);
  }

  for (auto &arucoCube : arucoCubes_)
  {
    arucoCube.board_type = cv::aruco::PREDEFINED_DICTIONARY_NAME(params_.used_board_type);
    for (auto &marker : arucoCube.markers_)
    {
      marker.marker_size_ = params_.len_of_cube_markers;

    }
  }
  arucoCubes_[0].cube_side_qr_id_ = params_.big_sber_ids;
  arucoCubes_[1].cube_side_qr_id_ = params_.small_sber_ids;
  arucoCubes_[1].cube_side_qr_id_.insert(arucoCubes_[1].cube_side_qr_id_.end(), params_.small_top_side_ids.begin(), params_.small_top_side_ids.end() );
  arucoCubes_[2].cube_side_qr_id_ = params_.big_enemy_ids;
  arucoCubes_[3].cube_side_qr_id_ = params_.small_enemy_ids;

  dictionary_ = cv::aruco::getPredefinedDictionary(params_.used_board_type);
  /** load detector params from yml file**/
  detectorParams_ = cv::aruco::DetectorParameters::create();

  detectorParams_->adaptiveThreshWinSizeMin = 3;
  detectorParams_->adaptiveThreshWinSizeMax = 23;
  detectorParams_->adaptiveThreshWinSizeStep = 13;
  detectorParams_->adaptiveThreshConstant = 15;
  detectorParams_->perspectiveRemovePixelPerCell = 15;
  detectorParams_->perspectiveRemoveIgnoredMarginPerCell = 0.33;
  detectorParams_->polygonalApproxAccuracyRate = 0.1;
}

bool SceneHolder::recalibrate_cb(aruco_detector::ArucoRecalibrate::Request &req, aruco_detector::ArucoRecalibrate::Response &res)
{
  std::cout<< req.side << std::endl;
  this->scene_status_ = Scene_statuses::search_static;
  res.result = 1;
  return 1;
}

bool SceneHolder::setSide_cb(aruco_detector::SetSide::Request &req, aruco_detector::SetSide::Response &res)
{
  std::cout<< req.side << std::endl;
  if ((req.side == "g") || (req.side == "G")) //  || (req.side == std::string("green"))
    { 
      std::cout << "changing to green side" << std::endl;
      this->params_.camera_position = this->params_.camera_position_green;
      res.result = 1;
      return 1;
    }
  else if ((req.side == "o") || (req.side == "O")) // || (req.side == std::string("orange"))
    {
      std::cout << "changing to orange side" << std::endl;
      this->params_.camera_position = this->params_.camera_position_orange;
      res.result = 1;
      return 1;
      // aruco_params.camera_position = aruco_params.camera_position_orange;
    }
  res.result = 0; 
  return 0;
  
}


void SceneHolder::imageCallback(const sensor_msgs::ImageConstPtr &original_image)
{
  try
  {
    this->cv_ptr_ = cv_bridge::toCvCopy(original_image, enc::MONO8);
    if (inverseImage_)
    {
      this->cv_ptr_->image = cv::Scalar::all(255) - this->cv_ptr_->image;
      // GaussianBlur(this->cv_ptr_->image, this->cv_ptr_->image, Size(3,3),0,0);
    }
    rotate(this->cv_ptr_->image, this->cv_ptr_->image, 1); 
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Could not convert from '%s' to 'MONO8'.", original_image->encoding.c_str());
  }

  if (scene_status_ == Scene_statuses::search_cubes)
  {
    preProcessImageBrighnessMask(this->cv_ptr_->image, cropped_scene_.areas_, cropped_scene_.candidates_,
                                 params_.brightness_threshold);
  }
  timestamp_ = original_image->header.stamp;
}

void SceneHolder::loadCameraParams()
{
  bool result_reading_camera_params =
      readCameraParameters(ros::package::getPath("aruco_detector") + "/camera_params.yml", SceneHolder::camera_matrix_,
                           SceneHolder::dist_coeffs_);
  if (result_reading_camera_params == false)
    ROS_ERROR("Could not open camera parasm file ");  //: '%s'",
                                                      // ros::package::getPath("aruco_detector")+"/camera_params.yml");
}

// find markers and save pixels of corners and ids
void SceneHolder::findMarkers()
{
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<cv::Vec3d> rvec, tvec;

  if (scene_status_ == Scene_statuses::search_cubes)
  {
    // cout <<" cropped_scene_.candidates_.size() "<< cropped_scene_.candidates_.size() << endl;
    for (uint8_t i = 0; i < cropped_scene_.candidates_.size(); i++)
    {
      std::vector<std::vector<cv::Point2f>> local_corners;
      std::vector<int> local_ids;

      detectMarkers(cropped_scene_.candidates_[i], dictionary_, local_corners, local_ids, detectorParams_);

      Rect local_rect = cropped_scene_.areas_[i];
      for (auto &local_corner : local_corners)
      {
        std::transform(local_corner.begin(), local_corner.end(), local_corner.begin(),
                       [local_rect](Point2f local_point) {
                         return Point2f(local_point.x + local_rect.x, local_point.y + local_rect.y);
                       });
        corners.push_back(local_corner);
      }
      for (auto id : local_ids)
      {
        ids.push_back(id);
      }
    }
  }

  else
  {
    detectMarkers(cv_ptr_->image, dictionary_, corners, ids, detectorParams_);
  }

  for (uint8_t i = 0; i < ids.size(); i++)
  {
    Marker new_marker(ids[i]);
    new_marker.set_corners(corners[i]);
    found_markers_.push_back(new_marker);
  }
}

void SceneHolder::allocateStaticMarkers()
{
  if (scene_status_ != Scene_statuses::search_static)
  {
    return;
  }

  if (found_markers_.size() > 0)
  {
    if (static_markers_.size() == 0)
    {
      ROS_ERROR("!!static_markers_.size() == 0!!");
    }
    for (uint8_t i = 0; i < found_markers_.size(); i++)
    {
      for (auto &static_marker : static_markers_)
      {
        int markerId = static_marker.getId();
        // std::cout << " markerId " << markerId <<std::endl;
        if (markerId == found_markers_[i].getId())
        {
          static_marker.is_visible_ = true;
          static_marker.addNewMarker(found_markers_[i].corners_);
          numb_visible_static_markers_++;
        }
      }
    }
  }
}

void SceneHolder::allocateCubes()
{
  if (scene_status_ == Scene_statuses::search_static)
  {
    return;
  }
  if (found_markers_.size() < 0)
  {
    // SET cubes as non-visible?
    return;
  }

  // for every marker cube check every found marker, if marker id correct add it to inner buffer.

  for (auto &marker_cube : arucoCubes_)
  {
    for (uint8_t i = 0; i < found_markers_.size(); i++)
    {
      if (marker_cube.isMineMarkerId(found_markers_[i].getId()))
      {
        marker_cube.is_visible_ = true;
        // cout << " adding cube marker with id " << found_markers_[i].getId() << endl ;
        marker_cube.markers_.push_back(found_markers_[i]);
      }
    }
  }
}

void SceneHolder::calcAllCenters()
{
  for (auto &static_marker : static_markers_)
  {
    static_marker.calcCenter(camera_matrix_, dist_coeffs_);
  }
  for (auto &marker_cube : arucoCubes_)
  {
    if (scene_status_ == Scene_statuses::search_cubes)
      marker_cube.calcCenter(camera_matrix_, dist_coeffs_, new_transform_from_cam_to_map_, params_);
  }
}

void SceneHolder::showImages(bool is_draw_markers, bool is_draw_table_perspective, bool is_show_undist)
{
  static bool showInMapCoords = false;
  if (is_draw_markers)
  {
    cv::Scalar Color = cv::Scalar(100, 0, 0);
    std::ostringstream oss;
    std::vector<std::vector<cv::Point2f>> corners_vector;
    std::vector<int> ids_vector;
    Mat coords;
    Mat coordinates_in_Map_base;
    cvtColor(cv_ptr_->image, cv_ptr_->image, CV_GRAY2RGB);

    //////////////// SHOW STATIC MARKERS///////////////////
    for (auto static_marker : static_markers_)
    {
      if (static_marker.is_visible_)
      {
        corners_vector.push_back(static_marker.corners_);
        ids_vector.push_back(static_marker.id_);
        if (!showInMapCoords)
        {
          coords = cv::Mat(static_marker.tvec_);
          Color = cv::Scalar(0, 0, 255);
        }
        else
        {
          if (transform_from_cam_to_map_.cols != 4)
          {
            showInMapCoords = !showInMapCoords;
            return;
          }
        }

        oss << std::setprecision(3) << coords.at<double>(0, 0) << " " << coords.at<double>(1, 0) << " "
            << coords.at<double>(2, 0);

        putText(cv_ptr_->image, oss.str(), static_marker.corners_[0], FONT_HERSHEY_SIMPLEX, 0.5, Color, 0);
        oss.str("");
        oss.clear();
      }
    }
    /////////////////////////SHOW CAM COORDS/////////////////////

    oss << "camera position: " << params_.camera_position[0] << " " << params_.camera_position[1] << " "
        << params_.camera_position[2];
    putText(cv_ptr_->image, oss.str(), Point(100, 100), FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(222, 100, 0), 0);
    oss.str("");
    oss.clear();

    //////////////// SHOW CUBES///////////////////
    for (auto cube_marker : arucoCubes_)
    {
      for (auto marker : cube_marker.markers_)
      {
        corners_vector.push_back(marker.corners_);
        ids_vector.push_back(marker.id_);
        coords = cv::Mat(marker.tvec_);
        oss << std::setprecision(3) << coords.at<double>(0, 0) << " " << coords.at<double>(1, 0) << " "
            << coords.at<double>(2, 0);
        putText(cv_ptr_->image, oss.str(), cube_marker.markers_[0].corners_[0], FONT_HERSHEY_SIMPLEX, 0.5, Color, 0);
        for (uint i = 0; i < marker.corners_.size(); i++)
        {
          Rect around_corner(marker.corners_[i].x - 5, marker.corners_[i].y - 5, 10, 10);
          cv_ptr_->image(around_corner) = cv::Scalar(i * 50);
        }
        oss.str("");
        oss.clear();
      }
    }
    cv::aruco::drawDetectedMarkers(cv_ptr_->image, corners_vector, ids_vector);
  }

  
  if ((new_transform_from_cam_to_map_.cols == 4) && (is_draw_table_perspective))
  {
    // ROS_WARN_STREAM_THROTTLE(1, "inside");
    ShowTablePerspevtive();
  }

  Mat image_resized;

  resize(cv_ptr_->image, image_resized, Size(500, 300));
  imshow("video", image_resized);

  char key = (char)waitKey(50);
  if (key == 'd')
  {
    showInMapCoords = !showInMapCoords;
  }
}

void SceneHolder::calc_new_transform()
{
  static int counter_fails_static_detection =
      0;  // counter that increments if one of static  markers in pictureis epsent
  if (scene_status_ != Scene_statuses::search_static)
  {
    return;
  }

  if (counter_fails_static_detection > params_.max_missed_static_markers_)  //
  {
    ROS_WARN_STREAM(" !!failed to detect all static markers, switching to cubefinder alg!! ");
    calibration_status_ = false;
    counter_fails_static_detection = 0;
    for (auto static_marker : static_markers_)
    {
      static_marker.centers_.clear();
      static_marker.centers_.resize(0);
      calibration_status_ = false;
    }
    scene_status_ = Scene_statuses::search_cubes;
    return;
  }

  for (auto static_marker : static_markers_)
  {
    if (!static_marker.is_visible_)
    {
      counter_fails_static_detection++;
      readCamToMapTransform(ros::package::getPath("aruco_detector") + "/new_transform_from_cam_to_map.yml",
                            new_transform_from_cam_to_map_);
      return;
    }
    // if ((static_marker.id_ == 5) && (!static_marker.is_visible_))
    // {
    //   counter_fails_static_detection++;
    //   readCamToMapTransform(ros::package::getPath("aruco_detector") + "/new_transform_from_cam_to_map.yml",
    //                         new_transform_from_cam_to_map_);
    //   return;
    // }
    // if ((static_marker.id_ == 0) && (!static_marker.is_visible_))
    // {
    //   counter_fails_static_detection++;
    //   readCamToMapTransform(ros::package::getPath("aruco_detector") + "/new_transform_from_cam_to_map.yml",
    //                         new_transform_from_cam_to_map_);
    //   return;
    // }
  }

  vector<cv::Vec3d> PointCameraBase;
  PointCameraBase.resize(2);

  for (auto static_marker : static_markers_)
  {
    if ((static_marker.is_visible_) && static_marker.id_ == params_.static_markers_ids[0])
      PointCameraBase[0] = static_marker.tvec_;
    if ((static_marker.is_visible_) && static_marker.id_ == params_.static_markers_ids[1])
      PointCameraBase[1] = static_marker.tvec_;
  }

  scene_status_ = Scene_statuses::search_cubes;
  calibration_status_ = true;
  counter_fails_static_detection = 0;
  ROS_WARN_STREAM_ONCE("all static markers visible, calculating new transform");
  Mat rotation = new_allocation_maps(Point3d(PointCameraBase[0][0], PointCameraBase[0][1], PointCameraBase[0][2]),
                                     Point3d(PointCameraBase[1][0], PointCameraBase[1][1], PointCameraBase[1][2]),
                                     params_.camera_position);
  Mat new_transform(3, 4, CV_64F);
  Rect r(0, 0, 3, 3);
  Mat dstroi = new_transform(r);
  rotation.convertTo(dstroi, dstroi.type(), 1, 0);

  new_transform.at<double>(0, 3) = params_.camera_position[0];
  new_transform.at<double>(1, 3) = params_.camera_position[1];
  new_transform.at<double>(2, 3) = params_.camera_position[2];
  new_transform_from_cam_to_map_ = new_transform;
  saveCamToMapTransform(ros::package::getPath("aruco_detector") + "/new_transform_from_cam_to_map.yml",
                        new_transform_from_cam_to_map_);
}

void SceneHolder::calcTransform()
{
  static int counter_fails_static_detection = 0;
  if (scene_status_ != Scene_statuses::search_static)
  {
    return;
  }

  vector<cv::Point3f> PointCameraBase;
  for (auto static_marker : static_markers_)
  {
    if (!static_marker.is_visible_)
    {
      ROS_ERROR_STREAM("static_marker id: " << static_marker.id_ << " is unvisible ");
      counter_fails_static_detection++;
      if (counter_fails_static_detection > 3)  // TODO add timer here!
      {
        ROS_WARN_STREAM(" !!failed to detect all static markers, switching to cubefinder alg!! ");
        counter_fails_static_detection = 0;
        for (auto static_marker : static_markers_)
        {
          static_marker.centers_.clear();
          static_marker.centers_.resize(0);
          calibration_status_ = false;
        }
        scene_status_ = Scene_statuses::search_cubes;
        return;
      }
      readCamToMapTransform(ros::package::getPath("aruco_detector") + "/transform_from_cam_to_map.yml",
                            transform_from_cam_to_map_);
      return;
    }

    ROS_INFO_STREAM("static_marker id: " << static_marker.id_ << " is visible ");
    PointCameraBase.push_back(static_marker.getAveragedStaticMarkerCenter());
  }

  std::vector<Vec3d> marker_pose;
  std::vector<Point2d> marker_pose_pixels;
  marker_pose.resize(2);
  marker_pose_pixels.resize(2);
  for (auto static_marker : static_markers_)
  {
    if ((static_marker.is_visible_) && static_marker.id_ == 5)
      marker_pose_pixels[0] = static_marker.corners_[0];
    if ((static_marker.is_visible_) && static_marker.id_ == 0)
      marker_pose_pixels[1] = static_marker.corners_[1];
  }
  //
  for (auto static_marker : static_markers_)
  {
    if ((static_marker.is_visible_) && static_marker.id_ == 5)
      marker_pose[0] = static_marker.tvec_;
    if ((static_marker.is_visible_) && static_marker.id_ == 0)
      marker_pose[1] = static_marker.tvec_;
  }

  PointCameraBase.push_back(Point3f(0, 0, 0));

  vector<cv::Point3f> input_arr;

  input_arr.push_back(cv::Point3f(0.15, 0.15, 0.12));  // FIFTH
  input_arr.push_back(cv::Point3f(2.85, 0.15, 0.12));  // NULLTH
                                                       // input_arr.push_back(cv::Point3f (0.62, 1.2,  0.0)); // SECOND
  input_arr.push_back(cv::Point3f(0.62, 0.10, 0.45));  // THIRD
  // input_arr.push_back(cv::Point3f (1.5,  1.5, 0.0));   // FIRST
  input_arr.push_back(cv::Point3f(2.38, 1.64, 0.0));  // SIXTH
  input_arr.push_back(
      cv::Point3f(params_.camera_position[0], params_.camera_position[1], params_.camera_position[2]));  // CAM
  // ROS_ERROR_STREAM(params_.camera_position[0] <<"  " <<  params_.camera_position[1]<<" "<<
  // params_.camera_position[2]);
  std::vector<uchar> inliers;

  if (PointCameraBase.size() != input_arr.size())
  {
    std::cout << "in calculating bases transform: PointCameraBase.size() != input_arr.size()" << endl;
    std::cout << "PointCameraBase size is " << PointCameraBase.size() << endl;
    return;
  }

  cv::estimateAffine3D(PointCameraBase, input_arr, transform_from_cam_to_map_, inliers);
  cout << "transform_from_cam_to_map_ " << transform_from_cam_to_map_ << endl;
  for (auto static_marker : static_markers_)
  {
    if (static_marker.centers_.size() != static_marker.max_len_of_centers_)
    {
      return;
    }
  }

  ROS_WARN_STREAM_ONCE("all static markers visible, calculating new transform");
  counter_fails_static_detection = 0;
  for (auto static_marker : static_markers_)
  {
    static_marker.centers_.clear();
    static_marker.centers_.resize(0);
    calibration_status_ = true;
  }
  scene_status_ = Scene_statuses::search_cubes;
  saveCamToMapTransform(ros::package::getPath("aruco_detector") + "/transform_from_cam_to_map.yml",
                        transform_from_cam_to_map_);
}

void SceneHolder::showStatistics()
{
  int number_of_static_markers_seen = 0;
  int number_of_cubes_seen = 0;
  for (auto static_marker : static_markers_)
  {
    if (static_marker.is_visible_)
    {
      number_of_static_markers_seen++;
      std::cout << "static marker id " << static_marker.id_ << "  with center at " << (static_marker.tvec_)
                << std::endl;
    }
  }

  for (auto marker_cube : arucoCubes_)
  {
    if (marker_cube.is_visible_)
    {
      number_of_cubes_seen++;
      std::cout << "marker_cube id " << marker_cube.markers_[0].id_ << "  with center at " << (marker_cube.center_)
                << std::endl;
    }
  }

  std::cout << "number_of_static_markers_seen " << number_of_static_markers_seen << " number_of_cubes_seen "
            << number_of_cubes_seen << std::endl;
}

void SceneHolder::publish(ros::Publisher sber_robot_pose1, ros::Publisher sber_robot_pose2,
                          ros::Publisher enemy_robot_pose1, ros::Publisher enemy_robot_pose2)
{
  if (scene_status_ != Scene_statuses::search_cubes)
  {
    return;
  }
  geometry_msgs::PoseStamped robot_pose_msg[4];

  for (auto &msg : robot_pose_msg)
  {
    msg.header.frame_id = "map";
    msg.header.stamp = timestamp_;
  }
  
  if ((arucoCubes_[0].markers_.size() != 0))
  {

    robot_pose_msg[0] = calc_msg_from_position(arucoCubes_[0], new_transform_from_cam_to_map_, true);
    sber_robot_pose1.publish(robot_pose_msg[0]);
  }
  
  if ((arucoCubes_[1].markers_.size() != 0))
  {
    robot_pose_msg[1] = calc_msg_from_position(arucoCubes_[1], new_transform_from_cam_to_map_, true);
    sber_robot_pose2.publish(robot_pose_msg[1]);
  }
  if ((arucoCubes_[2].markers_.size() != 0))
  {
    robot_pose_msg[2] = calc_msg_from_position(arucoCubes_[2], new_transform_from_cam_to_map_, true);
    enemy_robot_pose1.publish(robot_pose_msg[2]);
  }
  
  if ((arucoCubes_[3].markers_.size() != 0))
  {
    robot_pose_msg[3] = calc_msg_from_position(arucoCubes_[3], new_transform_from_cam_to_map_, true);
    enemy_robot_pose2.publish(robot_pose_msg[3]);

  }
}
  
//   for (int i = 0; i < 4; i++)
//   {
//     {
//       if ((arucoCubes_[i].markers_.size() == 0))
//       {
//         continue;
//         // robot_pose_msg[i].pose.position.x = std::numeric_limits<double>::infinity();
//         // robot_pose_msg[i].pose.position.y = std::numeric_limits<double>::infinity();
//         // robot_pose_msg[i].pose.position.z = std::numeric_limits<double>::infinity();
//         // robot_pose_msg[i].pose.orientation.w = 1;
//       }
//       else
//       {
//         robot_pose_msg[i] = calc_msg_from_position(arucoCubes_[i], new_transform_from_cam_to_map_, true);
//       }
//     }
//   }
//   sber_robot_pose1.publish(robot_pose_msg[0]);
//   sber_robot_pose2.publish(robot_pose_msg[1]);
//   enemy_robot_pose1.publish(robot_pose_msg[2]);
//   enemy_robot_pose2.publish(robot_pose_msg[3]);
// }

void SceneHolder::ShowTablePerspevtive()
{
  Mat image = cv_ptr_->image.clone();
  vector<Point3f> PointsInMapCoordsSystem;

  vector<Point2f> PointsInPixels;

  if (new_transform_from_cam_to_map_.cols != 4)
    return;
  Affine3d aff(new_transform_from_cam_to_map_);
  Affine3d affInv = aff.inv();
  vector<Vec3f> PointsInCamCoordsSystem;
  vector<Point3f> zeroPoint;
  vector<Point2f> vecPointsInPixels;
  zeroPoint.push_back(Point3f(0, 0, 0));

  Scalar color = Scalar(0, 255, 0);
  for (int z = 0; z < 5; z++)
  {
    PointsInCamCoordsSystem.clear();
    vecPointsInPixels.clear();
    float z_coord = 0.1 * z;
    if (z > 0.01)
      color = Scalar(255, 0, 0);
    PointsInCamCoordsSystem.push_back(Vec3f(affInv * Point3f(3, 1.0, z_coord)));
    PointsInCamCoordsSystem.push_back(Vec3f(affInv * Point3f(3.0, 0.0, z_coord)));
    PointsInCamCoordsSystem.push_back(Vec3f(affInv * Point3f(2.0, 0.0, z_coord)));
    PointsInCamCoordsSystem.push_back(Vec3f(affInv * Point3f(1.0, 0.0, z_coord)));
    PointsInCamCoordsSystem.push_back(Vec3f(affInv * Point3f(0.0, 0.0, z_coord)));
    PointsInCamCoordsSystem.push_back(Vec3f(affInv * Point3f(0.0, 1.0, z_coord)));
    for (uint8_t i = 0; i < PointsInCamCoordsSystem.size(); i++)
    {
      projectPoints(zeroPoint, Vec3f(0, 0, 0), PointsInCamCoordsSystem[i], camera_matrix_, dist_coeffs_,
                    PointsInPixels);
      circle(image, PointsInPixels[0], i + 5, Scalar(0, 0, 255));
      vecPointsInPixels.push_back(PointsInPixels[0]);
    }
    line(image, vecPointsInPixels[0], vecPointsInPixels[1], color, 2);
    line(image, vecPointsInPixels[1], vecPointsInPixels[2], color, 2);
    line(image, vecPointsInPixels[2], vecPointsInPixels[3], color, 2);
    line(image, vecPointsInPixels[3], vecPointsInPixels[4], color, 2);
    line(image, vecPointsInPixels[4], vecPointsInPixels[5], color, 2);
  }

  // Draw right recuperator
  PointsInCamCoordsSystem.clear();
  vecPointsInPixels.clear();

  PointsInCamCoordsSystem.push_back(Vec3f(affInv * Point3f(0.61, 0.1, 0)));
  projectPoints(zeroPoint, Vec3f(0, 0, 0), PointsInCamCoordsSystem[0], camera_matrix_, dist_coeffs_, PointsInPixels);
  vecPointsInPixels.push_back(PointsInPixels[0]);

  PointsInCamCoordsSystem.push_back(Vec3f(affInv * Point3f(0.61, 0.1, 0.4)));
  projectPoints(zeroPoint, Vec3f(0, 0, 0), PointsInCamCoordsSystem[1], camera_matrix_, dist_coeffs_, PointsInPixels);
  vecPointsInPixels.push_back(PointsInPixels[0]);

  line(image, vecPointsInPixels[0], vecPointsInPixels[1], color, 2);

  // Draw left recuperator

  PointsInCamCoordsSystem.clear();
  vecPointsInPixels.clear();

  PointsInCamCoordsSystem.push_back(Vec3f(affInv * Point3f(2.39, 0.1, 0)));
  projectPoints(zeroPoint, Vec3f(0, 0, 0), PointsInCamCoordsSystem[0], camera_matrix_, dist_coeffs_, PointsInPixels);
  vecPointsInPixels.push_back(PointsInPixels[0]);

  PointsInCamCoordsSystem.push_back(Vec3f(affInv * Point3f(2.39, 0.1, 0.4)));
  projectPoints(zeroPoint, Vec3f(0, 0, 0), PointsInCamCoordsSystem[1], camera_matrix_, dist_coeffs_, PointsInPixels);
  vecPointsInPixels.push_back(PointsInPixels[0]);

  line(image, vecPointsInPixels[0], vecPointsInPixels[1], color, 2);

  // Draw line (the one is close to central tracking device)
  PointsInCamCoordsSystem.clear();
  vecPointsInPixels.clear();

  PointsInCamCoordsSystem.push_back(Vec3f(affInv * Point3f(2.39, 0.46, 0)));
  projectPoints(zeroPoint, Vec3f(0, 0, 0), PointsInCamCoordsSystem[0], camera_matrix_, dist_coeffs_, PointsInPixels);
  vecPointsInPixels.push_back(PointsInPixels[0]);

  PointsInCamCoordsSystem.push_back(Vec3f(affInv * Point3f(2.39, 1.65, 0)));
  projectPoints(zeroPoint, Vec3f(0, 0, 0), PointsInCamCoordsSystem[1], camera_matrix_, dist_coeffs_, PointsInPixels);

  vecPointsInPixels.push_back(PointsInPixels[0]);

  PointsInCamCoordsSystem.push_back(Vec3f(affInv * Point3f(1.5, 1.65, 0)));
  projectPoints(zeroPoint, Vec3f(0, 0, 0), PointsInCamCoordsSystem[2], camera_matrix_, dist_coeffs_, PointsInPixels);
  vecPointsInPixels.push_back(PointsInPixels[0]);

  PointsInCamCoordsSystem.push_back(Vec3f(affInv * Point3f(0.61, 1.65, 0)));
  projectPoints(zeroPoint, Vec3f(0, 0, 0), PointsInCamCoordsSystem[3], camera_matrix_, dist_coeffs_, PointsInPixels);
  vecPointsInPixels.push_back(PointsInPixels[0]);

  PointsInCamCoordsSystem.push_back(Vec3f(affInv * Point3f(0.61, 0.46, 0)));
  projectPoints(zeroPoint, Vec3f(0, 0, 0), PointsInCamCoordsSystem[4], camera_matrix_, dist_coeffs_, PointsInPixels);
  vecPointsInPixels.push_back(PointsInPixels[0]);

  color = Scalar(255, 255, 0);
  line(image, vecPointsInPixels[0], vecPointsInPixels[1], color, 2);
  line(image, vecPointsInPixels[1], vecPointsInPixels[2], color, 2);
  line(image, vecPointsInPixels[2], vecPointsInPixels[3], color, 2);
  line(image, vecPointsInPixels[3], vecPointsInPixels[4], color, 2);
  resize(image,image, Size(), 0.3, 0.3, INTER_CUBIC);
  imshow("table Perspective", image);
}

void SceneHolder::clearOldData()
{
  found_markers_.clear();
  numb_visible_static_markers_ = 0;

  for (auto &static_marker : static_markers_)
    static_marker.clearOldData();

  for (auto &aruco_cube : arucoCubes_)
    aruco_cube.clearOldData();

  cropped_scene_.candidates_.clear();
  cv_ptr_ = NULL;
}

void check_intersection_and_extend(vector<Rect> &rects, Rect rect)
{
  // check current rect with all previous rects -> if intersection-> expand else add to vector
  uint8_t i = 0;
  bool do_append = true;
  
  // for each rect in rects
  while (i < rects.size())
  {
    // if there  is intersecion and new is not fully inside 
    if ( ((rects[i] & rect).area() > 0) & ((rects[i] & rect).area() < rect.area()) )
    {
      rects[i] = (rects[i] | rect);
      rect = rects[i];
      do_append = false;
      i = 0;
      continue;
    }

    if ( ((rects[i] & rect).area() > 0) & ((rects[i] & rect).area() ==  rect.area()) )
    {
      
      // if ((rects[i] & rect).area() < rect.area())
      do_append = false;
    }
    i += 1;
  }
  if (do_append)
    rects.push_back(rect);
}

void preProcessImageBrighnessMask(Mat &image, vector<Rect> &rects, vector<Mat> &candidates, int brightness_treshold)
{
  rects.clear();
  static cv::Ptr<aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  static cv::Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);

  Mat roi = image.clone();

  int devision_coeff = 5;
  resize(roi, roi, Size(roi.cols / devision_coeff, roi.rows / devision_coeff));

  uint8_t rect_size = 7;
  
  // for each pixel, if brighter than treshold -> create rects with the brightest areas
  for (uint8_t i = 0; i < roi.rows - 1; i++)
  {
    for (uint8_t j = 0; j < roi.cols - 1; j++)
      if (roi.at<uint8_t>(i, j) < brightness_treshold)
      {
        // if in area +-7 pixels rectange-> combine
        int j_min;
        ((j - rect_size) > 0) ? j_min = j - rect_size : j_min = 0;
        int i_min;
        ((i - rect_size) > 0) ? i_min = i - rect_size : i_min = 0;

        // ROS_WARN_STREAM_THROTTLE(1, roi.size);
        int j_max;
        ((j + rect_size) < roi.cols) ? j_max = j + rect_size : j_max = roi.cols;
        int i_max;
        ((i + rect_size) < roi.rows) ? i_max = i + rect_size : i_max = roi.rows;

        
        check_intersection_and_extend(rects, Rect(Point((j_min)*devision_coeff, (i_min)*devision_coeff),
                                                  Point((j_max)*devision_coeff, (i_max)*devision_coeff)));
      }
  }
  // std::cout << " "<< std::endl;
  for (auto &rect : rects)
  {
    rect = rect & (Rect(0, 0, image.cols - 1, image.rows - 1));
    if (rect.width > 0)
    {
      candidates.push_back(image(rect));
      // std::cout << "x "<<rect.x <<" y "<<rect.x << " h "<<rect.height <<" w "<< rect.width <<" "<< std::endl;
      cv::rectangle(image, rect, Scalar(125), 10);
    }
  }
  // imshow("small image", image);
  // waitKey(10);
}

void SceneHolder::saveStaticMarkersPosition()
{
  if (scene_status_ == Scene_statuses::search_static)
  {
    for (auto static_marker : static_markers_)
    {
      if (static_marker.is_visible_)
        ROS_DEBUG_STREAM("Found static marker with id: " << static_marker.id_
                                                         << ", center(in cam base): " << static_marker.tvec_);
    }
  }
}

void SceneHolder::saveLogsToFile()
{
  // saveCubePosition();
  saveStaticMarkersPosition();
}

geometry_msgs::PoseStamped SceneHolder::calc_msg_from_position(MarkerCube &arucoCube, const Mat &transform,
                                                               bool apply_allocation)
{
  Mat trans;
  tf::Quaternion qrot;

  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = "map";
  msg.header.stamp = ros::Time::now();

  msg.pose.position.x = arucoCube.center_.x;
  msg.pose.position.y = arucoCube.center_.y;
  msg.pose.position.z = 0;  // coordinates_in_Map_base.at<double>(2,0);

  Mat rvec_mat;  //= eulerAnglesToRotationMatrix();
  Vec3d rvec = arucoCube.markers_[0].getRvec();
  Rodrigues(rvec, rvec_mat);

  tf::Matrix3x3 mrot(rvec_mat.at<double>(0, 0), rvec_mat.at<double>(0, 1), rvec_mat.at<double>(0, 2),
                     rvec_mat.at<double>(1, 0), rvec_mat.at<double>(1, 1), rvec_mat.at<double>(1, 2),
                     rvec_mat.at<double>(2, 0), rvec_mat.at<double>(2, 1), rvec_mat.at<double>(2, 2));

  tfScalar roll, pitch, yaw;
  mrot.getRPY(roll, pitch, yaw);

  // if (apply_allocation)
  // {
  //   if (arucoCube.markers_[0].id_ != arucoCube.cube_side_qr_id_[4])
  //   {
  //     mrot.setRPY(roll + 3.14 / 2, pitch, yaw);
  //   }
  // }

  // if (arucoCube.markers_[0].id_ == arucoCube.cube_side_qr_id_[4])
  // {
  //   mrot.setRPY(roll, pitch, yaw + 3.14 / 2);
  // }

  if (arucoCube.markers_[0].id_ == arucoCube.cube_side_qr_id_[0])
  {
    mrot.setRPY(roll, pitch, yaw + 3.14 / 2);
  }

  // if (arucoCube.markers_[0].id_ == arucoCube.cube_side_qr_id_[1])
  // {
  //   mrot.setRPY(roll, pitch, yaw + 3.14 / 2);
  // }

  mrot.getRPY(roll, pitch, yaw);
  if (arucoCube.markers_[0].id_ == arucoCube.cube_side_qr_id_[2])
  {
    mrot.setRPY(roll, pitch, yaw - 3.14 / 2);
  }

  if (arucoCube.markers_[0].id_ == arucoCube.cube_side_qr_id_[3])
  {
    mrot.setRPY(roll, pitch, yaw - 3.14);
  }

  mrot.getRotation(qrot);

  qrot.normalize();

  msg.pose.orientation.x = qrot[0];
  msg.pose.orientation.y = qrot[1];
  msg.pose.orientation.z = qrot[2];
  msg.pose.orientation.w = qrot[3];
  return msg;
}

geometry_msgs::PoseStamped SceneHolder::calc_msg_from_position_old(const MarkerCube &arucoCube, const Mat &transform,
                                                                   bool apply_allocation)
{
  Mat trans;
  tf::Quaternion qrot;
  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = "map";
  msg.header.stamp = timestamp_;
  // msg.header.stamp = ros::Time::now();

  Point3f center = arucoCube.center_;
  Mat coordinates_in_Map_base = (Mat_<double>(4, 1) << center.x, center.y, center.z, 1);
  coordinates_in_Map_base = transform * coordinates_in_Map_base;

  msg.pose.position.x = coordinates_in_Map_base.at<double>(0, 0);
  msg.pose.position.y = coordinates_in_Map_base.at<double>(1, 0);
  msg.pose.position.z = 0;

  Affine3d marker_to_cam(arucoCube.rotation_, arucoCube.markers_[arucoCube.strongest_marker_id_].tvec_);

  trans = Mat((Affine3d(transform) * marker_to_cam).matrix);

  trans = apply_rotation_according_to_marker_side(arucoCube.strongest_marker_id_, trans, arucoCube.cube_side_qr_id_);
  tf::Matrix3x3 mrot(trans.at<double>(0, 0), trans.at<double>(0, 1), trans.at<double>(0, 2), trans.at<double>(1, 0),
                     trans.at<double>(1, 1), trans.at<double>(1, 2), trans.at<double>(2, 0), trans.at<double>(2, 1),
                     trans.at<double>(2, 2));

  tfScalar roll, pitch, yaw;
  mrot.getRPY(roll, pitch, yaw);
  if (apply_allocation)
  {
    if (arucoCube.markers_[0].id_ != arucoCube.cube_side_qr_id_[4])
    {
      mrot.setRPY(roll + 3.14 / 2, pitch, yaw);
    }
  }

  if (arucoCube.markers_[0].id_ == arucoCube.cube_side_qr_id_[4])
  {
    mrot.setRPY(roll, pitch, yaw + 3.14 / 2);
  }

  mrot.getRPY(roll, pitch, yaw);
  if (arucoCube.markers_[0].id_ == arucoCube.cube_side_qr_id_[2])
  {
    mrot.setRPY(roll, pitch, yaw - 3.14 / 2);
  }

  if (arucoCube.markers_[0].id_ == arucoCube.cube_side_qr_id_[3])
  {
    mrot.setRPY(roll, pitch, yaw - 3.14);
  }
  if (arucoCube.markers_[0].id_ == arucoCube.cube_side_qr_id_[0])
  {
    mrot.setRPY(roll, pitch, yaw + 3.14 / 2);
  }
  mrot.getRotation(qrot);

  qrot.normalize();

  msg.pose.orientation.x = qrot[0];
  msg.pose.orientation.y = qrot[1];
  msg.pose.orientation.z = qrot[2];
  msg.pose.orientation.w = qrot[3];
  return msg;
}
