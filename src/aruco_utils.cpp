#include <math.h>
#include <time.h>
#include <algorithm>
#include <functional>
#include <iostream>
#include <numeric>
#include <string>
#include "aruco_utils.h"
#include <sstream>

bool flag_to_write = false;
std::vector<std::vector<std::vector<Point2f>>> allCorners;
std::vector<std::vector<int>> allIds;

static Mat1d cameraMatrix;
static Mat distCoeffs;

bool saveCameraParams(const std::string &filename, Size imageSize, float aspectRatio, int flags,
                      const Mat &cameraMatrix, const Mat &distCoeffs, double totalAvgErr)
{
  FileStorage fs(filename, FileStorage::WRITE);
  if (!fs.isOpened())
    return false;

  time_t tt;
  time(&tt);
  struct tm *t2 = localtime(&tt);
  char buf[1024];
  strftime(buf, sizeof(buf) - 1, "%c", t2);

  fs << "calibration_time" << buf;

  fs << "image_width" << imageSize.width;
  fs << "image_height" << imageSize.height;

  if (flags & CALIB_FIX_ASPECT_RATIO)
    fs << "aspectRatio" << aspectRatio;

  if (flags != 0)
  {
    sprintf(buf, "flags: %s%s%s%s", flags & CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
            flags & CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
            flags & CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
            flags & CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
  }

  fs << "flags" << flags;

  fs << "camera_matrix" << cameraMatrix;
  fs << "distortion_coefficients" << distCoeffs;

  fs << "avg_reprojection_error" << totalAvgErr;

  return true;
}

bool saveCamToMapTransform(const std::string &filename, const Mat &transform)
{
  FileStorage fs(filename, FileStorage::WRITE);
  if (!fs.isOpened())
    return false;

  time_t tt;
  time(&tt);
  struct tm *t2 = localtime(&tt);
  char buf[1024];
  strftime(buf, sizeof(buf) - 1, "%c", t2);

  fs << "writing_time" << buf;

  fs << "mat_rows" << transform.rows;
  fs << "mat_cols" << transform.cols;

  fs << "trasnform" << transform;

  return true;
}

bool readCamToMapTransform(const std::string &filename, Mat &transform)
{
  FileStorage fs(filename, FileStorage::READ);
  if (!fs.isOpened())
  {
    std::cout << "cannot read transform" << endl;
    return false;
  }

  fs["trasnform"] >> transform;

  return true;
}

bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params)
{
  FileStorage fs(filename, FileStorage::READ);
  if (!fs.isOpened())
    return false;
  fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
  fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
  fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
  fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
  fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
  fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
  fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
  fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
  fs["minDistanceToBorder"] >> params->minDistanceToBorder;
  fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
  fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
  fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
  fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
  fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
  fs["markerBorderBits"] >> params->markerBorderBits;
  fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
  fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
  fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
  fs["minOtsuStdDev"] >> params->minOtsuStdDev;
  fs["errorCorrectionRate"] >> params->errorCorrectionRate;
  return true;
}


std::vector<int> parse_string_ids(std::string str)
{
  std::vector<int> result;
  stringstream ss(str);
  while( ss.good() )
  {
      string substr;
      getline( ss, substr, ',' );

      result.push_back(std::stoi(substr));
  }
  return result;
}


bool readRosParams(ros::NodeHandle pnh, ArucoDetectorParameters &aruco_params)
{
  pnh.getParam("exposure_static_markers", aruco_params.exposure_static_markers);
  pnh.getParam("exposure_cube_markers", aruco_params.exposure_cube_markers);
  pnh.getParam("len_of_cube_markers", aruco_params.len_of_cube_markers);
  pnh.getParam("len_of_cube", aruco_params.len_of_cube);
  // pnh.getParam("big_sber_ids", aruco_params.big_sber_ids);
  // pnh.getParam("small_sber_ids", aruco_params.small_sber_ids);
  // pnh.getParam("small_top_side_ids", aruco_params.small_top_side_ids);
  // pnh.getParam("big_enemy_ids", aruco_params.big_enemy_ids);
  // pnh.getParam("small_enemy_ids", aruco_params.small_enemy_ids);
  pnh.getParam("number_of_static_markers", aruco_params.number_of_static_markers);
  pnh.getParam("static_markers_ids", aruco_params.static_markers_ids);
  pnh.getParam("static_marker_sizes", aruco_params.static_marker_sizes);
  pnh.getParam("brightness_threshold", aruco_params.brightness_threshold);
  pnh.getParam("camera_position_green", aruco_params.camera_position_green);
  pnh.getParam("camera_position_orange", aruco_params.camera_position_orange);
  pnh.getParam("max_missed_static_markers", aruco_params.max_missed_static_markers_);
  pnh.getParam("used_cube_sides", aruco_params.used_cube_sides);
  pnh.getParam("used_board_type", aruco_params.used_board_type);
  pnh.getParam("width_marker", aruco_params.width_marker);
  pnh.getParam("use_top_line_marker", aruco_params.use_top_line_marker);
  pnh.getParam("use_bottom_line_marker", aruco_params.use_bottom_line_marker);
  ROS_WARN_STREAM ( "aruco_params.static_markers_ids: ");
  for (auto static_markers_id :aruco_params.static_markers_ids)
    ROS_WARN_STREAM (static_markers_id);
  // ---------- receive markers_ids. Due to ros params server "feature" need to reparse string to vector ----------//
  
  std::map<std::string, std::string> markers_ids;
  std::map<std::string, std::vector<int>> markers_ids_vect;
  pnh.getParam("markers", markers_ids);
  // structure : {key : "id, id, id, id"}
  for(auto const& imap: markers_ids) markers_ids_vect[imap.first] = parse_string_ids(imap.second);
  aruco_params.marker_ids = markers_ids_vect;
  aruco_params.big_sber_ids = markers_ids_vect["big_sber_cube_ids"];
  aruco_params.small_sber_ids = markers_ids_vect["small_sber_cube_ids"];
  aruco_params.small_top_side_ids = markers_ids_vect["small_top_side_ids"];
  aruco_params.big_enemy_ids = markers_ids_vect["big_enemy_ids"];
  aruco_params.small_enemy_ids = markers_ids_vect["small_enemy_ids"];
  // ------------end ----------//


  // ---------- receive marker_orientation. Due to ros params server "feature" need to reparse string to vector ----------//
  std::map<std::string, std::string> markers_rotation;
  std::map<std::string, std::vector<int>> markers_rot_vect;
  pnh.getParam("markers_rotation", markers_rotation);
  // structure : {key : "id, id, id, id"}
  for(auto const& imap: markers_rotation) markers_rot_vect[imap.first] = parse_string_ids(imap.second);
  aruco_params.markers_orientation = markers_rot_vect;
  // ------------end ----------//
  
  std::string side_color;
  pnh.getParam("side", side_color);
  if (side_color == "orange")
    {pnh.getParam("camera_position_orange", aruco_params.camera_position);}
  if (side_color == "green")
    {pnh.getParam("camera_position_green", aruco_params.camera_position);}

  return true;
}

bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs)
{
  FileStorage fs(filename, FileStorage::READ);
  if (!fs.isOpened())
  {
    cout << "CANNOT LOAD FILE WITH CAMERA PAREMETERS  " << filename << endl;
    return false;
  }
  fs["camera_matrix"] >> camMatrix;
  fs["distortion_coefficients"] >> distCoeffs;
  return true;
}

cv::Point3d calcCenterOfCube(cv::Vec3d rvec, cv::Vec3d tvec)
{
  cv::Mat rot_mat(3, 3, CV_64F);
  cv::Rodrigues(rvec, rot_mat);
  cv::Mat T1 = Mat::eye(4, 4, CV_64F); // matrice allocated to center of marker
  rot_mat.copyTo(T1(Range(0, 3), Range(0, 3)));
  (Mat(tvec))(Range(0, 3), Range(0, 1)).copyTo(T1(Range(0, 3), Range(3, 4)));

  cv::Mat T2 = Mat::zeros(4, 1, CV_64F);

  T2.at<double>(1, 0) = static_cast<double>(-1.0 * 0.11 / 2.0);
  // T2.at<double>(2, 0) = -10;
  T2.at<double>(3, 0) = static_cast<double>(1.0);

  T2 = T1 * T2;

  cv::Point3d coodrinates_center = Point3d(T2.at<double>(0, 0), T2.at<double>(1, 0), T2.at<double>(2, 0));
  return coodrinates_center;
}

cv::Point3d calcCenterOfRobot_by_disp(cv::Vec3d rvec, cv::Vec3d tvec, cv::Vec3d delta_rvec)
{
  cv::Mat rot_mat(3, 3, CV_64F);
  cv::Rodrigues(rvec, rot_mat);
  cv::Mat T1 = Mat::eye(4, 4, CV_64F); // matrice allocated to center of marker
  rot_mat.copyTo(T1(Range(0, 3), Range(0, 3)));
  (Mat(tvec))(Range(0, 3), Range(0, 1)).copyTo(T1(Range(0, 3), Range(3, 4)));

  cv::Mat T2 = Mat::zeros(4, 1, CV_64F);
  T2.at<double>(0, 0) = static_cast<double>(delta_rvec[0]);
  T2.at<double>(1, 0) = static_cast<double>(delta_rvec[1]);
  T2.at<double>(2, 0) = static_cast<double>(delta_rvec[2]);
  T2.at<double>(3, 0) = static_cast<double>(1.0);

  T2 = T1 * T2;

  cv::Point3d coodrinates_center = Point3d(T2.at<double>(0, 0), T2.at<double>(1, 0), T2.at<double>(2, 0));
  return coodrinates_center;
}

std::vector<double> calculate_center_of_cube(std::vector<cv::Vec3d> rvec, std::vector<cv::Vec3d> tvec, int index)
{
  return std::vector<double>({tvec[index][0], tvec[index][1], tvec[index][2]});
}

cv::Mat apply_rotation_according_to_marker_side(int marker_id, cv::Mat rvec_mat, std::vector<int>list_ids)
{
  // Mat rvec_mat(3, 3, CV_32F);

  int marker_side = -1;
  Mat new_rot_mat(3, 3, rvec_mat.type()), Roi(3, 3, rvec_mat.type());
  // TODO: change next line with parameter
  int NUMB_CUBE_SIDES = 6;
  for (int i = 0; i < NUMB_CUBE_SIDES; i++)
  {
    if (marker_id == list_ids[i])
      marker_side = i;
  }
  switch (marker_side)
  {
  case 0:
    Roi = (Mat_<double>(3, 3) << 0, 1, 0, -1, 0, 0, 0, 0, 1);
    new_rot_mat = Roi * rvec_mat(Range(0, 3), Range(0, 3));

    break;

  // left side
  case 1: // 8
    Roi = (Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, -1);
    new_rot_mat = Roi * rvec_mat(Range(0, 3), Range(0, 3));
    break;

  // back side
  case 2: // 20
    Roi = (Mat_<double>(3, 3) << 0, 0, -1, 1, 0, 0, 0, -1, 0);
    new_rot_mat = Roi * rvec_mat(Range(0, 3), Range(0, 3));
    break;

  // right side
  case 3: // 10
    Roi = (Mat_<double>(3, 3) << -1, 0, 0, 0, -1, 0, 0, 0, -1);
    new_rot_mat = Roi * rvec_mat(Range(0, 3), Range(0, 3));
    break;

  // upper side
  case 4:
    // Roi = (Mat_<double>(3,3) << 0,0,-1, 1,0,0, 0,1,0 );
    // ROS_INFO("case 4: ");
    Roi = (Mat_<double>(3, 3) << -1, 0, 0, 0, -1, 0, 0, 0, -1);
    new_rot_mat = Roi * rvec_mat(Range(0, 3), Range(0, 3));
    // std::cout  <<" rvec_mat "<< rvec_mat(Range(0,3), Range(0,3)) << std::endl << std::endl;
    break;

  default:
    cout << "ERROR AT cv::Vec3f calcOrientation(cv::Vec3f rvec, int id , uint8_t* id_list) " << endl;
    return rvec_mat;
  }
  // Vec3f new_rvec;

  return new_rot_mat;
}

double calculateDeviationOfVector(std::vector<double> v)
{
  double sum = std::accumulate(std::begin(v), std::end(v), 0.0);
  double m = sum / v.size();

  double accum = 0.0;
  std::for_each(std::begin(v), std::end(v), [&](const double d) { accum += (d - m) * (d - m); });

  double stdev = sqrt(accum / (v.size() - 1));

  return stdev;
}

Mat reflection(Mat A, Mat v)
{
  Mat v_transpose;
  transpose(v, v_transpose);

  Mat refl = A - 2 * v * (v_transpose * A) / (v_transpose * v);
  return refl;
}

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Mat &R)
{
  Mat Rt;
  transpose(R, Rt);
  Mat shouldBeIdentity = Rt * R;
  Mat I = Mat::eye(3, 3, shouldBeIdentity.type());
  return norm(I, shouldBeIdentity) < 1e-6;
}

// Calculates rotation matrix to euler angles

vector<double> rotationMatrixToEulerAngles(Mat &R)
{
  // assert(isRotationMatrix(R));
  (isRotationMatrix(R));
  float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

  bool singular = sy < 1e-6; // If

  float x, y, z;
  if (!singular)
  {
    x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
    y = atan2(-R.at<double>(2, 0), sy);
    z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
  }
  else
  {
    x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
    y = atan2(-R.at<double>(2, 0), sy);
    z = 0;
  }
  return vector<double>{x, y, z};
}

void move_and_norm_vector(Mat &vec, Mat transform, std::vector<float> camera_position, const float z_goal)
{
  Mat final_vec = (Mat_<double>(4, 1) << vec.at<double>(0, 0), vec.at<double>(1, 0), vec.at<double>(2, 0), 1);

  final_vec = transform * final_vec;

  Mat delta_final_vec = final_vec;

  delta_final_vec.at<double>(0, 0) = final_vec.at<double>(0, 0) - camera_position[0];
  delta_final_vec.at<double>(1, 0) = final_vec.at<double>(1, 0) - camera_position[1];
  delta_final_vec.at<double>(2, 0) = final_vec.at<double>(2, 0) - camera_position[2];

  delta_final_vec = delta_final_vec * (z_goal / delta_final_vec.at<double>(2, 0));
  final_vec.at<double>(0, 0) = delta_final_vec.at<double>(0, 0) + camera_position[0];
  final_vec.at<double>(1, 0) = delta_final_vec.at<double>(1, 0) + camera_position[1];
  final_vec.at<double>(2, 0) = delta_final_vec.at<double>(2, 0) + camera_position[2];
  vec = final_vec;
}

Mat eulerAnglesToRotationMatrix(Vec3f &theta)
{
  // Calculate rotation about x axis
  Mat R_x = (Mat_<double>(3, 3) << 1, 0, 0,
             0, cos(theta[0]), -sin(theta[0]),
             0, sin(theta[0]), cos(theta[0]));

  // Calculate rotation about y axis
  Mat R_y = (Mat_<double>(3, 3) << cos(theta[1]), 0, sin(theta[1]),
             0, 1, 0,
             -sin(theta[1]), 0, cos(theta[1]));

  // Calculate rotation about z axis
  Mat R_z = (Mat_<double>(3, 3) << cos(theta[2]), -sin(theta[2]), 0,
             sin(theta[2]), cos(theta[2]), 0,
             0, 0, 1);

  // Combined rotation matrix
  Mat R = R_z * R_y * R_x;

  return R;
}

Mat new_allocation_maps(Point3d marker1_cam_base, Point3d marker2_cam_base, std::vector<float> camera_position)
{


  Point3d marker1_map_base(0.07, 0.07, 0.06);
  Point3d marker2_map_base(3 - 0.07, 0.07, 0.06);
  Point3d left_marker, right_marker;
  if (marker1_cam_base.x < marker2_cam_base.x)
  {
    ROS_WARN_STREAM ("SWITCHING MARKERS POSITION");
    left_marker = marker1_cam_base;
    right_marker = marker2_cam_base;
  }
  else
  {
    ROS_WARN_STREAM ("NORMAL MARKERS");
    left_marker = marker2_cam_base;
    right_marker = marker1_cam_base;
  }
  Point3d cam_map_base(camera_position[0], camera_position[1], camera_position[2]);

  marker1_map_base -= cam_map_base;
  marker2_map_base -= cam_map_base;
  Point3d normal_in_map_base = marker1_map_base.cross(marker2_map_base);

  Point3d normal_in_cam_base = right_marker.cross(left_marker);

  normal_in_map_base = normal_in_map_base / sqrt(normal_in_map_base.x * normal_in_map_base.x + normal_in_map_base.y * normal_in_map_base.y + normal_in_map_base.z * normal_in_map_base.z);
  normal_in_cam_base = normal_in_cam_base / sqrt(normal_in_cam_base.x * normal_in_cam_base.x + normal_in_cam_base.y * normal_in_cam_base.y + normal_in_cam_base.z * normal_in_cam_base.z);

  Vec3d v =  Vec3d(normal_in_cam_base.cross(normal_in_map_base));
  double s = norm(v);
  double c =  (normal_in_cam_base.dot(normal_in_map_base));

  Mat rot = Mat::eye(3,3, CV_64F);
  Mat Vx = (Mat_<double>(3,3) << 0, -v[2],v[1],v[2],0,-v[0], -v[1], v[0], 0);
  cv::add(rot, Vx, rot);
  Mat last_part = Vx*Vx*(1-c)/s/s;
  cv::add(rot, last_part, rot);


  Mat marker1_cam_base_rotated_mat  = rot* Mat(right_marker);
  Vec3d rotated_first = Vec3d(marker1_cam_base_rotated_mat.at<double>(0,0), marker1_cam_base_rotated_mat.at<double>(1,0), marker1_cam_base_rotated_mat.at<double>(2,0));
  rotated_first = rotated_first / sqrt(rotated_first[0]*rotated_first[0] + rotated_first[1]*rotated_first[1] + rotated_first[2]*rotated_first[2]);
  Vec3d marker1_map_base_n = marker1_map_base/sqrt(marker1_map_base.x * marker1_map_base.x + marker1_map_base.y * marker1_map_base.y + marker1_map_base.z * marker1_map_base.z);
  Vec3d  v2 =  Vec3d(rotated_first.cross(marker1_map_base_n));
  double s2 = norm(v2);
  double c2 =  (rotated_first.dot(marker1_map_base_n));

  Mat rot2 = Mat::eye(3,3, CV_64F);
  Mat Vx2 = (Mat_<double>(3,3) << 0, -v2[2],v2[1],v2[2],0,-v2[0], -v2[1], v2[0], 0);
  cv::add(rot2, Vx2, rot2);
  Mat last_part2 = Vx2*Vx2*(1-c2)/s2/s2;
  cv::add(rot2, last_part2, rot2);

  //marker1_map_base_n = marker1_map_base / np.linalg.norm(marker1_map_base);

  //Vec3d rvec;
  //Rodrigues(rot, rvec);
  // cout << "rot " << rot << endl;
  // cout << "rot2 " << rot2 << endl;

  // cout << "rot*rot2 " << rot2*rot << endl;
  return rot2*rot;

}



void calc_transform_v2(Point3d marker1_cam_base, Point3d marker2_cam_base)
{

    // Point3d marker1_map_base(0.15, 0.15, 0.12);
    // Point3d marker2_map_base(3 - 0.15, 0.15, 0.12);
    // Point3d cam_map_base(1.37, 2., 1.02);
    
    // double angle_mark1 = marker1_cam_base.dot(marker1_map_base - cam_map_base)/norm(marker1_cam_base)/norm(marker1_map_base - cam_map_base);
    // double angle_mark2 = marker2_cam_base.dot(marker2_map_base - cam_map_base)/norm(marker2_cam_base)/norm(marker2_map_base - cam_map_base);
    // double angle_mark2_mark1 = (marker2_cam_base - marker1_cam_base).dot(marker2_map_base - marker1_map_base)/norm(marker2_cam_base - marker1_cam_base)/norm(marker2_map_base - marker1_map_base);
    

    //angle betwen marker1_cam_base and (marker1_map_base - cam_map_base)
    //angle betwen marker2_cam_base and (marker2_map_base - cam_map_base)
    




}
