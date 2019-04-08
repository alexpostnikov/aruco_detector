#include "std_msgs/String.h"
#include <iostream>
#include <string>
#include <fstream>
#include <iterator>


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include "aruco_parameters.h"
#include "aruco_marker_classes.h"

#include <ros/console.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>

using namespace cv;
using namespace std;	

bool saveCameraParams(const std::string &filename, Size imageSize, float aspectRatio, int flags, 	const Mat &cameraMatrix, const Mat &distCoeffs, double totalAvgErr);

bool calibrate_camera(cv_bridge::CvImagePtr image);

std::vector<double> calculate_center_of_cube(std::vector<cv::Vec3d> rvec , std::vector<cv::Vec3d> tvec, int index);

std::vector<double> combine_results(MarkerCube &cube1_v_blurred, MarkerCube &cube1_v_original);

bool readRosParams(ros::NodeHandle pnh, ArucoDetectorParameters &aruco_params);
bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params);

geometry_msgs::PoseStamped createMessageFromCube( MarkerCube &cube);

bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs);

double calculateDeviationOfVector (std::vector<double> v);

void calculateCameraPosition(vector<double> center1, vector<double> center2,  vector<double> expected_center1, vector<double> expected_center2, vector<double> &tvec, vector<double> &rvec);

Mat reflection (Mat A, Mat v);

bool isRotationMatrix(Mat &R);

vector<double> rotationMatrixToEulerAngles(Mat &R);

void clear_data_from_previous_image(vector <MarkerCube> &vector_of_cubes);

void printCubesCoordsOnImage (Mat image, std::vector <MarkerCube> vector_of_cubes, Mat transform, bool changeState);

cv::Point3d calcCenterOfCube (cv::Vec3d rvec, cv::Vec3d tvec);

cv::Point3d calcCenterOfRobot_by_disp(cv::Vec3d rvec, cv::Vec3d tvec, std::vector<float> delta_rvec);

bool readCamToMapTransform(const std::string &filename,   Mat &transform);

bool saveCamToMapTransform(const std::string &filename,  const Mat &transform);

cv::Mat apply_rotation_according_to_marker_side (int marker_id, cv::Mat rvec_mat, std::vector<int> list_ids);


void move_and_norm_vector(Mat &vec, Mat transform, std::vector<float> camera_position, const float z_goal);
Mat eulerAnglesToRotationMatrix(Vec3f &theta);

cv::Mat new_allocation_maps(Point3d marker1_cam_base, Point3d marker2_cam_base, std::vector<float> camera_position);


cv::Mat new_allocation_maps(cv::Point3d marker1_cam_base, cv::Point3d marker2_cam_base, std::vector<float> camera_position);