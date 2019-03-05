#include "ros/ros.h"
//#include <boost/assign/list_of.hpp>

#include <opencv2/line_descriptor.hpp>
#include "opencv2/core/utility.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Dense>
#include <opencv2/aruco.hpp>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include "std_msgs/String.h"

#include <chrono>
namespace enc = sensor_msgs::image_encodings;

cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(8);
cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();


std::vector<std::vector<cv::Point2f>> local_corners;
std::vector<int> local_ids;

void imageCallback(const sensor_msgs::ImageConstPtr &original_image)
{
  cv_bridge::CvImagePtr cv_ptr_ = cv_bridge::toCvCopy(original_image, enc::MONO8);
  cv_ptr_->image = cv::Scalar::all(255) - cv_ptr_->image;
  cvtColor(cv_ptr_->image, cv_ptr_->image, CV_GRAY2RGB);
  cv::aruco::detectMarkers(cv_ptr_->image, dictionary, local_corners, local_ids, detectorParams);
  cv::aruco::drawDetectedMarkers(cv_ptr_->image, local_corners, local_ids);
  cv::imshow("video", cv_ptr_->image);
  cv::waitKey(50);
}

int main(int argc, char **argv)
{
  
  detectorParams->adaptiveThreshWinSizeMin = 3;
  detectorParams->adaptiveThreshWinSizeMax = 23;
  detectorParams->adaptiveThreshWinSizeStep = 13;
  detectorParams->adaptiveThreshConstant = 15;
  detectorParams->perspectiveRemovePixelPerCell = 15;
  detectorParams->perspectiveRemoveIgnoredMarginPerCell = 0.33;
  detectorParams->polygonalApproxAccuracyRate = 0.1;
  ros::init(argc, argv, "aruco_show_ids_node");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("ocam/image_raw", 1, imageCallback);
  ros::spin();
  return 0;
}