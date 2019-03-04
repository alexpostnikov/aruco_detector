// ### TODO:  check  during pose detection  that corners allocated properly
// ### TODO:  add    during pose detection  all 4 corners
// ### TODO:  check  during prefiltration that returned proper pixel
// ### TODO:  check  during prefiltration that returned proper pixel

#include <functional>
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
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include "std_msgs/String.h"

#include <chrono>
#include <cstdlib>
#include <ctime>
#include "aruco_detector/ChangeExposure.h"
#include "aruco_marker_classes.h"
#include "aruco_scene.h"
#include "aruco_utils.h"
#include "serialunix.h"
using namespace cv;

#define DEBUG

namespace enc = sensor_msgs::image_encodings;
void soft_delay(long int ticks)
{
  for (; ticks > 0; ticks--)
    ;
}
ArucoDetectorParameters aruco_params;
volatile bool recalibrate = false;


int calc_new_exposure(cv::Mat image, int area, int target_brightness, int current_exposure);
void switchToCubeFinding(SceneHolder sceneHolder, ros::ServiceClient changeExposureClient, aruco_detector::ChangeExposure changeExposureClientValue, Scene_statuses::Scene_status prev_status);
void checkExposureForStaticMarkers(SceneHolder sceneHolder, ros::ServiceClient changeExposureClient, aruco_detector::ChangeExposure changeExposureClientValue);
int main(int argc, char** argv)
{
  std::string path("/dev/ttyACM0");
  ros::init(argc, argv, "aruco_detector_node");

  ros::NodeHandle pnh;  // parameters node handler
  readRosParams(pnh, aruco_params);
  SceneHolder sceneHolder(aruco_params);
  sceneHolder.loadCameraParams();
  ros::NodeHandle n;
  ros::ServiceClient changeExposureClient = n.serviceClient<aruco_detector::ChangeExposure>("ChangeExposure");
  ros::Subscriber sub = n.subscribe("ocam/image_raw", 1, &SceneHolder::imageCallback, &sceneHolder);
  ros::Publisher enemy_robot_pose1 = n.advertise<geometry_msgs::PoseStamped>("/enemy_robot1/aruco", 30);
  ros::Publisher enemy_robot_pose2 = n.advertise<geometry_msgs::PoseStamped>("/enemy_robot2/aruco", 30);
  ros::Publisher sber_robot_pose1 = n.advertise<geometry_msgs::PoseStamped>("/big_robot/aruco", 30);
  ros::Publisher sber_robot_pose2 = n.advertise<geometry_msgs::PoseStamped>("/small_robot/aruco", 30);

  std::srand(std::time(nullptr));
  aruco_detector::ChangeExposure changeExposureClientValue;

  changeExposureClientValue.request.a = aruco_params.exposure_static_markers;

  
  // ------------- set initial Exposure  -------------
  int timeout = 10;
  while (!changeExposureClient.call(changeExposureClientValue) && (timeout--))
  {
    soft_delay(100);
  }
  // ------------- END set initial Exposure  ------------- //

  ros::Rate loop_rate(30);
  Scene_statuses::Scene_status prev_status = Scene_statuses::search_static;
  sceneHolder.params_ = aruco_params;
  while (ros::ok())
  {
    if (sceneHolder.cv_ptr_ != NULL)
    {
      sceneHolder.findMarkers();

      sceneHolder.allocateStaticMarkers();

      sceneHolder.allocateCubes();

      sceneHolder.calcAllCenters();

      sceneHolder.calc_new_transform();

      sceneHolder.showImages(true, true, false);

      sceneHolder.publish(sber_robot_pose1, sber_robot_pose2, enemy_robot_pose1, enemy_robot_pose2);

      sceneHolder.saveLogsToFile();

      switchToCubeFinding(sceneHolder, changeExposureClient, changeExposureClientValue, prev_status);

      checkExposureForStaticMarkers(sceneHolder, changeExposureClient, changeExposureClientValue);

      prev_status = sceneHolder.scene_status_;

      sceneHolder.clearOldData();
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

void checkExposureForStaticMarkers(SceneHolder sceneHolder, ros::ServiceClient changeExposureClient, aruco_detector::ChangeExposure changeExposureClientValue)
{
  if ((sceneHolder.scene_status_ == Scene_statuses::search_static) &&
    (sceneHolder.numb_visible_static_markers_ != sceneHolder.params_.number_of_static_markers))
  {
    int new_exposure = calc_new_exposure(sceneHolder.cv_ptr_->image, 100, 70, changeExposureClientValue.request.a);
    if (new_exposure > 0)
    {
      changeExposureClientValue.request.a = new_exposure;
      std::cout << "setting new exposure : " << new_exposure << std::endl;
      int timeout = 2;
      while (!changeExposureClient.call(changeExposureClientValue) && timeout--)
      {
        soft_delay(100);
      }
    }
  }
}

void switchToCubeFinding(SceneHolder sceneHolder, ros::ServiceClient changeExposureClient, aruco_detector::ChangeExposure changeExposureClientValue, Scene_statuses::Scene_status prev_status){

  if (prev_status != sceneHolder.scene_status_)
      {
        changeExposureClientValue.request.a = sceneHolder.params_.exposure_cube_markers;

        int timeout = 2;
        while (!changeExposureClient.call(changeExposureClientValue) && timeout--)
        {
          soft_delay(100);
        };
      }
}

int calc_new_exposure(cv::Mat image, int area, int target_brightness, int current_exposure)
{
  cv::Rect rect_area(Point(image.rows / 2 - area, image.cols / 2 - area),
                     Point(image.rows / 2 + area, image.cols / 2 + area));
  cv::Mat imageRoi = image(rect_area).clone();
  Scalar mean_brightness = cv::mean(imageRoi);
  // std::cout << "mean_bright" << mean_brightness.val[0] << std::endl;
  if (mean_brightness.val[0] == 0)
  {
    mean_brightness.val[0] = 1;
  }
  int new_exposure = current_exposure / (target_brightness / mean_brightness.val[0]);
  new_exposure = (new_exposure - current_exposure) / 1.8 + current_exposure;
  std::srand(std::time(nullptr));

  new_exposure = new_exposure + std::rand() / ((RAND_MAX + 1u) / 30) - 15;
  return new_exposure;
}
