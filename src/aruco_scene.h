#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include "aruco_marker_classes.h"
#include "ros/ros.h"

namespace Scene_statuses
{
enum Scene_status
{
  search_static = 0x00,  // first state -> looking for static markers -> without pre-filtration
  search_cubes = 0x01    // after calculating transform (cam->map) looking for only marker cubes -> using pre-filtration
};
}
typedef Scene_statuses::Scene_status Scene_status;



class Cropped_scene
{
public:
  std::vector<cv::Mat> candidates_;  // crepped parts of camera snapshot
  std::vector<cv::Rect> areas_;  // saved coordinates(pixels x1,y1,x2,y2) of candidates_ relative to original snapshot

  Cropped_scene(std::vector<cv::Mat> cand, std::vector<cv::Rect> areas)
  {
    candidates_ = cand;
    areas_ = areas;
  };
  Cropped_scene(){};
};

class SceneHolder
{
private:
  bool inverseImage_ = true;  // if true -> do inversing of image (black- white; white- black)

  cv::Mat transform_from_cam_to_map_;  // transformation from camera to map
  cv::Mat camera_matrix_, dist_coeffs_;

public:

  SceneHolder();
  ros::Time timestamp_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detectorParams_;
  SceneHolder(ArucoDetectorParameters);
  std::vector<MarkerStatic> static_markers_;  // vector of visible static marker
  void calc_new_transform();
  cv::Mat new_transform_from_cam_to_map_;
  MarkerCube arucoCubes_[4];  // array of arucoCubes_ -> 2 of them our team, and 2 enemy
  ArucoDetectorParameters params_;
  Scene_status scene_status_ = Scene_statuses::search_static;  // status of scene (looking for static markers or cubes)
  bool calibration_status_ = 0; //if 0 - calibration failed, if 1 - success
  cv_bridge::CvImagePtr cv_ptr_;                               // ptr to image
  Cropped_scene cropped_scene_;
  std::vector<Marker> found_markers_;                         // every found marker
  int numb_visible_static_markers_;                           // number of visible static markers
  void loadCameraParams();                                    // load camera matrix and dist coeffs
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);  // receiving new image from cam driver
  void findMarkers();                                         // find markers from image
  void allocateStaticMarkers();                               // check all found markers for containing static markers
  void allocateCubes();                                       // check all found markers for containing cube   markers
  void calcAllCenters();                                      // calc centers of gound markers
  void calcTransform();  // calculate transform cam -> map (if all static markers are visible/ else load it from
                         // previous game)
  void showImages(bool is_draw_markers, bool is_draw_axis, bool is_show_undist);  //  show image on screen

  void publish(ros::Publisher sber_robot_pose1, ros::Publisher sber_robot_pose2, ros::Publisher enemy_robot_pose1,
               ros::Publisher enemy_robot_pose2);  // publish cube positions
  void saveLogsToFile();
  void clearOldData();

  void showStatistics();
  void ShowTablePerspevtive();  // show table position on screen
  void saveCubePosition();
  void saveStaticMarkersPosition();
  geometry_msgs::PoseStamped calc_msg_from_position_old(const MarkerCube &arucoCube, const cv::Mat &transform,
                                                  bool apply_allocation);
  geometry_msgs::PoseStamped calc_msg_from_position(MarkerCube &arucoCube, const cv::Mat &transform, bool apply_allocation);
  cv::Mat getTransform()
  {
    return transform_from_cam_to_map_;
  }

  // Mat getTransform(void);
};

void allocateZAxis(cv::Mat);
void preProcessImageBrighnessMask(cv::Mat& image, std::vector<cv::Rect>& rects, std::vector<cv::Mat>& candidates,
                                  int treshold);
