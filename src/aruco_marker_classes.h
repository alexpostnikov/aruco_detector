#pragma once
#include <iostream>  
#include <tf/transform_broadcaster.h>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <queue>
#include <ctime>
#include <fstream>
#include "aruco_parameters.h"
#include <map>
#include <string>
typedef struct
{
  // params
  int exposure_static_markers = 30;
  int exposure_cube_markers = 7;
  int used_cube_sides = 5;
  int static_state_exposure = 50;
  int cube_state_exposure = 10;
  int used_board_type = 8;
  int use_top_line_marker = 1;
  int use_bottom_line_marker = 0;
  float len_of_cube_markers = 0.064;
  float len_of_cube = 0.08;
  float width_marker = 0.06;
  std::vector<int> big_sber_ids =   {99, 99, 99, 99, 99, 99, 99, 99 };
  std::vector<int> small_sber_ids = { 99, 99, 99, 99, 99, 99, 99, 99 };
  std::vector<int> big_enemy_ids =  {99, 99, 99, 99, 99, 99, 99, 99 };
  std::vector<int> small_enemy_ids = { 99, 99, 99, 99, 99, 99, 99, 99 };
  std::vector<int> small_top_side_ids = { 1, 2, 3, 99, 99, 99, 99 };
  
  int number_of_static_markers = 6;
  std::vector<int> static_markers_ids = { 5, 0, 2, 3, 1, 6 };
  float static_marker_sizes = 0.16;
  float brightness_threshold = 70;
  std::vector<float> camera_position = {1.5,2,1.02};
  std::vector<float> camera_position_orange = {1.5,2,1.02};
  std::vector<float> camera_position_green = {1.5,2,1.02};
  int max_missed_static_markers_= 20;
  std::map<std::string, std::vector<int>> marker_ids;
  std::map<std::string, std::vector<int>> markers_orientation;
  std::map<std::string, float> markers_heights;
  std::map<std::string, std::vector<float>> markers_poses;
} ArucoDetectorParameters;

class Marker {

	public:
		cv::Vec3d rvec_, tvec_; //rotation vector translation vector of marker
		int id_; // id of marker
		std::vector<cv::Point2f> corners_; //pixels numbers
		double marker_size_;			   // input param -> msize of marker

		Marker(cv::Vec3d rv, cv::Vec3d tv, int id, double size) {rvec_ = rv; tvec_ = tv; id_= id; marker_size_ = size;}
		Marker(cv::Vec3d rv, cv::Vec3d tv, int id) {rvec_ = rv; tvec_ = tv; id_= id; marker_size_ = 0;}
		Marker(int id) {id_= id; marker_size_ = 0; }
		Marker(){marker_size_ = 0;}
		
		cv::Vec3d getRvec(void){return rvec_;}
		void setRvec  (cv::Vec3d rvec)   {rvec_ = rvec;}
		
		cv::Vec3d getTvec(void){return tvec_;}
		void setTvec (cv::Vec3d tvec)  { tvec_ = tvec;}
		
		int getId(void){return id_;}

		void set_corners(std::vector<cv::Point2f> corners) {corners_ = corners;}

		std::vector<cv::Point2f> getCorners(void) {return corners_;}

		double getSize(void) {return marker_size_;}
		
		void clearOldData(){corners_.clear();} 
};

class MarkerStatic : public Marker
{
		
	public: 
		std::vector<cv::Point3d> centers_; //center of static marker
		uint16_t max_len_of_centers_ = 2;  //max len of vector of centers_
		bool is_visible_ = false; // is marker is visible or not
		MarkerStatic(uint8_t id, double size) {id_=id; marker_size_ = size;}
		
		void calcCenter(cv::Mat cameraMatrix, cv::Mat distCoeffs); // calculate  center of marker knowing std::vector<cv::Point2f> corners_, and camera params
		cv::Point3d getAveragedStaticMarkerCenter(); // averages centers_ 
		void addNewMarker (std::vector<cv::Point2f> corners){ corners_ = corners;} //saves corners of marker;
		void clearOldData(){corners_.clear(); is_visible_ = false;}
};


class MarkerCube {

public:

	tf::Quaternion quat_;                                  // quaternion to be published
	std::vector<Marker> markers_;                          // vector of seen markers
	enum cv::aruco::PREDEFINED_DICTIONARY_NAME board_type; // used board type
	std::vector<int> cube_side_qr_id_;						   // markers id
	cv::Point3d center_;								   // calculated center of cube
	cv::Vec3d   rotation_;								   // calculated rvec of cube in cam coords
	// MarkerCube(cv::aruco::PREDEFINED_DICTIONARY_NAME, uint8_t*);
	MarkerCube();
	int strongest_marker_id_;							   // id of most close
	bool is_visible_ = false;
	bool isMineMarkerId(int id);
	bool calcCenter(cv::Mat cameraMatrix, cv::Mat distCoeffs, cv::Mat new_transform_from_cam_to_map, ArucoDetectorParameters params);
	void clearOldData() {markers_.clear(); is_visible_ = false;}
};


void allocate_marker_rvec(cv::Vec3d &rvec, const cv::Vec3d tvec, int marker_id, const cv::Mat transform_from_cam_to_map_, uint8_t* cube_side_qr_id_);
tf::Quaternion  calcOrientation (cv::Vec3f rvec, int id , uint8_t* id_list);

