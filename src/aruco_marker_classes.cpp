#include <iostream>
#include "aruco_marker_classes.h"
#include "aruco_parameters.h"
#include "aruco_utils.h"
//#include <opencv2/aruco.hpp>

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
	for (uint8_t i = 0; i < sizeof(cube_side_qr_id_) / sizeof(cube_side_qr_id_[0]); i++)
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
		std::vector<Point2f> new_point;
		undistortPoints(marker.corners_, new_point, cameraMatrix, distCoeffs, cv::noArray(), cameraMatrix);
		
		// TODO: constants? from param server?
		std::vector<Mat> corners_in_map_coords;
		float z_goal_sber_robots = 0.421 - params.camera_position[2];
		float z_goal_enemy_robots = 0.28 - params.camera_position[2];
		float z_goal_top_side_markers = 0.34 - params.camera_position[2];
		float z_goal_top_side_enemys = 0.28 + 8 - params.camera_position[2];
		// special marker (rotated unnormally)
		if (marker.id_ == 46)
		{
			for (uint8_t i = 2; i < 4; i++)
			{
				Mat vec_second = (Mat_<double>(3, 1) << new_point[i].x, new_point[i].y, 1);
				vec_second = cameraMatrix.inv() * vec_second;
				Mat second_corner = vec_second;
				move_and_norm_vector(second_corner, new_transform_from_cam_to_map, params.camera_position, z_goal_sber_robots);
				corners_in_map_coords.push_back(second_corner);
			}
		}
		/*** TOP SIDE MARKER ***/
		else if ((marker.id_ == 2) || (marker.id_ == 1) || (marker.id_ == 3) || (marker.id_ == 6) || (marker.id_ == 8))
		{
			for (uint8_t i = 0; i < 2; i++)
			{
				Mat vec_second = (Mat_<double>(3, 1) << new_point[i].x, new_point[i].y, 1);
				vec_second = cameraMatrix.inv() * vec_second;
				Mat second_corner = vec_second;
				move_and_norm_vector(second_corner, new_transform_from_cam_to_map, params.camera_position,
														 z_goal_top_side_markers);
				corners_in_map_coords.push_back(second_corner);
			}
		}

		else if ((marker.id_ == 4) || (marker.id_ == 7))
		{
			for (uint8_t i = 0; i < 2; i++)
			{
				Mat vec_second = (Mat_<double>(3, 1) << new_point[i].x, new_point[i].y, 1);
				vec_second = cameraMatrix.inv() * vec_second;
				Mat second_corner = vec_second;
				move_and_norm_vector(second_corner, new_transform_from_cam_to_map, params.camera_position,
														 z_goal_top_side_enemys);
				corners_in_map_coords.push_back(second_corner);
			}
		}
		/*** NORMAL MARKERS ***/
		else
		{
			for (uint8_t i = 0; i < 2; i++)
			{
				Mat vec_second = (Mat_<double>(3, 1) << new_point[i].x, new_point[i].y, 1);
				vec_second = cameraMatrix.inv() * vec_second;
				Mat second_corner = vec_second;
				// if enemy robot
				if ((marker.id_ == params.big_enemy_ids[0]) || (marker.id_ == params.big_enemy_ids[1]) ||
						(marker.id_ == params.big_enemy_ids[2]) || (marker.id_ == params.big_enemy_ids[3]) ||
						(marker.id_ == params.small_enemy_ids[0]) || (marker.id_ == params.small_enemy_ids[1]) ||
						(marker.id_ == params.small_enemy_ids[2]) || (marker.id_ == params.small_enemy_ids[3]))
				{
					move_and_norm_vector(second_corner, new_transform_from_cam_to_map, params.camera_position,
															 z_goal_enemy_robots);
				}
				else
				// if sber robot
				{
					move_and_norm_vector(second_corner, new_transform_from_cam_to_map, params.camera_position,
															 z_goal_sber_robots);
				}
				corners_in_map_coords.push_back(second_corner);
			}
		}
		// calc average from all corners to get real marker center
		float angle = 0;
		if (corners_in_map_coords.size() > 2)
		{
			average_position = (Mat_<double>(2, 1) << (corners_in_map_coords[0].at<double>(0, 0) +
																								 corners_in_map_coords[1].at<double>(0, 0) +
																								 corners_in_map_coords[2].at<double>(0, 0) +
																								 corners_in_map_coords[3].at<double>(0, 0)) /
																										4,
													(corners_in_map_coords[0].at<double>(1, 0) + corners_in_map_coords[1].at<double>(1, 0) +
													 corners_in_map_coords[0].at<double>(2, 0) + corners_in_map_coords[1].at<double>(3, 0)) /
															4);
			marker.setTvec(cv::Vec3d(average_position.at<double>(0, 0), average_position.at<double>(1, 0), 0));
			float dot = corners_in_map_coords[0].at<double>(0, 0) - corners_in_map_coords[1].at<double>(0, 0);
			float det = corners_in_map_coords[0].at<double>(1, 0) - corners_in_map_coords[1].at<double>(1, 0);
			angle = atan2(dot, det);

			float dot1 = corners_in_map_coords[0].at<double>(0, 0) - corners_in_map_coords[1].at<double>(0, 0);
			float det1 = corners_in_map_coords[0].at<double>(1, 0) - corners_in_map_coords[1].at<double>(1, 0);
			float angle1 = atan2(dot1, det1);
			if (abs(angle - angle1) < 0.2)
			{
				angle = (angle + angle1) / 2;
			}
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
		marker.setRvec(angle_vec);

		if ((marker.id_ == 2))
		{
			Vec3d angle_vec(0.0, 0.0, -(angle - 3.14));
			marker.setRvec(angle_vec);
			center_ = calcCenterOfRobot_by_disp(angle_vec, marker.getTvec(), Vec3d(0.02, -0.07, 0));
		}

		else if ((marker.id_ == 8))
		{
			Vec3d angle_vec(0.0, 0.0, -(angle - 3.14));
			marker.setRvec(angle_vec);
			center_ = calcCenterOfRobot_by_disp(angle_vec, marker.getTvec(), Vec3d(0.02, 0.07, 0));
		}

		else if ((marker.id_ == 6))
		{
			Vec3d angle_vec(0.0, 0.0, -(angle - 3.14));
			marker.setRvec(angle_vec);
			center_ = calcCenterOfRobot_by_disp(angle_vec, marker.getTvec(), Vec3d(0.02, -0.07, 0));
		}

		else if ((marker.id_ == 1))
		{
			Vec3d angle_vec(0.0, 0.0, -(angle - 3.14));
			marker.setRvec(angle_vec);
			center_ = calcCenterOfRobot_by_disp(angle_vec, marker.getTvec(), Vec3d(0.03, 0.07, 0));
		}

		else if (marker.id_ == 3)
		{
			Vec3d angle_vec(0.0, 0.0, -(angle - 3.14));
			marker.setRvec(angle_vec);
			center_ = calcCenterOfRobot_by_disp(angle_vec, marker.getTvec(), Vec3d(0.09, 0, 0));
		}
		else if (marker.id_ == 4)

		{
			Vec3d angle_vec(0.0, 0.0, -(angle - 3.14));

			marker.setRvec(angle_vec);

			center_ = calcCenterOfRobot_by_disp(angle_vec, marker.getTvec(), Vec3d(0.03, 0., 0));
		}

		else if (marker.id_ == 45)
		{
			Vec3d angle_vec(0.0, 0.0, (angle + 3.14 / 2.0));

			marker.setRvec(angle_vec);

			center_ = calcCenterOfRobot_by_disp(angle_vec, marker.getTvec(), Vec3d(0.03, 0., 0));
		}

		else if (marker.id_ == 41)
		{
			Vec3d angle_vec(0.0, 0.0, (angle + 3.14 / 2.0));

			marker.setRvec(angle_vec);

			center_ = calcCenterOfRobot_by_disp(angle_vec, marker.getTvec(), Vec3d(0.03, 0., 0));
		}

		else if (marker.id_ == 31)
		{
			Vec3d angle_vec(0.0, 0.0, (angle + 3.14 / 2.0));

			marker.setRvec(angle_vec);

			center_ = calcCenterOfRobot_by_disp(angle_vec, marker.getTvec(), Vec3d(0.03, 0., 0));
		}

		else
		{
			center_ = calcCenterOfCube(angle_vec, marker.getTvec());
		}
	}
	return true;
}
