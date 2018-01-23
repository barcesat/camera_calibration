#pragma once

#include <ros/ros.h>
#include <ros/console.h>

#include<opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

namespace camera_calibration {


	class VerifyCameraCalibration
	{
		public:
			VerifyCameraCalibration();


		private:

			// ROS
			ros::NodeHandle nh_;
			image_transport::CameraSubscriber sub_video_;
			image_transport::Publisher pub_original_image_;
			image_transport::Publisher pub_corrected_image_;   // undistorted image


		// camera parameters
		ros::Time timestamp_frame_;
		cv::Mat camera_matrix_;
		cv::Mat dist_coeff_;
		bool cinfo_received_ = false;  // camera info received once or no


		// frames
		cv::Mat original_img_;
		cv::Mat corrected_img_;

		void callback_sub_video(const sensor_msgs::ImageConstPtr& data, const sensor_msgs::CameraInfoConstPtr& cinfo);

		void publish_video();


	};







}

