#include "verify_camera_calibration.h"

namespace camera_calibration {


VerifyCameraCalibration::VerifyCameraCalibration(){




	// ROS communication
	image_transport::ImageTransport it(nh_);
	sub_video_ = it.subscribeCamera("video",10, &VerifyCameraCalibration::callback_sub_video, this);
	pub_corrected_image_ = it.advertise("corrected_image",10);


}


void VerifyCameraCalibration::callback_sub_video(const sensor_msgs::ImageConstPtr& data, const sensor_msgs::CameraInfoConstPtr& cinfo){
	// printf("here \n");
	// get newest image
	try{
		original_img_ = cv_bridge::toCvCopy(data,sensor_msgs::image_encodings::BGR8)->image;
	}
	catch(cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge execption: %s", e.what());
		return;
	}

	// get camera frame timestamp
	timestamp_frame_ = data->header.stamp;

	// save the camera parameters one time
	if(!cinfo_received_)
	{
		// camera_matrix_ (K) is 3x3
		// dist_coeff_    (D) is a column vector of 4, 5, or 8 elements
	    camera_matrix_ = cv::Mat(              3, 3, CV_64FC1);
    	dist_coeff_    = cv::Mat(cinfo->D.size(), 1, CV_64FC1);

	    // convert rosmsg vectors to cv::Mat
	    for(int i=0; i<9; i++)
	      camera_matrix_.at<double>(i/3, i%3) = cinfo->K[i];

	    for(int i=0; i<cinfo->D.size(); i++)
	      dist_coeff_.at<double>(i, 0) = cinfo->D[i];

	  cinfo_received_ = true;

	}

	publish_video();


}

void VerifyCameraCalibration::publish_video(){

	// undistort the camera
	cv::undistort(original_img_, corrected_img_, camera_matrix_, dist_coeff_);


	// publish the images
	sensor_msgs::ImagePtr corrected_img;
	corrected_img = cv_bridge::CvImage(std_msgs::Header(),sensor_msgs::image_encodings::BGR8, corrected_img_).toImageMsg();
	
	pub_corrected_image_.publish(corrected_img);



}







}