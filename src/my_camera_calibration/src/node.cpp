#include <ros/ros.h>
#include "verify_camera_calibration.h"

int main(int argc, char** argv){
	// start node
	ros::init(argc, argv, "verify_camera_calibration_node");


	camera_calibration::VerifyCameraCalibration verify_camera_calibration;


	ros::spin();
	return 0;
}