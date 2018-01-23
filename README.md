# camera_calibration

This repository contains the custom ROS package my_camera_calibration. 
It is used to calibrate a usb_camera or other camera specified by the user using the ROS camera calibration package.
After calibrating, the the original image and undistorted image can be viewed. 

## Installation and Setup

Since this package depends on ROS, ROS must be installed. If ROS is not installed, see instructions on how to install it.
Note, ROS is not supported on every platform. These instructions are for ROS kinetic installed on Ubuntu.
- [Install ROS](http://wiki.ros.org/kinetic/Installation)

If you are going to use a USB camera (like a webcam), you will need to install the ROS [usb_cam](http://wiki.ros.org/usb_cam) package. 
To install it via the command line, use
```bash
$ sudo apt install ros-kinetic-usb-cam
```

If you are going to be using a different camera that requries an additional driver, you will need to find a ROS package that 
supports that camera. A list of some available cameras can be seen here [Sensors/Cameras](http://wiki.ros.org/Sensors/Cameras). 
Note: there are many other camera drivers that have ROS wrappers; just google them. 

Once you have ROS installed and a ROS package for whatever camera you are using, install this repository in any directory. 

```bash
$ git clone https://github.com/pmarke/camera_calibration.git
```
Now navigate to the workspace.

```bash
$ cd camera_calibration
```
Build the packages using catkin_make. 

```bash
$ catkin_make
```
You packages should be built, and you there should be three subdirectories: build, src, and devel. The devel folder contains
the built executible files, and a bash script that needs to be run. This bash script tells ROS where this package is. To
run the script

```bash
$ source devel/setup.bash
```
You will have to do this every time you open a new terminal. If you dont wan't to, you can add the command to your basrc 
script. 

```bash
$ echo "source <path_to_workspace>camera_calibration/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```
Now every time you open up a new terminal, the path to this package will be added to the environment variable ROS_PATH.

Everything should now be set up. Before using the package, I will explain a few things. 

## ROS Launch Files

ROS Launch Files are used to run multiple executible files, pass in arguments to those files, and much more with 
one simple command. To learn more, go [here](http://wiki.ros.org/roslaunch/XML).This repository contains four launch files located in /camera_calibration/src/my_camera_calibration/launch.
The four launch files are *webcam.launch camera.launch, camera_calibration.launch,* and *verify_calibration.launch*. 

#### If not using a webcam

If you are not using a webcam, then you will need to modify the *camera.launch* file to run your specific camera. The example 
in this repository is for a point gray camera. To find one for your camera, go to the ROS package directory of your specific 
camera. An easy way to do this is by using the ROS command **roscd**.

```bash
$ roscd <package_name>
```
example
```bash
$ pointgrey_camera_driver
```
Inside the directory you should see a launch folder. Navigate into it. There should be a launch file for your camera. Use 
this launch file to modify the camera.launch file in /camera_calibration/src/my_camera_calibration/launch. Note: your arguments
might change depending on your camera. If so, you will need to make appropriate changes to camera_calibration.launch and verify_calibration.launch
to reflect your changes. 

#### If using webcam

If you are using a webcam, you will need to modify the webcam.launch file 
to meet your webcam's specifications. Look for the section

```xml
		<node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen" >
		    <param name="video_device" value="/dev/video0" />
		    <param name="image_width" value="640" />
		    <param name="image_height" value="480" />
		    <param name="pixel_format" value="yuyv" />          <!-- possible values are mjpeg, yuyv, uyvy -->
		    <param name="camera_frame_id" value="usb_cam" />
		    <param name="io_method" value="mmap"/>              <!-- possible valuse are mmap, read, userptr -->
		    <param name="camera_name" value="narrow_stereo"/>  <!-- Must be the same as the camera name in the 																					webcam_intrinsic_parameters.yaml -->
		    <param name="camera_info_url" if="$(arg calibrated)" value="file://$(find my_camera_calibration)/param/webcam_intrinsic_parameters.yaml"/>
		    <remap from="usb_cam/image_raw" to="$(arg topic_name)"/>
  		</node>
```

Edit the values (ignore camera_name and camera_info_url for now) to reflect your webcam's specifications. That's it. 

### Calibrate Camera

To calibrate the camera, use the *camera_calibration.launch* file. There are some arguments that you will need to change. 
First look at this section. 
```xml

	<!-- Arguments if using rospackage camera_calibration -->
	<arg name="use_webcam" default = "true"/>   <!-- Use the webcam on your computer instead of a different camera. -->
	<arg name="topic_name" default="image_raw"/>   <!-- The topic for the video. DO NOT CHANGE UNLESS YOU ARE FAMILIAR WITH NAMESPACES-->
	<arg name="frame_rate" default="30.0"/>     <!-- Frame rate of camera, not webcam -->

```

I suggest not changing *topic_name* unless you want to have fun playing with namespaces. Adjust the other parameters as needed.

*camera_calibration.launch uses the python script [cameracalibrator.py](http://wiki.ros.org/camera_calibration) to calibrate 
the camera. You will need to adjust the arguments passed into the script according to the chessboard you are using. 
Below is what the section of interest looks like. Note: change only *size* and *square*. 

```xml

			<!-- camera calibration node. This node will call subscibe to the topic "topic name" to get images
			thses images are used in the calibration method -->
		<!-- 
			size  : mxn inner corners of the checkerboard
			sqaure: the size of the square in meters
			k     : number of radial distortion coefficients to use
			image : topic name it will subscribe to to get the image -->
		<node pkg="camera_calibration" type="cameracalibrator.py" name="camera_calibrator" output="screen"
			args="
			--size 9x6                   
			--square 0.0235 
			--k-coefficients=3 
			--no-service-check 
			image:=/$(arg topic_name) 
			camera:/my_camera"/>

```

Now that everything is set up. Run the launch file

```bash
$ roslaunch my_camera_calibration camera_calibration.launch
```
##### Troubleshoot 

If you are getting an error that a package can't be found. Verify that the ROS package path contains the path to the
camera_calibration package

```bash
$ printenv | grep ROS_PACKAGE
```
If you do not see it, add it.
```bash
$ source <path_to_workspace>/camera_calibration/devel/setup.bash
```
##### Continue
If everything has worked, a window will pop up displaying your camera's view. To start calibrating, place the chessboard image
in the camera's view. Notice how their are several criteria such as x, y, size, and skew. As you move the camera around, you
will meet the various criteria and you will see a bar change underneath each criteria change from red to yellow then green.
Once each criteria is sufficiently met, hit the calibrate button. Once it is done calibrating, hit the save button. You
now have intrinsic parameters saved in a file. 

Navigate to the file and extract it.

```bash
$ tar -xvzf <file_name>
```
One of the extracted files will have the name *ost.yaml*. This file contains all of the intrinsic parameters.
Move that file to the directory /camera_calibration/src/my_camera/calibration/param/ . If you are using a webcam, rename
the file to *webcam_intrinsic_parameters.yaml*, or if you are using a different camera, rename the file
*camera_intrinsic_parameters.yaml*.

### Verify Camera Calibration. 

You will need to modify the webcam.launch or camera.launch file to tell it the name of your file that contains
all of the camera's instrinsic parameters. 

#### Using webcam

We first need to get the camera name found in the file containing the intrinsic parameters. Open up the *webcam_intrinsic_parameters.yaml*
file, and look for the camera name. 

```yaml
image_width: 640
image_height: 480
camera_name: narrow_stereo
```
Now open the webcam.launch file and change the value of camera_name to the name of your webcam.

```xml
 <param name="camera_name" value="<name of camera>"/> 
```
That's it.

#### Using other camera

Navigate to your camera.launch file and make sure that one of your parameters is *camera_info_url*. This parameter
needs to contain the file path/name of your camera_intrinsic_parameters.yaml file. 

```xml
 	<param name="camera_info_url" if="$(arg calibrated)" value="file://$(find my_camera_calibration)/param/camera_intrinsic_parameters.yaml"/>
```

#### Continue

Now run the *verify_calibration.launch file*

```bash
$ roslaunch my_camera_calibration verify_calibration.launch 
```
Two windows will pop open. One will be the original image and the other will be the undistorted image. Note: if your camera
doesn't have much distortion, you wont see much of a difference. 






