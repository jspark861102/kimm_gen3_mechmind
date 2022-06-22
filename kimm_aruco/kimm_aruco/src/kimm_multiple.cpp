/*****************************
 Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are
 permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of
 conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list
 of conditions and the following disclaimer in the documentation and/or other materials
 provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are those of the
 authors and should not be interpreted as representing official policies, either expressed
 or implied, of Rafael Muñoz Salinas.
 ********************************/

/**
 * @file simple_double.cpp
 * @author Bence Magyar
 * @date June 2012
 * @version 0.1
 * @brief ROS version of the example named "simple" in the ArUco software package.
 */

#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <kimm_aruco/aruco_ros_utils.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <dynamic_reconfigure/server.h>
#include <kimm_aruco/ArucoThresholdConfig.h>

#include <std_msgs/String.h>
#include <camera_calibration_parsers/parse_ini.h>

cv::Mat inImage;
aruco::CameraParameters camParam;
bool useRectifiedImages, normalizeImageIllumination, is_image;
int dctComponentsToRemove;
aruco::MarkerDetector mDetector;
std::vector<aruco::Marker> markers;
ros::Subscriber cam_info_sub;
ros::Subscriber call_from_robot_sub;
bool cam_info_received;
image_transport::Publisher image_pub;
// image_transport::Publisher debug_pub;
ros::Publisher pose_pub1;
ros::Publisher pose_pub2;
std::string child_name1;
std::string parent_name;
std::string child_name2;
std::string reference_name;
std::string calib_filename;

bool isrobotcall;
bool rotate_marker_axis_for_ros;

double marker_size;
int marker_id1;
int marker_id2;

#define PUB 1

void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    double ticksBefore = cv::getTickCount();
    static tf::TransformBroadcaster br;    
    static tf::TransformListener tf_listener;

    if (cam_info_received)
    {
      ros::Time curr_stamp = msg->header.stamp;
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        inImage = cv_ptr->image;

        if (normalizeImageIllumination)
        {
          ROS_WARN("normalizeImageIllumination is unimplemented!");
  //        cv::Mat inImageNorm;
  //        pal_vision_util::dctNormalization(inImage, inImageNorm, dctComponentsToRemove);
  //        inImage = inImageNorm;
        }

        if (isrobotcall)
        {
          // detection results will go into "markers"
          markers.clear();
          // ok, let's detect
          mDetector.detect(inImage, markers, camParam, marker_size, false);
          // for each marker, draw info and its boundaries in the image
          for (unsigned int i = 0; i < markers.size(); ++i)
          {
            // only publishing the selected marker
            if (markers[i].id == marker_id1)
            {
              // ROS_WARN("reference_name is %s", reference_name.data());

              //broadcasting tf (camera to marker)
              tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i], rotate_marker_axis_for_ros);
              br.sendTransform(tf::StampedTransform(transform, curr_stamp, parent_name, child_name1));
              geometry_msgs::Pose poseMsg;

              if (is_image)
              {
                tf::poseTFToMsg(transform, poseMsg);       
                pose_pub1.publish(poseMsg);
              }
              else
              {
                //publish marker pose w.r.t. reference_name (ex:odom, baselink, camera_color_optical_frame...)
                tf::StampedTransform reference_to_marker;
                try{
                  tf_listener.waitForTransform(reference_name, child_name1, ros::Time(0), ros::Duration(1.0));
                  tf_listener.lookupTransform(reference_name, child_name1, ros::Time(0), reference_to_marker);
                  tf::poseTFToMsg(reference_to_marker, poseMsg);
                  pose_pub1.publish(poseMsg);
                }
                catch (tf::TransformException &ex) {
                    continue;
                }
              }                        
            }
            else if (markers[i].id == marker_id2)
            {
              //broadcasting tf (camera to marker)
              tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i], rotate_marker_axis_for_ros);
              br.sendTransform(tf::StampedTransform(transform, curr_stamp, parent_name, child_name2));
              geometry_msgs::Pose poseMsg;

              if (is_image)
              {
                tf::poseTFToMsg(transform, poseMsg);       
                pose_pub2.publish(poseMsg);
              }
              else
              {
                //publish marker pose w.r.t. reference_name (ex:odom, baselink, camera_color_optical_frame...)
                tf::StampedTransform reference_to_marker;
                try{
                  tf_listener.waitForTransform(reference_name, child_name2, ros::Time(0), ros::Duration(1.0));
                  tf_listener.lookupTransform(reference_name, child_name2, ros::Time(0), reference_to_marker);
                  tf::poseTFToMsg(reference_to_marker, poseMsg);
                  pose_pub2.publish(poseMsg);
                }
                catch (tf::TransformException &ex) {
                    continue;
                }
              }              
            }
            // but drawing all the detected markers
            markers[i].draw(inImage, cv::Scalar(0, 0, 255), 2);
          }

          // paint a circle in the center of the image
          cv::circle(inImage, cv::Point(inImage.cols / 2, inImage.rows / 2), 4, cv::Scalar(0, 255, 0), 1);  

          // draw a 3D cube in each marker if there is 3D info
          if (camParam.isValid() && marker_size != -1)
          {
            for (unsigned int i = 0; i < markers.size(); ++i)
            {
              aruco::CvDrawingUtils::draw3dCube(inImage, markers[i], camParam);
            }
          }

        }

        if (image_pub.getNumSubscribers() > 0)
        {
          // show input with augmented information
          cv_bridge::CvImage out_msg;
          out_msg.header.stamp = curr_stamp;
          out_msg.encoding = sensor_msgs::image_encodings::RGB8;
          out_msg.image = inImage;
          image_pub.publish(out_msg.toImageMsg());
        }
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
    }
#if defined(PUB)
  isrobotcall = true;
#else
  isrobotcall = false;
#endif
}

// wait for one camerainfo, then shut down that subscriber
void cam_info_callback(const sensor_msgs::CameraInfo &msg)
{
  camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);
  cam_info_received = true;
  cam_info_sub.shutdown();
}

void call_from_robot_callback(const std_msgs::String &msg)
{
  std_msgs::String a; 
  a.data = "aruco_call";
  if (msg.data.compare(a.data) == 0)
    isrobotcall = true;

#if (!PUB)
  ROS_WARN("isrobotcall is %d", isrobotcall);
#endif
}

bool parseCalibrationFile(std::string calib_filename)
{
  sensor_msgs::CameraInfo camera_calibration_data;
  std::string camera_name = "camera";

  camera_calibration_parsers::readCalibrationIni(calib_filename, camera_name, camera_calibration_data);

  // Alocation of memory for calibration data
  cv::Mat  *intrinsics       = new(cv::Mat)(3, 3, CV_64F);
  cv::Mat  *distortion_coeff = new(cv::Mat)(5, 1, CV_64F);
  cv::Size *image_size       = new(cv::Size);

  image_size->width = camera_calibration_data.width;
  image_size->height = camera_calibration_data.height;

  for(size_t i = 0; i < 3; i++)
    for(size_t j = 0; j < 3; j++)
    intrinsics->at<double>(i,j) = camera_calibration_data.K.at(3*i+j);

  for(size_t i = 0; i < 5; i++)
    distortion_coeff->at<double>(i,0) = camera_calibration_data.D.at(i);

  ROS_DEBUG_STREAM("Image width: " << image_size->width);
  ROS_DEBUG_STREAM("Image height: " << image_size->height);
  ROS_DEBUG_STREAM("Intrinsics:" << std::endl << *intrinsics);
  ROS_DEBUG_STREAM("Distortion: " << *distortion_coeff);


  //Load parameters to aruco_calib_param_ for aruco detection
  camParam.setParams(*intrinsics, *distortion_coeff, *image_size);

  //Simple check if calibration data meets expected values
  if ((intrinsics->at<double>(2,2) == 1) && (distortion_coeff->at<double>(0,4) == 0))
  {
    ROS_INFO_STREAM("Calibration data loaded successfully");
    return true;
  }
  else
  {
    ROS_WARN("Wrong calibration data, check calibration file and filepath");
    return false;
  }
}

void reconf_callback(aruco_ros::ArucoThresholdConfig &config, std::uint32_t level)
{
  mDetector.setDetectionMode(aruco::DetectionMode(config.detection_mode), config.min_image_size);
  normalizeImageIllumination = config.normalizeImage;
  dctComponentsToRemove = config.dctComponentsToRemove;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kimm_multiple");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);

  dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig> server;
  dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig>::CallbackType f_;
  f_ = boost::bind(&reconf_callback, _1, _2);
  server.setCallback(f_);

  normalizeImageIllumination = false;

#if (PUB)
  isrobotcall = true;
#else
  isrobotcall = false;
#endif

  rotate_marker_axis_for_ros = true;

  nh.param<bool>("image_is_rectified", useRectifiedImages, true);
  ROS_INFO_STREAM("Image is rectified: " << useRectifiedImages);

  image_transport::Subscriber image_sub = it.subscribe("/image", 1, &image_callback);

  call_from_robot_sub =  nh.subscribe("/kimm_gen3_pnp/aruco_call", 1, &call_from_robot_callback);

  cam_info_received = false;
  image_pub = it.advertise("result", 1);
  // debug_pub = it.advertise("debug", 1);
  pose_pub1 = nh.advertise<geometry_msgs::Pose>("pose", 100);
  pose_pub2 = nh.advertise<geometry_msgs::Pose>("pose2", 100);

  nh.param<double>("marker_size", marker_size, 0.05);
  nh.param<int>("marker_id1", marker_id1, 582);
  nh.param<int>("marker_id2", marker_id2, 26);
  nh.param<bool>("normalizeImage", normalizeImageIllumination, true);
  nh.param<int>("dct_components_to_remove", dctComponentsToRemove, 2);
  if (dctComponentsToRemove == 0)
    normalizeImageIllumination = false;
  nh.param<std::string>("reference_name", reference_name, "");    
  nh.param<std::string>("parent_name", parent_name, "");
  nh.param<std::string>("child_name1", child_name1, "");
  nh.param<std::string>("child_name2", child_name2, "");
  nh.param<std::string>("calibration_file", calib_filename, "");
  nh.param<bool>("is_image", is_image, false); 
  nh.param<bool>("rotate_marker_axis_for_ros", rotate_marker_axis_for_ros, true);   

  if (is_image)
  {
    parseCalibrationFile(calib_filename);
    cam_info_received = true;
  }
  else
    cam_info_sub = nh.subscribe("/camera_info", 1, &cam_info_callback);
  
  if (parent_name == "" || child_name1 == "" || child_name2 == "")
  {
    ROS_ERROR("parent_name and/or child_name was not set!");
    return -1;
  }
  
  ROS_INFO("ArUco node started with marker size of %f meters and marker ids to track: %d, %d", marker_size, marker_id1,
           marker_id2);
  ROS_INFO("ArUco node will publish pose to TF with (%s, %s) and (%s, %s) as (parent,child).", parent_name.c_str(),
           child_name1.c_str(), parent_name.c_str(), child_name2.c_str());

  ros::spin();
}
