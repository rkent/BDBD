/**
  This node is mostly a demo to show getting a distance in C++ from the depth image, manipulating
  that image, and converting into a ROS Image message.
*/
#include "ros/ros.h"
// #include "std_msgs/String.h"
// #include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

static const std::string OPENCV_WINDOW = "Image window";


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  double filteredMax = 500.;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/sr305/depth/image_rect_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg);
      // ROS_INFO("Image size: %i %i", cv_ptr->image.rows, cv_ptr->image.cols);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    int col2 = cv_ptr->image.cols/2;
    int row2 = cv_ptr->image.rows/2;
    auto center = cv_ptr->image.at<short>(row2, col2);
    float fcenter = center * .00125;
    ROS_INFO("Center pixel: %i, Center distance: %f (meters) ", center, fcenter);
    // Normalize for visualization
    /*
    cv::Scalar mean = 0.;
    cv::Scalar std = 0.;
    cv::meanStdDev(cv_ptr->image, mean, std);
    filteredMax = mean[0] + 3.*std[0];
    */
    double newMin = 0.0;
    double newMax = 0.0;
    cv::minMaxIdx(cv_ptr->image, &newMin, &newMax);
    double frate = 0.02;
    filteredMax = (1. - frate) * filteredMax + frate * newMax;
    if (filteredMax <= 0.0) {
      filteredMax = 1.;
    }
  
    cv::Mat normed = (60000. / filteredMax) * cv_ptr->image;
    // cv::normalize(cv_ptr->image, normed, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    //auto ncenter = normed.at<uchar>(col2, row2);
    //ROS_INFO("NCenter: %i", ncenter);

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, normed);
    cv::waitKey(3);

    // Output modified video stream
    sensor_msgs::ImagePtr outMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", normed).toImageMsg();
    image_pub_.publish(outMsg);
  }
};

int main(int argc, char **argv)
{
/**
 * The ros::init() function needs to see argc and argv so that it can perform
 * any ROS arguments and name remapping that were provided at the command line.
*/
    ros::init(argc, argv, "showDepthImage");
    ImageConverter ic;
    ros::spin();
    return(0);
}