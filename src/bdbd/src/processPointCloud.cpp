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
#include <sensor_msgs/point_cloud_conversion.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <chrono>
#include <vector>
#include <iomanip>
#include <algorithm>

class PointsProcessor
{
  ros::NodeHandle nh_;
  ros::Subscriber pc_sub;

public:
  PointsProcessor()
  {
    // Subscribe to input video feed and publish output video feed
    pc_sub = nh_.subscribe("/sr305/depth/color/points", 1, &PointsProcessor::pcCb, this);
  }

  ~PointsProcessor()
  {
  }

  void pcCb(const sensor_msgs::PointCloud2ConstPtr& pc2msg)
  {
    //std::cout << "Shape: " << pc2msg->height << " " << pc2msg->width << "\n";
    static int frame_count = 0;
    auto start = std::chrono::steady_clock::now();
    sensor_msgs::PointCloud pcmsg;
    sensor_msgs::convertPointCloud2ToPointCloud(*pc2msg, pcmsg);
    auto middle = std::chrono::steady_clock::now();
    // std::vector<float> xPoints;
    std::vector<geometry_msgs::Point32> clPoints;
    frame_count++;
    if (frame_count % 30 != 0) {
      return;
    }

    for (int i = 0; i < pcmsg.points.size(); ++i)
    {
      auto x = pcmsg.points[i].x;
      auto y = pcmsg.points[i].y;
      auto z = pcmsg.points[i].z;
      // if (i % 10 == 0)
      //if (x > -.01 && x < .01)
      {
        // std::cout << "X: " << x << " Y: " << y << " Z: " << z << "\n";
        // xPoints.push_back(z);
        clPoints.push_back(pcmsg.points[i]);
      }
    }

    auto end = std::chrono::steady_clock::now();
    std::cout << std::fixed << std::showpos;
    std::cout << "cl point count: " << clPoints.size() << "\n";
    std::cout << "\n";
    /*
    std::sort(std::begin(xPoints), std::end(xPoints), [](float x1, float x2) -> bool
    {
        return x1 < x2;
    });
    /* */
    std::sort(std::begin(clPoints), std::end(clPoints), [](const geometry_msgs::Point32 &p1, const geometry_msgs::Point32 &p2) -> bool
    {
        return p1.z < p2.z;
    });

    /* */
    // show 20 points
    auto delta = clPoints.size() / 20;
    // auto delta = 1;
    for (auto i = 0; i < clPoints.size(); i+=delta) {
        std::cout << "(" << std::setw(5) << i << ") point: (" << clPoints[i].x << " , " << clPoints[i].y << " , " << clPoints[i].z << ")\n";
        // std::cout << xPoints[i] << "\n";
    }
  
    // show range of x at a specific distance
    std::vector<geometry_msgs::Point32> d50Points;
    for (auto& point: clPoints) {
      if (point.z > 0.495 && point.z < 0.505) {
        d50Points.push_back(point);
      }
    }
    std::cout << "d50 point count: " << d50Points.size() << "\n";
    // Sort these limited range values by x
    std::sort(std::begin(d50Points), std::end(d50Points), [](const geometry_msgs::Point32 &p1, const geometry_msgs::Point32 &p2) -> bool
    {
        return p1.x < p2.x;
    });
    // Print the range
    for (auto i = 0; i < d50Points.size(); i += d50Points.size() / 20) {
      auto point = d50Points[i];
      std::cout << " (" << point.x << "," << point.y << "," << point.z << ")";
    }
    std::cout << "\n";

    std::chrono::duration<double> elapsed_seconds = end-start;
    std::chrono::duration<double> middle_seconds = middle-start;
    std::cout << "middle time: " << middle_seconds.count() << " elapsed time: " << elapsed_seconds.count() << "s\n";

  }

};

int main(int argc, char **argv)
{
/**
 * The ros::init() function needs to see argc and argv so that it can perform
 * any ROS arguments and name remapping that were provided at the command line.
*/
    ros::init(argc, argv, "processPointCloud");
    PointsProcessor pp;
    ros::spin();
    return(0);
}