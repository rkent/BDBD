/**
    This node uses the SR305 camera to detect local blocking. The view is divided into three zones,
    left, center, and right. Within each zone, we report 1) maximum length of clear path confirmed,
    and 2) length to any detected obstacles.
*/
#include "ros/ros.h"
#include <ros/console.h>
// #include "std_msgs/String.h"
// #include <sstream>
#include <sensor_msgs/point_cloud_conversion.h>
#include <bdbd/RoadBlocking.h>
#include <iostream>
#include <chrono>
#include <vector>
#include <iomanip>
#include <algorithm>
#include <string>

using namespace std;

class DetectBlocking
{
    ros::NodeHandle nh_;
    ros::Subscriber pc_sub;
    ros::Publisher rb_pub;
    const float min_depth = .25;
    const float max_depth = .55;
    const float min_height = .08;
    const float max_height = .13;
    const int tan_divisions = 12;
    const int depth_divisions = 12;
    const float min_tan = -0.45;
    const float max_tan = 0.45;
    int frame_count = 0;

    public:
    DetectBlocking()
    {
        // Subscribe to input video feed and publish output feed
        pc_sub = nh_.subscribe("/sr305/depth/color/points", 1, &DetectBlocking::pcCb, this);
        rb_pub = nh_.advertise<bdbd::RoadBlocking>("detectBlocking/roadBlocking", 1);
    }

    ~DetectBlocking()
    {
    }

    void pcCb(const sensor_msgs::PointCloud2ConstPtr& pc2msg)
    {
        auto start = std::chrono::steady_clock::now();
        sensor_msgs::PointCloud pcmsg;
        sensor_msgs::convertPointCloud2ToPointCloud(*pc2msg, pcmsg);
        auto middle = std::chrono::steady_clock::now();
        std::vector<std::vector<std::vector<geometry_msgs::Point32>>> pointMatrix(tan_divisions, vector<vector<geometry_msgs::Point32>>(depth_divisions));

        frame_count++;

        // Divide all points into buckets by depth, tangent
        for (int i = 0; i < pcmsg.points.size(); i++)
        {
            auto point = pcmsg.points[i];
            auto z = point.z;
            if (z == 0.0) {
                continue;
            }
            auto y = point.y;
            auto x = point.x;

            int iz = 1 + int(float(depth_divisions - 2) * (z - min_depth) / (max_depth - min_depth));
            if (iz < 0) {
                iz = 0;
            } else if (iz > depth_divisions - 1) {
                iz = depth_divisions - 1;
            }

            float a = x / z;
            int ja = 1 + int(float(tan_divisions - 2) * (a - min_tan) / (max_tan - min_tan));
            if (ja < 0) {
                ja = 0;
            } else if (ja > tan_divisions - 1) {
                ja = tan_divisions - 1;
            }

            pointMatrix[iz][ja].push_back(point);
        }

        // Detect 1) distance to dropoff, and 2) distance to obstacle in left, center, and right

        // left is region 0, center 1, right 2
        vector<float> road_depths = {0.0, 0.0, 0.0};
        vector<float> obstacle_depths = {2.0, 2.0, 2.0};
        geometry_msgs::Point32 opoint;

        for (int i = 0; i < depth_divisions; i++) {
            
            float least_region_depths[3] = {100.0, 100.0, 100.0};
            bool road_in_regions[3] = {true, true, true};
            for (int j = 0; j < tan_divisions; j++) {
                auto points = pointMatrix[i][j];

                int region;
                if (j < tan_divisions / 3)
                    region = 0;
                else if (j  > 2 * tan_divisions / 3)
                    region = 2;
                else
                    region = 1;

                float best_cell_depth = 0.0;
                for (auto& point: points) {
                    if (point.z == 0.0) {
                        continue; // not a valid datapoint
                    }
                    if (point.y > min_height && point.y < max_height) {
                        // This point is valid road
                        if (point.z > best_cell_depth) {
                            best_cell_depth = point.z;
                        }
                    } else {
                        // This point is an obstacle
                        if (obstacle_depths[region] == 0.0 || obstacle_depths[region] > point.z) {
                            obstacle_depths[region] = point.z;
                            if (opoint.z == 0.0)
                                opoint = point;
                        }
                    }
                }
                if (best_cell_depth == 0.0) { // no road found in cell!
                    road_in_regions[region] = false;
                } else if (best_cell_depth < least_region_depths[region]) {
                    least_region_depths[region] = best_cell_depth;
                }
            }
            for (int k = 0; k < 3; k++) {
                if (road_in_regions[k]) {
                    road_depths[k] = least_region_depths[k];
                }
            }
        }

        bdbd::RoadBlocking rb;
        rb.leftRoadDepth = road_depths[0];
        rb.centerRoadDepth = road_depths[1];
        rb.rightRoadDepth = road_depths[2];
        rb.leftObstacleDepth = obstacle_depths[0];
        rb.centerObstacleDepth = obstacle_depths[1];
        rb.rightObstacleDepth = obstacle_depths[2];
        rb_pub.publish(rb);

        if (frame_count % 10 != 0) {
            return;
        }

/*
        std::cout << std::fixed;
        auto end = std::chrono::steady_clock::now();

        std::chrono::duration<double> elapsed_seconds = end-start;
        std::chrono::duration<double> middle_seconds = middle-start;
        std::cout << "\nmiddle time: " << middle_seconds.count() << " elapsed time: " << elapsed_seconds.count() << "s\n";

        for (int k = 0; k < 3; k++) {
            string name = k == 0 ? "left" : k == 1 ? "center" : "right";
            cout << "Region " << name << " road_depths: " << road_depths[k] << " Obstacle depths:" << obstacle_depths[k] << "\n";
        }

/*
        if (opoint.z != 0.0) {
            cout << "First obstacle: " << opoint.x << " " << opoint.y << " " << opoint.z << "\n";
        }

        // vertical slice
        cout << "Vertical counts: ";
        for (int i = 0; i < depth_divisions; i++) {
            cout << setw(5) << pointMatrix[i][tan_divisions/2].size();
        }
        cout << "\n";

        // horizontal slice
        cout << "Horizontal counts: ";
        for (int j = 0; j < tan_divisions; j++) {
            cout << setw(5) << pointMatrix[depth_divisions/2][j].size();
        }
        cout << "\n";
        // search for low-count bins
        for (int i = 0; i < depth_divisions; i++) {
            for (int j = 0; j < tan_divisions; j++) {
                auto points = pointMatrix[i][j];
                if (points.size() == 0) {
                    cout << "Zero count at " << i << "," << j << "\n";
                }
            }
        }
/* */
    }
};

int main(int argc, char **argv)
{
/**
* The ros::init() function needs to see argc and argv so that it can perform
* any ROS arguments and name remapping that were provided at the command line.
*/
    ros::init(argc, argv, "detectBlocking");
    ROS_INFO("detectBlocking starting up");
    DetectBlocking pp;
    ros::spin();
    return(0);
}