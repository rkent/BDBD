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
#include <std_msgs/String.h>
#include <bdbd/RoadBlocking.h>
#include <iostream>
#include <chrono>
#include <vector>
#include <iomanip>
#include <algorithm>
#include <string>
#include <stdio.h>
#include "libcpp/fitting3d.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>


using namespace std;

class DetectBlocking
{
    ros::NodeHandle nh_;
    ros::Subscriber pc_sub;
    ros::Publisher rb_pub;
    const float min_depth = .25;
    const float max_depth = .55;
    const float min_height = .07;
    const float max_height = .14;
    const float max_distance_road = 0.02; // distance from road plane for allowable road
    const float min_distance_obstacle = 0.03; // distance from road plane to declare an obstacle
    const int tan_divisions = 12;
    const int depth_divisions = 12;
    const float min_tan = -0.45;
    const float max_tan = 0.45;
    const float max_obstacle = 2.0;
    const int max_points = 20000; // The maximum number of PointCloud points to process


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
        std::vector<geometry_msgs::Point32> region_obstacles(3);

        frame_count++;

        int roadPointCount = 0;
        vector<Eigen::Vector3f> eigenPoints;

        geometry_msgs::Point32 lowestRoad, highestRoad, farthestRoad;
        lowestRoad.y = 100.;
        float farthestDistance = 0.0;

        // Divide all points into buckets by depth, tangent
        int indexDelta = (pcmsg.points.size() / max_points) + 1;
        for (int i = 0; i < pcmsg.points.size(); i += indexDelta)
        {
            auto point = pcmsg.points[i];
            auto z = point.z;
            if (z < .001) {
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

            // save points for plane analysis
            if (y > min_height && y < max_height) {
                // This point is possibly valid road
                roadPointCount++;
                // only use some points for plane calculation to reduce CPU load
                if (roadPointCount % 10 == 0) {
                    eigenPoints.push_back(Eigen::Vector3f(x, y, z));
                }
            }
        }

        // Calculate the road plane

        std::pair<Eigen::Vector3f, Eigen::Vector3f> plane;
        if (eigenPoints.size() > 10) {
            plane = best_plane_from_points<Eigen::Vector3f>(eigenPoints);
        } else {
            plane.first = Eigen::Vector3f(0.0, (max_height - min_height)/2., 0.0);
            plane.second = Eigen::Vector3f(0.0, 1.0, 0.0);
        }

        //std::cout << std::fixed;
        //cout << "Plane point: " << plane.first(0) << " " << plane.first(1) << " " << plane.first(2);
        //cout << " normal: " << plane.second(0) << " " << plane.second(1) << " " << plane.second(2) << "\n";

        // Detect 1) distance to dropoff, and 2) distance to obstacle in left, center, and right

        // left is region 0, center 1, right 2
        vector<float> road_depths = {0.0, 0.0, 0.0};
        vector<float> obstacle_depths = {max_obstacle, max_obstacle, max_obstacle};

        for (int i = 0; i < depth_divisions; i++) {
            
            float least_region_depths[3] = {100.0, 100.0, 100.0};
            bool road_in_regions[3] = {true, true, true};
            
            for (int j = 0; j < tan_divisions; j++) {
                vector<geometry_msgs::Point32> obstacles_in_region;
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
                    if (point.z <= 0.001) {
                        continue; // not a valid datapoint
                    }
        
                    // calculate distance from plane to point
                    Eigen::Vector3f p(point.x, point.y, point.z);
                    auto distance = plane.second.dot(plane.first - p);

                    // Is this valid road?
                    if (distance < max_distance_road) {
                        // This point is valid road
                        if (point.y > min_height && point.y < max_height && point.z > min_depth && point.z < max_depth) {
                            // global limits as diagnostics 
                            if (distance > farthestDistance) {
                                farthestRoad = point;
                                farthestDistance = distance;
                            }
                            if (point.y < lowestRoad.y) lowestRoad = point;
                            if (point.y > highestRoad.y) highestRoad = point;
                        }
                        if (point.z > best_cell_depth) {
                            best_cell_depth = point.z;
                        }
                    } else if (distance > min_distance_obstacle) {
                        // This point is an obstacle
                        obstacles_in_region.push_back(point);
                    }
                }
                if (best_cell_depth == 0.0) { // no road found in cell!
                    road_in_regions[region] = false;
                } else if (best_cell_depth < least_region_depths[region]) {
                    least_region_depths[region] = best_cell_depth;
                }

                //if (obstacles_in_region.size() > 0) {
                //    printf("region obstacle count: %li\n", obstacles_in_region.size());
                //}
                // We expect a minimum number of detected obstacle points to declare valid
                if (obstacles_in_region.size() > 100) {
                    for (auto point: obstacles_in_region) {
                        if (obstacle_depths[region] == max_obstacle || obstacle_depths[region] > point.z) {
                            obstacle_depths[region] = point.z;
                            region_obstacles[region] = point;
                        }
                    }
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

        //if (frame_count % 10 != 0) {
        //    return;
        //}

/*
        for (int i = 0; i < 3; i++) {
            auto point = region_obstacles[i];
            if (point.z < 0.30 && point.z >= .001) {
                ROS_INFO("Region %i obstacle at %6.3f, %6.3f, %6.3f", i, point.x, point.y, point.z);
            }
        }
        auto l = lowestRoad;
        auto h = highestRoad;
        auto f = farthestRoad;
        printf("Road limits: lowest %6.3f %6.3f %6.3f highest: %6.3f %6.3f %6.3f farthest: %6.3f @ %6.3f %6.3f %6.3f\n", l.x, l.y, l.z, h.x, h.y, h.z, farthestDistance, f.x, f.y, f.z);

/*
        auto end = std::chrono::steady_clock::now();

        std::chrono::duration<double> elapsed_seconds = end-start;
        std::chrono::duration<double> middle_seconds = middle-start;
        std::cout << "\nmiddle time: " << middle_seconds.count() << " elapsed time: " << elapsed_seconds.count() << "s\n";

        for (int k = 0; k < 3; k++) {
            string name = k == 0 ? "left" : k == 1 ? "center" : "right";
            cout << "Region " << name << " road_depths: " << road_depths[k] << " Obstacle depths:" << obstacle_depths[k] << "\n";
        }

/*
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

        // Print out a point in each horizontal bin
        cout << setprecision(3);
        for (int j = 0; j < tan_divisions; j++) {
            if (pointMatrix[depth_divisions/2][j].size() > 0) {
                cout << "( " << pointMatrix[depth_divisions/2][j][0].x;
                cout << " , " << pointMatrix[depth_divisions/2][j][0].y;
                cout << " , " << pointMatrix[depth_divisions/2][j][0].z;
                cout << " ) ";          }
        }
        cout << "\n";
        // search for low-count bins
        /*
        for (int i = 0; i < depth_divisions; i++) {
            for (int j = 0; j < tan_divisions; j++) {
                auto points = pointMatrix[i][j];
                if (points.size() == 0) {
                    cout << "Zero count at " << i << "," << j << "\n";
                }
            }
        }
        */
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