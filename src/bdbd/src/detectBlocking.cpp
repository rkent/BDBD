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
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <bdbd/RoadBlocking.h>
#include <bdbd/RoadBlockingN.h>
#include <iostream>
#include <chrono>
#include <vector>
#include <array>
#include <iomanip>
#include <algorithm>
#include <string>
#include <stdio.h>
#include <math.h>
#include "libcpp/fitting3d.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;

class DetectBlocking
{
    // *** Configuration constants
    // These determine the membership in depth buckets
    const int angle_divisions = 52;
    const int depth_divisions = 52;
    const int angle_road_combine = 2;
    const int depth_road_combine = 1;
    const int point_plane_skip = 3;
    const int MIN_PLANE_POINTS = 25;
    const int MAX_PLANE_DENORMAL = 0.95;
    const float min_depth = 0.25;
    const float min_depth_division = 0.20;
    const float max_road_depth = 0.40; // We cannot reliably see road past this depth
    const float max_depth = 2.00;
    const float min_height = .04;
    const float max_height = .16;
    const float max_distance_road = 0.03; // distance from road plane for allowable road
    const float min_distance_obstacle = 0.05; // distance from road plane to declare an obstacle
    const float min_angle = -0.45;
    const float max_angle = 0.45;
    const float max_obstacle = 2.0;
    const float BIG_DEPTH = 2.0;
    const float min_obstacle_count = 10; // point count in bucket to recognize an obstacle
    const float min_road_count = 2; // point count in bucket to recognize road
    const float eps = 0.01; // added to max when no obstacles detected
    const int max_points = 10000; // The maximum number of PointCloud points to process
    const std::string target_frame = "sr305_color_frame";
    const bool USE_ROAD_BLOCKING = true;

    // *** Calculated constants
    const float delta_angle = (max_angle - min_angle) / (angle_divisions - 3.0);

    // *** Class variables
    vector<float> angle_angles;
    int frame_count = 0;
    ros::NodeHandle nh_;
    ros::Subscriber pc_sub;
    ros::Publisher rb_pub;
    ros::Publisher rbn_pub;
    ros::Publisher ls_pub;

    public:
    DetectBlocking()
    {
        // Subscribe to input video feed and publish output feed
        pc_sub = nh_.subscribe("/sr305/depth/color/points", 1, &DetectBlocking::pcCb, this);
        rb_pub = nh_.advertise<bdbd::RoadBlocking>("detectBlocking/roadBlocking", 1);
        rbn_pub = nh_.advertise<bdbd::RoadBlockingN>("detectBlocking/roadBlockingN", 1);
        ls_pub = nh_.advertise<sensor_msgs::LaserScan>("detectBlocking/scan", 1);

        for (int j = 0; j < angle_divisions; j++) {
            float division_angle = min_angle + (j - 1) * delta_angle + delta_angle/2.0;
            angle_angles.push_back(division_angle);
        }
    }

    ~DetectBlocking()
    {
    }

    void pcCb(const sensor_msgs::PointCloud2ConstPtr& pc2msg)
    {
        auto start = chrono::steady_clock::now();
        sensor_msgs::PointCloud pcmsg;
        sensor_msgs::convertPointCloud2ToPointCloud(*pc2msg, pcmsg);
        auto middle = chrono::steady_clock::now();
        vector<vector<vector<geometry_msgs::Point32>>> pointMatrix(depth_divisions, vector<vector<geometry_msgs::Point32>>(angle_divisions));
        vector<vector<vector<float>>> depthMatrix(depth_divisions, vector<vector<float>>(angle_divisions));
        vector<geometry_msgs::Point32> region_obstacles(3);

        frame_count++;

        int roadPointCount = 0;
        vector<Eigen::Vector3f> roadPlanePoints;

        geometry_msgs::Point32 lowestRoad, highestRoad, farthestRoad;
        lowestRoad.y = 100.;
        float farthestDistance = 0.0;

        // Divide all points into buckets by depth, tangent
        // Assumed coordinate system: x positive right, y positive down, z positive forward.
        int indexDelta = (pcmsg.points.size() / max_points) + 1;
        // ROS_WARN_STREAM("Point cloud points " << pcmsg.points.size() << " delta " <<indexDelta);
        for (int i = 0; i < pcmsg.points.size(); i += indexDelta)
        {
            auto point = pcmsg.points[i];
            auto x = point.x;
            auto z = point.z;
            if (z < .001) {
                ROS_WARN_STREAM("Small or negative distance in point:" <<  point);
                continue;
            }
            float depth = sqrt(x * x + z * z);

            int iz = 1 + int(float(depth_divisions - 2) * (depth - min_depth_division) / (max_depth - min_depth_division));
            if (iz < 0) {
                iz = 0;
            } else if (iz > depth_divisions - 1) {
                iz = depth_divisions - 1;
            }

            float a = atan(x / z);
            int ja = 1 + int(float(angle_divisions - 2) * (a - min_angle) / (max_angle - min_angle));
            if (ja < 0) {
                ja = 0;
            } else if (ja > angle_divisions - 1) {
                ja = angle_divisions - 1;
            }

            pointMatrix[iz][ja].push_back(point);
            depthMatrix[iz][ja].push_back(depth);
        }

        // calculate the average height in each bucket
        Eigen::MatrixXd bucketHeight(depth_divisions, angle_divisions);
        for (int i = 0; i < depth_divisions; i++) {
            for (int j = 0; j < angle_divisions; j++) {
                auto points = pointMatrix[i][j];
                double height_sum = 0.0;
                for (auto& point: points) {
                    height_sum += point.y;
                }
                bucketHeight(i, j) = points.size() != 0.0 ? height_sum / points.size() : 0.0;
            }
        }

        // save points for plane analysis
        for (int i = 1; USE_ROAD_BLOCKING && i < depth_divisions - 1; i++) {
            for (int j = 1; j < angle_divisions - 1; j++) {
                if (
                    bucketHeight(i, j) > min_height &&
                    bucketHeight(i, j) < max_height &&
                    abs(bucketHeight(i, j) - bucketHeight(i-1, j)) < .2 * max_distance_road
                ) {
                    // This point is probably valid road
                    auto points = pointMatrix[i][j];
                    // only use some points for plane calculation to reduce CPU load
                    for (int k = 0; k < points.size(); k += point_plane_skip) {
                        auto point = points[k];
                        roadPlanePoints.push_back(Eigen::Vector3f(point.x, point.y, point.z));
                    }
                }
            }
        }

        // Calculate the road plane

        bool validPlane = false;
        pair<Eigen::Vector3f, Eigen::Vector3f> plane;
        if (USE_ROAD_BLOCKING) {
            if (roadPlanePoints.size() > MIN_PLANE_POINTS) {
                validPlane = true;
                plane = best_plane_from_points<Eigen::Vector3f>(roadPlanePoints);
            } else {
                ROS_WARN_STREAM("Insufficient points to determine road plane: " << roadPlanePoints.size());
                validPlane = false;
            }

            // plane should always point in one direction
            if (plane.second[1] < 0.0) {
                plane.second *= -1.0;
            }

            // check reasonableness of plane
            if (plane.second[1] < MAX_PLANE_DENORMAL) {
                ROS_WARN_STREAM("Direction of road plane is not reasonable" << plane.second[1]);
                validPlane = false;
            }

        }
        if (!validPlane) {
            plane.first = Eigen::Vector3f(0.0, (max_height - min_height)/2., 0.0);
            plane.second = Eigen::Vector3f(0.0, 1.0, 0.0);
        }

        // obstacles

        vector<float> angle_obstacle_depth(angle_divisions, BIG_DEPTH);
        vector<float> angle_obstacle_height_raw(angle_divisions, 0.0);
        vector<float> angle_obstacle_height_relative(angle_divisions, 0.0);

        for (int i = 0; i < depth_divisions; i++) {
            for (int j = 0; j < angle_divisions; j++) {
                vector<geometry_msgs::Point32> obstacles_in_bucket;
                double obstacles_depth_sum = 0.0;
                double obstacles_distance_sum = 0.0;
                double obstacles_height_sum = 0.0;
                auto points = pointMatrix[i][j];
                auto depths = depthMatrix[i][j];
                for (int k = 0; k < points.size(); k++) {
                    auto point = points[k];
                    auto depth = depths[k];
                    // calculate distance from plane to point
                    Eigen::Vector3f p(point.x, point.y, point.z);
                    auto distance = plane.second.dot(plane.first - p);
                    if (distance > min_distance_obstacle) {
                        // This point is an obstacle (above the road plane)
                        obstacles_in_bucket.push_back(point);
                        obstacles_distance_sum += distance;
                        obstacles_height_sum += point.y;
                        obstacles_depth_sum += depth;
                    }
                }

                // We expect a minimum number of detected obstacle points to declare valid
                if (obstacles_in_bucket.size() >= min_obstacle_count) {
                    float obstacles_depth = obstacles_depth_sum / obstacles_in_bucket.size();
                    if (obstacles_depth < angle_obstacle_depth[j]) {
                        // ROS_WARN_STREAM("j " << j << " count " << obstacles_in_bucket.size() << " sum " << obstacles_depth_sum << " depth " << obstacles_depth);
                        float obstacles_distance = obstacles_distance_sum / obstacles_in_bucket.size();
                        float obstacles_height = obstacles_height_sum / obstacles_in_bucket.size();
                        angle_obstacle_depth[j] = obstacles_depth;
                        angle_obstacle_height_raw[j] = obstacles_height;
                        angle_obstacle_height_relative[j] = obstacles_distance;
                    }
                }
            }
        }

        // roads

        vector<float> angle_road_start(angle_divisions, BIG_DEPTH);
        vector<float> angle_road_end(angle_divisions, 0.0);
        vector<bool> angle_road_continuous(angle_divisions, true); // is the road unbroken?
        vector<float> angle_road_height_raw(angle_divisions, 0.0);
        vector<float> angle_road_height_relative(angle_divisions, 0.0);
        vector<vector<float>> depth_sums_in_buckets(depth_divisions, vector<float>(angle_divisions, 0.0)); 
        vector<vector<float>> distance_sums_in_buckets(depth_divisions, vector<float>(angle_divisions, 0.0)); 
        vector<vector<float>> height_sums_in_buckets(depth_divisions, vector<float>(angle_divisions, 0.0)); 
        vector<vector<int>> counts_in_buckets(depth_divisions, vector<int>(angle_divisions, 0));
        for (int i = 0; USE_ROAD_BLOCKING && i < depth_divisions; i++) {
            for (int j = 0; j < angle_divisions; j++) {
                auto points = pointMatrix[i][j];
                auto depths = depthMatrix[i][j];

                for (int k = 0; k < points.size(); k++) {
                    auto point = points[k];
                    auto depth = depths[k];
                    // calculate distance from plane to point
                    Eigen::Vector3f p(point.x, point.y, point.z);
                    auto distance = plane.second.dot(plane.first - p);

                    // Is this valid road?
                    if (validPlane &&
                        abs(distance) < max_distance_road &&
                         // ? why do I need these checks?
                        point.y > min_height &&
                        point.y < max_height)
                    {
                        // This point is valid road
                        counts_in_buckets[i][j]++;
                        distance_sums_in_buckets[i][j] += distance;
                        depth_sums_in_buckets[i][j] += depth;
                        height_sums_in_buckets[i][j] += point.y;
                    }
                }
            }
        }

        // Average values to reduce noise
        vector<vector<float>> avg_depth_sums_in_buckets(depth_divisions, vector<float>(angle_divisions, 0.0)); 
        vector<vector<float>> avg_distance_sums_in_buckets(depth_divisions, vector<float>(angle_divisions, 0.0)); 
        vector<vector<float>> avg_height_sums_in_buckets(depth_divisions, vector<float>(angle_divisions, 0.0)); 
        vector<vector<int>> avg_counts_in_buckets(depth_divisions, vector<int>(angle_divisions, 0));
        for (int i = 1; USE_ROAD_BLOCKING && i < depth_divisions - 1; i++) {
            for (int j = 1; j < angle_divisions - 1; j++) {

                for (int ii = i - depth_road_combine; ii <= i + depth_road_combine; ii++) {
                    if (ii < 0 || ii > depth_divisions - 1) continue;
                    for (int jj = j - angle_road_combine; jj <= j + angle_road_combine; jj++) {
                        if (jj < 0 || jj > depth_divisions - 1) continue;

                        avg_counts_in_buckets[i][j] += counts_in_buckets[ii][jj];
                        avg_depth_sums_in_buckets[i][j] += depth_sums_in_buckets[ii][jj];
                        avg_distance_sums_in_buckets[i][j] += distance_sums_in_buckets[ii][jj];
                        avg_height_sums_in_buckets[i][j] += height_sums_in_buckets[ii][jj];
                    }
                }

                // Calculate roads per angle, requiring minimum bucket count
                if (avg_counts_in_buckets[i][j] >= min_road_count) {
                    float fcount = (float)(avg_counts_in_buckets[i][j]);
                    float roads_depth = avg_depth_sums_in_buckets[i][j] / fcount;
                    angle_road_start[j] = min(angle_road_start[j], roads_depth);
                    if (angle_road_continuous[j]) {
                        angle_road_end[j] = max(angle_road_end[j], roads_depth);
                        angle_road_height_raw[j] = avg_height_sums_in_buckets[i][j] / fcount;
                        angle_road_height_relative[j] = avg_distance_sums_in_buckets[i][j] / fcount;
                    }
                    /*
                    ROS_WARN_STREAM(
                        "i " << i << 
                        " j " << j << 
                        " count " << avg_counts_in_buckets[i][j] << 
                        " sum " << avg_depth_sums_in_buckets[i][j] <<
                        " roads_depth " << roads_depth <<
                        " angle_road_start[j] " << angle_road_start[j] <<
                        " angle_road_end[j] " << angle_road_end[j] <<
                        " angle_road_continuous[j] " << angle_road_continuous[j]
                    );
                    */
                } else {
                    // No road, end road if it is started.
                    if (angle_road_start[j] < BIG_DEPTH) {
                        angle_road_continuous[j] = false;
                    }
                }
            }
        }

        bdbd::RoadBlockingN rbn;
        for (int j = 0; j < angle_divisions; j++) {
            rbn.angle.push_back(angle_angles[j]);
            rbn.obstacle_depth.push_back(angle_obstacle_depth[j]);
            rbn.obstacle_height_raw.push_back(angle_obstacle_height_raw[j]);
            rbn.obstacle_height_relative.push_back(angle_obstacle_height_relative[j]);
            rbn.road_start.push_back(angle_road_start[j]);
            rbn.road_end.push_back(angle_road_end[j]);
            rbn.road_height_raw.push_back(angle_road_height_raw[j]);
            rbn.road_height_relative.push_back(angle_road_height_relative[j]);
        }
        auto rosnow = ros::Time::now();
        double now = (double)rosnow.sec + 1.e-9 * (double)rosnow.nsec;
        // cout << setprecision(15) << "now sec " << rosnow.sec << " nsec " << rosnow.nsec << " double now " << now << "\n";
        rbn.rostime = now;
        rbn_pub.publish(rbn);

        // Publish LaserScan

        // build laserscan output
        sensor_msgs::LaserScan scan_output;
        scan_output.header = pc2msg->header;
        // The LaserScan frame is rotated relative to the depth frame
        scan_output.header.frame_id = target_frame;
        scan_output.angle_min = min_angle;
        scan_output.angle_max = max_angle;
        scan_output.angle_increment = delta_angle;
        scan_output.time_increment = 0.0;
        scan_output.scan_time = 0.0;
        scan_output.range_min = min_depth;
        scan_output.range_max = max_depth;

        // range is the distance to an impenetrable dropoff or obstacle
        for (int jj = 1; jj < angle_divisions - 1; jj++) {
            // flip angle coordinates
            int j = angle_divisions - jj - 1;
            float range = min_depth;
            if (USE_ROAD_BLOCKING) {
                if (angle_road_start[j] < min_depth) {
                    // We see at least some road
                    if (angle_road_end[j] < max_road_depth) {
                        // There is at least some missing road in front of us
                        range = min(angle_road_end[j], angle_obstacle_depth[j]);
                        if (angle_obstacle_depth[j] > max_road_depth + .05) {
                            // This blocking is only due to a potential dropoff 
                            ROS_WARN_STREAM("Road dropoff at " << range << " obstacle at " << angle_obstacle_depth[j]);
                        }
                    }
                    else {
                        // We cannot detect road length, but show any obstacles
                        range = angle_obstacle_depth[j];
                        // ROS_WARN_STREAM("Obstacle blocking at " << range);
                    }
                }
                range = min(max_depth + eps, max(min_depth + eps, range));
            } else {
                range = min(max_depth + eps, angle_obstacle_depth[j]);
            }
            scan_output.ranges.push_back(range);
            // ROS_WARN_STREAM("Laser scan j: " << j << " angle: " << angle_angles[j] << " range: " << range);
        }
        ls_pub.publish(scan_output);

        // if (USE_ROAD_BLOCKING)
        {
            // interpret results as left, center, right regions
            // left is region 0, center 1, right 2
            vector<float> road_depths(3, 0.0);
            vector<float> obstacle_depths(3, max_obstacle);
            for (int j = 0; j < angle_divisions; j++) {
                int region;
                if (j < angle_divisions / 3)
                    region = 0;
                else if (j >= 2 * angle_divisions / 3)
                    region = 2;
                else
                    region = 1;
                obstacle_depths[region] = min(obstacle_depths[region], angle_obstacle_depth[j]);
                if (USE_ROAD_BLOCKING) {
                    if (angle_road_start[j] < min_depth) {
                        road_depths[region] = max(road_depths[region], angle_road_end[j]);
                    }
                }
                else {
                    road_depths[region] = max_road_depth;
                }
            }
            bdbd::RoadBlocking rb;
            rb.rostime = now;

            rb.leftRoadDepth = road_depths[0];
            rb.centerRoadDepth = road_depths[1];
            rb.rightRoadDepth = road_depths[2];
            rb.leftObstacleDepth = obstacle_depths[0];
            rb.centerObstacleDepth = obstacle_depths[1];
            rb.rightObstacleDepth = obstacle_depths[2];
            rb_pub.publish(rb);
        }
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