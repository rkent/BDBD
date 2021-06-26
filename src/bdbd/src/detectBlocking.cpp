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
    ros::NodeHandle nh_;
    ros::Subscriber pc_sub;
    ros::Publisher rb_pub;
    ros::Publisher rbn_pub;
    ros::Publisher ls_pub;
    // These determine the membership in depth buckets
    const float min_depth = .30;
    const float max_depth = .70;
    const float min_height = .04;
    const float max_height = .16;
    const float max_distance_road = 0.03; // distance from road plane for allowable road
    const float min_distance_obstacle = 0.05; // distance from road plane to declare an obstacle
    const int angle_divisions = 23;
    const int depth_divisions = 10;
    const float min_angle = -0.45;
    const float max_angle = 0.45;
    const float max_obstacle = 2.0;
    const float min_obstacle_count = 10; // point count in bucket to recognize an obstacle
    const float min_road_count = 1; // point count in bucket to recognize road
    const float eps = 0.01; // added to max when no obstacles detected
    const int max_points = 40000; // The maximum number of PointCloud points to process
    const float delta_angle = (max_angle - min_angle) / (angle_divisions - 3.0);
    const std::string target_frame = "sr305_color_frame";
    vector<float> angle_angles;

    int frame_count = 0;

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
            auto z = point.z;
            if (z < .001) {
                ROS_WARN_STREAM("Small or negative distance in point:" <<  point);
                continue;
            }
            auto y = point.y;
            auto x = point.x;
            float depth = sqrt(x * x + z * z);

            int iz = 1 + int(float(depth_divisions - 2) * (depth - min_depth) / (max_depth - min_depth));
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
        }

        // calculate the average height in each bucket
        Eigen::MatrixXd avgHeight(depth_divisions, angle_divisions);
        //Eigen::MatrixXd avgDistance(depth_divisions, angle_divisions);
        /* */
        for (int i = 0; i < depth_divisions; i++) {
            for (int j = 0; j < angle_divisions; j++) {
                auto points = pointMatrix[i][j];
                // ROS_WARN_STREAM("i j " << i << " " << j << " points.size() " << points.size());
                double height_sum = 0.0;
                //double distance_sum = 0.0;
                for (auto& point: points) {
                    // calculate distance from plane to point
                    Eigen::Vector3f p(point.x, point.y, point.z);
                    //distance_sum += plane.second.dot(plane.first - p);
                    height_sum += point.y;
                }
                avgHeight(i, j) = points.size() != 0.0 ? height_sum / points.size() : 0.0;
                //avgDistance(i, j) = points.size() != 0.0 ? distance_sum / points.size() : 0.0;
            }
        }
        //ROS_WARN_STREAM("Average height:\n" << avgHeight);
        //ROS_WARN_STREAM("Average distance:\n" << avgDistance);
        /* */

        // save points for plane analysis
        for (int i = 1; i < depth_divisions - 1; i++) {
            for (int j = 1; j < angle_divisions - 1; j++) {
                if (
                    avgHeight(i, j) > min_height &&
                    avgHeight(i, j) < max_height &&
                    abs(avgHeight(i, j) - avgHeight(i-1, j)) < .2 * max_distance_road
                ) {
                    // This point is probably valid road
                    auto points = pointMatrix[i][j];
                    // only use some points for plane calculation to reduce CPU load
                    for (int k = 0; k < points.size(); k += 5) {
                        auto point = points[k];
                        roadPlanePoints.push_back(Eigen::Vector3f(point.x, point.y, point.z));
                    }
                }
            }
        }

        // Calculate the road plane

        bool validPlane = false;
        pair<Eigen::Vector3f, Eigen::Vector3f> plane;
        if (roadPlanePoints.size() > 50) {
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
        if (plane.second[1] < .95) {
            ROS_WARN_STREAM("Direction of road plane is not reasonable" << plane.second[1]);
            validPlane = false;
        }

        if (!validPlane) {
            plane.first = Eigen::Vector3f(0.0, (max_height - min_height)/2., 0.0);
            plane.second = Eigen::Vector3f(0.0, 1.0, 0.0);
        }

        //cout << fixed;
        //cout << "Plane point: " << plane.first(0) << " " << plane.first(1) << " " << plane.first(2);
        //cout << " point count: " << roadPlanePoints.size() << " valid? " << validPlane;
        //cout << " normal: " << plane.second(0) << " " << plane.second(1) << " " << plane.second(2) << "\n";

        // Detect 1) distance to dropoff, and 2) distance to obstacle in left, center, and right

        // 2.0 is 2 meters, beyond the range of an sr305
        const float BIG_DEPTH = 2.0;
        vector<float> angle_obstacle_depth(angle_divisions, BIG_DEPTH);
        vector<float> angle_obstacle_height_raw(angle_divisions, 0.0);
        vector<float> angle_obstacle_height_relative(angle_divisions, 0.0);
        vector<float> angle_road_start(angle_divisions, BIG_DEPTH);
        vector<float> angle_road_end(angle_divisions, 0.0);
        vector<bool> angle_road_continuous(angle_divisions, true); // is the road unbroken?
        vector<float> angle_road_height_raw(angle_divisions, 0.0);
        vector<float> angle_road_height_relative(angle_divisions, 0.0);

        for (int i = 0; i < depth_divisions; i++) {
            for (int j = 0; j < angle_divisions; j++) {
                vector<geometry_msgs::Point32> obstacles_in_bucket;
                vector<geometry_msgs::Point32> roads_in_bucket;
                double roads_distance_sum = 0.0;
                double roads_depth_sum = 0.0;
                double roads_height_sum = 0.0;
                double obstacles_distance_sum = 0.0;
                double obstacles_depth_sum = 0.0;
                double obstacles_height_sum = 0.0;
                bool did_point = false;

                auto points = pointMatrix[i][j];

                float min_road_depth = BIG_DEPTH;
                float max_road_depth = 0.0;
                float min_obstacle_depth = BIG_DEPTH;
                for (auto& point: points) {
                    float depth = sqrt(point.x * point.x + point.z * point.z);
                    // calculate distance from plane to point
                    Eigen::Vector3f p(point.x, point.y, point.z);
                    auto distance = plane.second.dot(plane.first - p);

                    // Is this valid road?
                    if (validPlane && abs(distance) < max_distance_road) {
                        // This point is valid road
                        if (point.y > min_height && point.y < max_height) { // ? why do I need this check?
                            roads_in_bucket.push_back(point);
                            roads_distance_sum += distance;
                            roads_depth_sum += depth;
                            roads_height_sum += point.y;
                            max_road_depth = max(max_road_depth, depth);
                            min_road_depth = min(min_road_depth, depth);
                        } else if (!did_point) {
                            did_point = true;
                            //ROS_WARN_STREAM("point neither obstacle nor road A (" << point.x << " " << point.y << " " << point.z);
                        }
                    } else if (distance > min_distance_obstacle) {
                        // This point is an obstacle (above the road plane)
                        obstacles_in_bucket.push_back(point);
                        obstacles_distance_sum += distance;
                        obstacles_depth_sum += depth;
                        obstacles_height_sum += point.y;
                        min_obstacle_depth = min(min_obstacle_depth, depth);
                    } else if (!did_point) {
                        did_point = true;
                        //ROS_WARN_STREAM("point neither obstacle nor road B " << " distance: " << distance);
                        //cout << " point: (" << point.x << " " << point.y << " " << point.z << ")\n";
                    }
                }

                //if (obstacles_in_bucket.size() > 0) {
                //    printf("region obstacle count: %li\n", obstacles_in_bucket.size());
                //}
                float roads_distance = roads_in_bucket.size() ? roads_distance_sum / roads_in_bucket.size() : 0.0;
                float roads_depth = roads_in_bucket.size() ? roads_depth_sum / roads_in_bucket.size() : 0.0;
                float roads_height = roads_in_bucket.size() ? roads_height_sum / roads_in_bucket.size() : 0.0;
                float obstacles_distance = obstacles_in_bucket.size() ? obstacles_distance_sum / obstacles_in_bucket.size() : 0.0;
                float obstacles_depth = obstacles_in_bucket.size() ? obstacles_depth_sum / obstacles_in_bucket.size() : 0.0;
                float obstacles_height = obstacles_in_bucket.size() ? obstacles_height_sum / obstacles_in_bucket.size() : 0.0;
                // cout << "bucket(" << i << "," << j << ") " << " count " << points.size() << " " << " angle " << angle_angles[j];
                //if (obstacles_in_bucket.size() > 1 and obstacles_distance < 0.2) {
                //    cout << " obstacles(j, count, dist, depth, height): ("
                //    << j << " , "
                //    << obstacles_in_bucket.size() << " , "
                //    << obstacles_distance << " , "
                //    << obstacles_depth << " , "
                //    << obstacles_height << " )\n";
                //}
                //if (roads_in_bucket.size()) {
                //    cout << "roads[" << i << "," << j << "] ";
                //    cout << "(count, dist, depth, height): (" << roads_in_bucket.size() << ',';
                //    cout << roads_distance << "," << roads_depth << "," << roads_height << ")\n";
                //}

                // We expect a minimum number of detected obstacle points to declare valid
                if (obstacles_in_bucket.size() >= min_obstacle_count) {
                    angle_obstacle_depth[j] = min(angle_obstacle_depth[j], min_obstacle_depth);
                    angle_obstacle_height_raw[j] = obstacles_height;
                    angle_obstacle_height_relative[j] = obstacles_distance;
                } else if (obstacles_in_bucket.size() > 0) {
                    //ROS_WARN_STREAM(j << " d " << obstacles_depth << " Not enough points in obstacle bucket: " << obstacles_in_bucket.size());
                }

                // We expect a minimum number of road points to declare valid
                if (roads_in_bucket.size() >= min_road_count) {
                    angle_road_start[j] = min(angle_road_start[j], min_road_depth);
                    if (angle_road_continuous[j]) {
                        angle_road_end[j] = max(angle_road_end[j], max_road_depth);
                        angle_road_height_raw[j] = roads_height;
                        angle_road_height_relative[j] = roads_distance;
                    }
                } else {
                    // No road, end road if it is started.
                    if (roads_in_bucket.size() > 0) {
                        //ROS_WARN_STREAM(j << " d " << roads_depth << " Not enough points in road bucket: " << roads_in_bucket.size());
                    }
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
        sensor_msgs::LaserScan output;
        output.header = pc2msg->header;
        // The LaserScan frame is rotated relative to the depth frame
        output.header.frame_id = target_frame;
        output.angle_min = min_angle;
        output.angle_max = max_angle;
        output.angle_increment = delta_angle;
        output.time_increment = 0.0;
        output.scan_time = 0.0;
        output.range_min = min_depth;
        output.range_max = max_depth;

        /* */
        // range is the distance to an impenetrable dropoff or obstacle
        for (int jj = 1; jj < angle_divisions - 1; jj++) {
            // flip angle coordinates
            int j = angle_divisions - jj - 1;
            float range = min_depth;
            if (angle_road_start[j] < min_depth) {
                // There is at least some valid road in front of us
                range = min(angle_road_end[j], angle_obstacle_depth[j]);
            }
            range = min(max_depth + eps, max(min_depth + eps, range));
            output.ranges.push_back(range);
            // ROS_WARN_STREAM("Laser scan j: " << j << " angle: " << angle_angles[j] << " range: " << range);
        }
        /* */
        ls_pub.publish(output);

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
            if (angle_road_start[j] < min_depth) {
                road_depths[region] = max(road_depths[region], angle_road_end[j]);
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