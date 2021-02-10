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
    const float min_depth = .25;
    const float max_depth = .75;
    const float min_height = .07;
    const float max_height = .17;
    const float max_distance_road = 0.03; // distance from road plane for allowable road
    const float min_distance_obstacle = 0.05; // distance from road plane to declare an obstacle
    const int tan_divisions = 12;
    const int depth_divisions = 12;
    const int max_depth_division_plane = 6; // last depth division for plane calculation
    const float min_tan = -0.45;
    const float max_tan = 0.45;
    const float max_obstacle = 2.0;
    const float min_obstacle_count = 5; // point count in bucket to recognize an obstacle
    const float min_road_count = 3; // point count in bucket to recognize road
    const int max_points = 20000; // The maximum number of PointCloud points to process
    const float delta_tan = (max_tan - min_tan) / (tan_divisions - 2.0);
    const float REQUIRED_DEPTH = 0.30; // the sr305 can reliably detect this close

    vector<float> tan_angles;

    int frame_count = 0;

    public:
    DetectBlocking()
    {
        // Subscribe to input video feed and publish output feed
        pc_sub = nh_.subscribe("/sr305/depth/color/points", 1, &DetectBlocking::pcCb, this);
        rb_pub = nh_.advertise<bdbd::RoadBlocking>("detectBlocking/roadBlocking", 1);
        rbn_pub = nh_.advertise<bdbd::RoadBlockingN>("detectBlocking/roadBlockingN", 1);

        for (int j = 0; j < tan_divisions; j++) {
            float division_tan = min_tan + (j - 1) * delta_tan + delta_tan/2.0;
            float division_angle = atan(division_tan);
            tan_angles.push_back(division_angle);
            // cout << "j " << j << " tan " << division_tan << " angle " << division_angle << " " << tan_angles[j] << "\n";
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
        vector<vector<vector<geometry_msgs::Point32>>> pointMatrix(tan_divisions, vector<vector<geometry_msgs::Point32>>(depth_divisions));
        vector<geometry_msgs::Point32> region_obstacles(3);

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
                cout << point << "\n";
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
            if (y > min_height && y < max_height && iz < max_depth_division_plane) {
                // This point is possibly valid road
                roadPointCount++;
                // only use some points for plane calculation to reduce CPU load
                if (roadPointCount % 5 == 0) {
                    eigenPoints.push_back(Eigen::Vector3f(x, y, z));
                }
            }
        }

        // Calculate the road plane

        bool validPlane = false;
        pair<Eigen::Vector3f, Eigen::Vector3f> plane;
        if (eigenPoints.size() > 50) {
            validPlane = true;
            plane = best_plane_from_points<Eigen::Vector3f>(eigenPoints);
        } else {
            validPlane = false;
            plane.first = Eigen::Vector3f(0.0, (max_height - min_height)/2., 0.0);
            plane.second = Eigen::Vector3f(0.0, 1.0, 0.0);
        }
        // plane should always point in one direction
        if (plane.second[1] < 0.0) {
            plane.second *= -1.0;
        }
        // check reasonableness of plane

        if (plane.second[1] < .98) {
            validPlane = false;
        }

        //cout << fixed;
        //cout << "Plane point: " << plane.first(0) << " " << plane.first(1) << " " << plane.first(2);
        //cout << " point count: " << eigenPoints.size() << " valid? " << validPlane;
        //cout << " normal: " << plane.second(0) << " " << plane.second(1) << " " << plane.second(2) << "\n";

        // Detect 1) distance to dropoff, and 2) distance to obstacle in left, center, and right

        // 2.0 is 2 meters, beyond the range of an sr305
        const float BIG_DEPTH = 2.0;
        vector<float> tan_obstacle_depth(tan_divisions, BIG_DEPTH);
        vector<float> tan_obstacle_height_raw(tan_divisions, 0.0);
        vector<float> tan_obstacle_height_relative(tan_divisions, 0.0);
        vector<float> tan_road_start(tan_divisions, BIG_DEPTH);
        vector<float> tan_road_end(tan_divisions, 0.0);
        vector<bool> tan_road_continuous(tan_divisions, true); // is the road unbroken?
        vector<float> tan_road_height_raw(tan_divisions, 0.0);
        vector<float> tan_road_height_relative(tan_divisions, 0.0);

        for (int i = 0; i < depth_divisions; i++) {
            for (int j = 0; j < tan_divisions; j++) {
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
                    if (point.z <= 0.001) {
                        continue; // not a valid datapoint
                    }
        
                    // calculate distance from plane to point
                    Eigen::Vector3f p(point.x, point.y, point.z);
                    auto distance = plane.second.dot(plane.first - p);

                    // Is this valid road?
                    if (validPlane && abs(distance) < max_distance_road) {
                        // This point is valid road
                        // if (point.y > min_height && point.y < max_height && point.z > min_depth && point.z < max_depth) {
                        //if (true) {
                        if (point.y > min_height && point.y < max_height) {
                            roads_in_bucket.push_back(point);
                            roads_distance_sum += distance;
                            roads_depth_sum += point.z;
                            roads_height_sum += point.y;
                            max_road_depth = max(max_road_depth, point.z);
                            min_road_depth = min(min_road_depth, point.z);
                        } else if (!did_point) {
                            did_point = true;
                            // cout << "point neither obstacle nor road A (" << point.x << " " << point.y << " " << point.z << ")\n";
                        }
                    } else if (distance > min_distance_obstacle) {
                        // This point is an obstacle (above the road plane)
                        obstacles_in_bucket.push_back(point);
                        obstacles_distance_sum += distance;
                        obstacles_depth_sum += point.z;
                        obstacles_height_sum += point.y;
                        min_obstacle_depth = min(min_obstacle_depth, point.z);
                    } else if (!did_point) {
                        did_point = true;
                        //cout << "point neither obstacle nor road B " << " distance: " << distance;
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
                // cout << "bucket(" << i << "," << j << ") " << " count " << points.size() << " " << " angle " << tan_angles[j];
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
                    tan_obstacle_depth[j] = min(tan_obstacle_depth[j], min_obstacle_depth);
                    tan_obstacle_height_raw[j] = obstacles_height;
                    tan_obstacle_height_relative[j] = obstacles_distance;
                }

                // We expect a minimum number of road points to declare valid
                if (roads_in_bucket.size() >= min_road_count) {
                    tan_road_start[j] = min(tan_road_start[j], min_road_depth);
                    if (tan_road_continuous[j]) {
                        tan_road_end[j] = max(tan_road_end[j], max_road_depth);
                        tan_road_height_raw[j] = roads_height;
                        tan_road_height_relative[j] = roads_distance;
                    }
                } else {
                    // No road, end road if it is started.
                    if (tan_road_start[j] < BIG_DEPTH) {
                        tan_road_continuous[j] = false;
                    }
                }
            }
        }

        bdbd::RoadBlockingN rbn;
        for (int j = 0; j < tan_divisions; j++) {
            rbn.angle.push_back(tan_angles[j]);
            rbn.obstacle_depth.push_back(tan_obstacle_depth[j]);
            rbn.obstacle_height_raw.push_back(tan_obstacle_height_raw[j]);
            rbn.obstacle_height_relative.push_back(tan_obstacle_height_relative[j]);
            rbn.road_start.push_back(tan_road_start[j]);
            rbn.road_end.push_back(tan_road_end[j]);
            rbn.road_height_raw.push_back(tan_road_height_raw[j]);
            rbn.road_height_relative.push_back(tan_road_height_relative[j]);
        }
        auto rosnow = ros::Time::now();
        double now = (double)rosnow.sec + 1.e-9 * (double)rosnow.nsec;
        // cout << setprecision(15) << "now sec " << rosnow.sec << " nsec " << rosnow.nsec << " double now " << now << "\n";
        rbn.rostime = now;
        rbn_pub.publish(rbn);
        // cout << rbn << "\n";

        // interpret results as left, center, right regions
        // left is region 0, center 1, right 2
        vector<float> road_depths(3, 0.0);
        vector<float> obstacle_depths(3, max_obstacle);
        for (int j = 0; j < tan_divisions; j++) {
            int region;
            if (j < tan_divisions / 3)
                region = 0;
            else if (j >= 2 * tan_divisions / 3)
                region = 2;
            else
                region = 1;
            obstacle_depths[region] = min(obstacle_depths[region], tan_obstacle_depth[j]);
            if (tan_road_start[j] < REQUIRED_DEPTH) {
                road_depths[region] = max(road_depths[region], tan_road_end[j]);
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