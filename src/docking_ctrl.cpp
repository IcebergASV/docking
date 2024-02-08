#include <ros/ros.h>
#include <task_master/TaskStatus.h>
#include <task_master/TaskGoalPosition.h>
#include <task_master/Task.h>
#include <string>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <iostream>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include "lidar_point.h"

class Docking {
public:
    Docking(): nh_(""), private_nh_("~")
    {
        // ROS parameters -------------------------------------------------------------------------------------------------
        // example: private_nh_.param<double>("error", wp_error_tolerance, 1.0); // Tolerance radius for determining if asv reached gate waypoint
        private_nh_.param<std::string>("local_pose_topic", local_pose_topic_p, "/mavros/local_position/pose"); // topic params allows remaping
        private_nh_.param<double>("dock_distance", dock_distance_p, 7.0);
        private_nh_.param<double>("dock_discontinuity_range", dock_disc_p, 2.0);
        private_nh_.param<double>("slope_range", slope_range_p, 0.5);


        // ROS subscribers ------------------------------------------------------------------------------------------------
        
        task_to_exec_ = nh_.subscribe("/task_to_execute", 10, &Docking::dockingCallback, this); // task_master dictates which task is is be executed
        
        local_pos_ = nh_.subscribe(local_pose_topic_p, 10, &Docking::localPositionCallback, this);

        lidar_scan_ = nh_.subscribe("/laser/scan", 10, &Docking::scanCallback, this);

        // ROS publishers ------------------------------------------------------------------------------------------------------

        task_status_ = nh_.advertise<task_master::TaskStatus>("task_status", 10); // publishes task progress to /task_status

        task_goal_position_ = nh_.advertise<task_master::TaskGoalPosition>("task_goal_position", 10); // publishes desired positions to /task_goal_position to move asv
    }

    void spin() {
        ros::Rate rate(10);
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber task_to_exec_;
    ros::Subscriber local_pos_;
    ros::Subscriber lidar_scan_;
    ros::Publisher task_status_;
    ros::Publisher task_goal_position_;

    std::string local_pose_topic_p;

    double laser_angle_min_;
    double laser_angle_max_;
    double laser_angle_increment_;
    double dock_distance_p;
    double dock_disc_p;
    double slope_range_p;

    geometry_msgs::PoseStamped current_pos_;
    task_master::TaskGoalPosition goal_pos_;
    sensor_msgs::LaserScan scan_msg_;

    std::string TAG = "DOCKING_CTRL: ";


    std::vector<lidarPoint> createLidarPoints(const std::vector<float>& distances, double start_angle , double angle_increment) {
        std::vector<lidarPoint> lidarPoints;
        ROS_DEBUG_STREAM(TAG << "start angle: " << start_angle);
        // Add the first Lidar point
        lidarPoint firstPoint(distances[0], start_angle);
        lidarPoints.push_back(firstPoint);

        // Add the remaining Lidar points
        double currentAngle = start_angle + angle_increment;
        for (size_t i = 1; i < distances.size(); i++) {
            double distance = distances[i];
            lidarPoint point(distance, currentAngle);
            lidarPoints.push_back(point);

            currentAngle += angle_increment;
        }

    return lidarPoints;
    }

    void dockingCallback(const task_master::Task msg) {    
        // starts node, now scan callback does work
        ROS_DEBUG_STREAM(TAG << "dockingCallback");
        if(msg.current_task == task_master::Task::DOCKING) {
            ROS_DEBUG_STREAM(TAG << "Starting docking task");
            task_master::TaskStatus task_status;
            task_status.status = task_master::TaskStatus::IN_PROGRESS;
            task_status_.publish(task_status);
        }
    }

    void localPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pos_ = *msg;
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        scan_msg_ = *msg;

        laser_angle_min_ = scan_msg_.angle_min;
        laser_angle_max_ = scan_msg_.angle_max;
        laser_angle_increment_ = scan_msg_.angle_increment;
        // create vector of all lidar points
        double starting_angle = laser_angle_min_ + (M_PI/2.0); 
        std::vector<lidarPoint> scanPoints = Docking::createLidarPoints(scan_msg_.ranges, starting_angle, laser_angle_increment_);
        if (scanPoints.size()<1){
            ROS_WARN_STREAM(TAG << "No points added to scanPoints vector");
            return;
        }
        ROS_DEBUG_STREAM(TAG << "all lidar points = " << scanPoints.size());
        // filter points not part of dock
        std::vector<lidarPoint> dock_points;
        for (int i = 0; i < scanPoints.size(); i++) {
            double i_dist = scanPoints[i].getDistance();
            if (i_dist < dock_distance_p && i_dist > 2) { // magic number = dist to not track boat
                dock_points.push_back(scanPoints[i]);
            }
        }
        ROS_DEBUG_STREAM(TAG << "Found " << dock_points.size() << " points on the dock");
        if (dock_points.size()<1) {
            ROS_WARN_STREAM(TAG << "No scan points found on dock");
            return;
        }

        std::vector<lidarPoint> dock_edges;
        std::vector<int> edge_indexes;
        double current_dist = dock_points[0].getDistance();
        bool isGettingCloser;
        if (dock_points[1].getDistance() > current_dist) { // check discontinuity
            isGettingCloser = false;
        }
        else {
            isGettingCloser = true;
        }
        current_dist = dock_points[1].getDistance();
        for (int i = 2; i < dock_points.size(); i++) {
            double i_dist = dock_points[i].getDistance();
            if (isGettingCloser && current_dist<i_dist) { // this represents an outer corner
                ROS_DEBUG_STREAM(TAG << "outer corner found");
                dock_edges.push_back(dock_points[i]);
                edge_indexes.push_back(i);
                isGettingCloser = false;
            }
            else if (!isGettingCloser && current_dist>i_dist) { // inner corner, switch to getting closer
                isGettingCloser = true;
                ROS_INFO_STREAM(TAG << "inner corner found");
            }
            current_dist = i_dist;
        }
        if (dock_edges.size()<1) {
            ROS_WARN_STREAM(TAG << "No outer corners detected");
            return;
        }
        else {
            ROS_INFO_STREAM(TAG << "Detected " << dock_edges.size() << " outer corners");
            for (int i = 0; i<edge_indexes.size(); i++) {
                ROS_INFO_STREAM(TAG << "Edge " << i+1 << ", index = " << edge_indexes[i]);
            }
        }

        for (int i = 0; i < dock_edges.size(); i++) {
            double ix = dock_edges[i].getDistance() * cos(dock_edges[i].getAngle());
            double iy = dock_edges[i].getDistance() * sin(dock_edges[i].getAngle());
            ROS_INFO_STREAM(TAG << "Point: " << ix << ", " << iy);
        }

        // loop 3 times
        // in loops 1,2,3 :
            // remove point 2, then 3, then 4
            // calculate slope for line 2-1, then 3-2, then 4-3
            // if all slopes are equal within a range, then don't consider the removed point
            // then we have best 4 points

        double x1 = dock_edges[0].getDistance() * cos(dock_edges[0].getAngle());
        double y1 = dock_edges[0].getDistance() * sin(dock_edges[0].getAngle());
        double x2 = dock_edges[1].getDistance() * cos(dock_edges[1].getAngle());
        double y2 = dock_edges[1].getDistance() * sin(dock_edges[1].getAngle());
        double x3 = dock_edges[2].getDistance() * cos(dock_edges[2].getAngle());
        double y3 = dock_edges[2].getDistance() * sin(dock_edges[2].getAngle());
        double x4 = dock_edges[3].getDistance() * cos(dock_edges[3].getAngle());
        double y4 = dock_edges[3].getDistance() * sin(dock_edges[3].getAngle());
        double x5 = dock_edges[4].getDistance() * cos(dock_edges[4].getAngle());
        double y5 = dock_edges[4].getDistance() * sin(dock_edges[4].getAngle());

        // 1345, 1245, 1235
        double s13 = (y3-y1)/(x3-x1);
        double s34 = (y4-y3)/(x4-x3);
        double s45 = (y5-y4)/(x5-x4);
        
        // check if above 3 slopes are within slope_range of each other
        // a within range of b, a within range of c, b within range of c
        // if slopes are same, then have valid set of edges, remove the one not included

        // with 4 edges, find midpoints 12, 23, 34
        // 12 = left midpoint, 23 = center midpoint, 34 = right midpoint
        // determine how to proceed with calculating destinations (go to mid? go further from dock?)

        // once all functionality is present, reduce to helper functions
    }
    
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "docking_ctrl");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
        ros::console::notifyLoggerLevelsChanged();

    Docking docking;

    docking.spin();

    return 0;
}