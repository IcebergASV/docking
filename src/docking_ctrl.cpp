#include <ros/ros.h>
#include <task_master/TaskStatus.h>
#include <task_master/TaskGoalPosition.h>
#include <task_master/Task.h>
#include <string>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <iostream>

class Docking {
public:
    Docking(): nh_(""), private_nh_("~")
    {
        // ROS parameters -------------------------------------------------------------------------------------------------
        // example: private_nh_.param<double>("error", wp_error_tolerance, 1.0); // Tolerance radius for determining if asv reached gate waypoint
        private_nh_.param<std::string>("local_pose_topic", local_pose_topic_p, "/mavros/local_position/pose"); // topic params allows remaping


        // ROS subscribers ------------------------------------------------------------------------------------------------
        
        task_to_exec_ = nh_.subscribe("/task_to_execute", 10, &Docking::dockingCallback, this); // task_master dictates which task is is be executed
        
        local_pos_ = nh_.subscribe(local_pose_topic_p, 10, &Docking::localPositionCallback, this);

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
    ros::Publisher task_status_;
    ros::Publisher task_goal_position_;

    std::string local_pose_topic_p;

    geometry_msgs::PoseStamped current_pos_;
    task_master::TaskGoalPosition goal_pos_;

    std::string TAG = "DOCKING_CTRL: ";

    void dockingCallback(const task_master::Task msg) {    
        ROS_DEBUG_STREAM(TAG << "dockingCallback");
        task_master::TaskStatus task_status;
        task_status.task.current_task = task_master::Task::DOCKING;
        task_status.status = task_master::TaskStatus::NOT_STARTED;

        if(msg.current_task == task_master::Task::DOCKING) {
            ROS_INFO_STREAM(TAG << "Hello World");
            task_status.status = task_master::TaskStatus::IN_PROGRESS;
        }
        task_status_.publish(task_status);
    }

    void localPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pos_ = *msg;
    }
    
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "docking_ctrl");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
        ros::console::notifyLoggerLevelsChanged();

    Docking docking;

    docking.spin();

    return 0;
}