#ifndef RTRRTP_H_
#define RTRRTP_H_

#include <ros/ros.h>
#include <thread>
#include <std_srvs/Empty.h>
#include <mutex>
#include "sea_planner/rtrrt.h"
#include "sea_planner/get_state.h"

using namespace std;
using namespace util;

enum PlannerState{
    ACTIVE=0,
    WAITING=1,
};

class rtrrtp : public ParamLoader
{
    private:
        ros::NodeHandle nh;
        // ROS publisher 
        string global_frame;
        string base_frame;
        ros::Publisher pub_rtrrt_path;
        ros::Publisher pub_selected_branch;
        ros::Publisher pub_ref_line;
        ros::Publisher pub_target_marker;
        ros::Publisher pub_goal_marker;
        ros::Publisher pub_fake_path;
        ros::Publisher pub_waypoint_marker;
        ros::Publisher pub_waypoint;
        ros::Publisher pub_global_frontier;

        // ROS subscriber
        ros::Subscriber sub_goal;

        // ROS Service
        ros::ServiceServer planningTriger;
        ros::ServiceServer stopServer;
        ros::ServiceServer resetServer;
        ros::ServiceServer getStateServ;
        
        ros::Timer poseUpdater;
        ros::Timer visualizer;
        rtrrt *rtRRT;    // Launch rtrrt
        Pose robot_pose;
        nav_msgs::Path rtrrt_path;
        Nodes selectedBranch_nodes;
        std_srvs::Empty general_req;
        PlannerState cur_state;
        geometry_msgs::PoseStamped final_goal;
        geometry_msgs::PoseStamped subgoal;
        geometry_msgs::PointStamped way_point;
        std::mutex mtx;
        vector<rtrrt_ns::Node*> closeList;    // Store visited frontier 
        rtrrt_ns::Node *target_node;    // Taget node distance to goal or subgoal less than threshold
        unordered_map<rtrrt_ns::Node*, float> globalFrontier;    // Global frontier for replanning
        vector<rtrrt_ns::Node*> Path_node; // Experiment need

        bool tagetFound;    // The flag of finding target node in tree
        bool newGoalReceived;   // The flag of receiving a new goal
        bool updatedPose;   // The flag of updating robot pose using listening TF
        bool useFinalGoal;  // The flag of using final goal as target
        bool useSubGoal;    // The flag of using subgoal as target

    public:
        rtrrtp();
        ~rtrrtp();
        void init();    // Planner Initialization
        void searchPath();  // Searching path on the tree, for finding a shortest and static path
        void executeCycle(); // Planner steer robot to arrive final goal
        void publishPath(); // Publish rtrrt path(robot_pose -> target_goal)
        void publishWaypoint(); // Publish waypoint
        void selectNavType();   // Using subgoal or final goal?
        void validateFrontier();    // Validate all global frontiers
        void visualizationCB(const ros::TimerEvent &event);   // Timer: Publish markers
        void updateRobotPoseCB(const ros::TimerEvent &event); // Update robot pose through listening TF
        void ExecuteThread(); // Multithread for execute plan
        void stayCurrentPose(); // When changing the goal, the robot should stay current pose
        void resetResource(); // Reset all vars and tree for resetting planner
        bool getStateHandler(sea_planner::get_state::Request& req,
                             sea_planner::get_state::Response& state_res);   // Get current planner state
        bool resetPlanner(std_srvs::Empty::Request& req,
                          std_srvs::Empty::Response& res);    // Reset planner for re-planning
        bool stopPlanner(std_srvs::Empty::Request& req,
                          std_srvs::Empty::Response& res);     // Stopping planner when robot arrives goal and waits next goal
        void goalHandler(const geometry_msgs::PoseStampedConstPtr & goal_point);  // Goal callback func
};

#endif  // RTRRTP_H_