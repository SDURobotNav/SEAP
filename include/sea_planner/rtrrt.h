#ifndef RTRRT_H_
#define RTRRT_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <utility>
#include <map>
#include <list>
#include <queue>
#include <random>
#include <cstdlib>
#include "rtrrt_base.h"
#include "kdtree/kdtree.h"

using namespace std;
using namespace util;
using namespace rtrrt_ns;
using namespace termcolor;

typedef pair<rtrrt_ns::Node*, rtrrt_ns::Node*> NodePair;
typedef unordered_map<rtrrt_ns::Edge*, NodePair> Tree;
typedef vector<rtrrt_ns::Node*> Nodes;
typedef list<rtrrt_ns::Node*> NodesList;
typedef list<rtrrt_ns::Edge*> Edges;

class rtrrt : public ParamLoader
{
    private:
        // ROS parameter
        ros::NodeHandle nh;
        string global_frame;
        string base_frame;
        // ROS publisher for visualization
        ros::Publisher pub_rtrrt_edge;
        ros::Publisher pub_rtrrt_node;
        ros::Publisher pub_root_marker;
        ros::Publisher pub_leaf_node;
        ros::Publisher pub_frontier_node;
        ros::Publisher pub_goal_marker;
        ros::Publisher pub_subgoal_marker;
        ros::Publisher pub_path;
        ros::Publisher pub_range_map;
        ros::Publisher pub_sample_node;
        ros::Publisher pub_obstacle_node;

        ros::Timer TimerVis;

        rtrrt_ns::Node sampleNode;
        rtrrt_ns::Node newNode;
        queue<rtrrt_ns::Node*> q_rewire_from_root;  // Rewiring-from-rand queue
        queue<rtrrt_ns::Node*> q_rewire_from_rand;  // Rewiring-from-root queue
        ros::Publisher treePub;
        std::mt19937_64 gen_;
        std::uniform_real_distribution<double> uniform_rand_;
        std::uniform_real_distribution<double> uniform_rand1_;
        float path_length;
        grid_map::GridMap sensor_range_map;
        float map_x;
        float map_y;
        float map_resolution;
        bool receivedGridMap;
        bool updatedPose;
        kdtree* kdTree_;
        kdtree* obstacleTree_;
        int skipCountRewiring = 3; // Skip certain num update step to control rewiring-from-root freq

    public:
        Tree tree;
        NodesList node_set;
        Nodes obstacle_node_set;
        Edges edge_set;
        rtrrt_ns::Node *root;
        NodesList leaf_nodes;
        geometry_msgs::Pose goal;
        geometry_msgs::Point *current_target;   // Current target node in the tree
        rtrrt_ns::Node *current_target_node;   // Current target node in the tree
        util::Pose robot_pose;
        bool newGoalReceived;
        unordered_map<rtrrt_ns::Node*, float> frontierNodes;    // Frontier nodes and its information gain
        grid_map::GridMap full_grid_map;
        // ROS subscriber
        ros::Subscriber sub_grid_map;

        rtrrt();
        ~rtrrt();
        void init();    // Initialization
        void clearAll();   // Clear whole tree data and cache
        rtrrt_ns::Node sample();  // Sample point using rrt, informed rrt, random approached within space  
        bool checkNewNode(rtrrt_ns::Node x_new);    // Check new node is valid? or not?
        bool checkEdge(rtrrt_ns::Edge edge); // Check edge is valid? or not?
        rtrrt_ns::Node steer(rtrrt_ns::Node x_s); // Steer from parent node to sample, and prolong a certain length
        bool addNodeEdge(rtrrt_ns::Node x_new, rtrrt_ns::Node *x_parent);  // Add node, edge, and N-E pair to current tree
        void reselectParent(rtrrt_ns::Node* x_new);   // Reselect parent node within a certain range
        void rewireRandomNode(); // Rewire the branch at one of node in tree
        void rewireFromRoot();   // Rewire the whole tree 
        void executeTreeUpdate();   // Execute one step for updating tree
        void updateRobotPose(); // Update robot pose through listening TF
        void publishTree(); // Vistualize tree in rviz
        rtrrt_ns::Node* findTarget();  // Find the best target node near the goal
        void changeRoot(rtrrt_ns::Node *new_node);  // Change current root to new node 
        void updateGoal(geometry_msgs::PoseStamped goal_point);  // Update Goal position from external input
        void getFrontierNode(); // Get frontier node from all rtrrt leaf nodes
        kdres* getNeiborNodes(float point_x, float point_y, float range);  // Get neibor nodes set of given point
        rtrrt_ns::Node* getNearestNode(float point_x, float point_y);  // Get nearest node of given point
        rtrrt_ns::Node* getObstacleNode(float point_x, float point_y);  // Get nearest obstacle node of given point
        void updateOrientationArray(rtrrt_ns::Node *new_node, rtrrt_ns::Node *rrt_node);  // Update the value of node's orientation array
        void gridMapHandler(const grid_map_msgs::GridMapConstPtr &raw_map);  // Grid map callback func
        void visualizationCb(const ros::TimerEvent&);   // Timer callback for visualize the whole tree(node + edge)
        void visualizeSample(rtrrt_ns::Node sample_node);   // Visualization sample point pose

};

#endif  // RTRRT_H_