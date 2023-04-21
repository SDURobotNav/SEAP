#include "sea_planner/rtrrtp.h"
using namespace termcolor;

void rtrrtp::init(){
    rtRRT = new rtrrt;
    target_node = NULL;
    global_frame = globalFrame;
    base_frame = baseFrame;
    useFinalGoal = false;
    useSubGoal = false;
    cur_state = PlannerState::WAITING;
    // Publisher init
	pub_rtrrt_path = nh.advertise<nav_msgs::Path>("/rtrrt_path",10);
    pub_selected_branch = nh.advertise<visualization_msgs::MarkerArray>("selected_branch",10);
    pub_ref_line = nh.advertise<visualization_msgs::Marker>("reference_line",10);
    pub_fake_path = nh.advertise<visualization_msgs::Marker>("expected_path", 10);
	pub_waypoint = nh.advertise<geometry_msgs::PointStamped>("way_point", 10);
	pub_waypoint_marker = nh.advertise<visualization_msgs::Marker>("way_point_marker", 10);
    pub_goal_marker = nh.advertise<visualization_msgs::Marker>("Goal", 10);
    pub_target_marker = nh.advertise<visualization_msgs::Marker>("target", 10);
    pub_global_frontier = nh.advertise<visualization_msgs::Marker>("global_frontier", 10);
	// Subscriber init
    sub_goal = nh.subscribe("/move_base_simple/goal",1, &rtrrtp::goalHandler,this);
    // Service init
    stopServer = nh.advertiseService("sea_planner/stop_planning", &rtrrtp::stopPlanner, this);
    resetServer = nh.advertiseService("sea_planner/reset_planner", &rtrrtp::resetPlanner, this);
    getStateServ = nh.advertiseService("sea_planner/get_state", &rtrrtp::getStateHandler, this);

    ros::Rate Rater(15);
    ros::Rate visRater(2);
	poseUpdater = nh.createTimer(Rater, &rtrrtp::updateRobotPoseCB, this);
	visualizer = nh.createTimer(visRater, &rtrrtp::visualizationCB, this);

}

rtrrtp::rtrrtp(){
    init();
    std::cout << bold << green
              << "SEAP initialization accomplished"
              << reset << std::endl;
    std::thread executeThread(&rtrrtp::ExecuteThread, this);
    executeThread.detach(); // Detach the sub-thread for parallelism
}

rtrrtp::~rtrrtp(){
    delete rtRRT;
    closeList.clear();
    global_frame.clear();
    selectedBranch_nodes.clear();
}

void rtrrtp::resetResource(){
    delete rtRRT;
    std::cout << "Delete rtRRT" << std::endl;
    rtRRT = new rtrrt;
    target_node = NULL;
    global_frame = globalFrame;
    base_frame = baseFrame;
    useFinalGoal = false;
    useSubGoal = false;
    cur_state = PlannerState::WAITING;
}

bool rtrrtp::resetPlanner(std_srvs::Empty::Request& req,
                          std_srvs::Empty::Response& res){
    if(cur_state == PlannerState::ACTIVE){
        ROS_WARN("Planner is active! Please stop planner firstly.");
        return false;
    }
    std::cout << cyan 
              << "Reseting planner"
              << reset << std::endl;
    mtx.lock();
    rtRRT->sub_grid_map.shutdown(); // Close grid map spin Channel
    closeList.clear();
    globalFrontier.clear();
    resetResource();
    mtx.unlock();
    return true;
}

bool rtrrtp::stopPlanner(std_srvs::Empty::Request& req,
                          std_srvs::Empty::Response& res){
    std::cout << yellow 
              << "Stop path planning"
              << reset << std::endl;
    mtx.lock();
    target_node = NULL;
    cur_state = PlannerState::WAITING;
    newGoalReceived = false;
    rtRRT->newGoalReceived = false;
    mtx.unlock();
    stayCurrentPose();
    return true;
}

bool rtrrtp::getStateHandler(sea_planner::get_state::Request& req,
                             sea_planner::get_state::Response& state_res){
    state_res.stamp = ros::Time::now();
    if(cur_state == PlannerState::ACTIVE){
        state_res.state = "ACTIVE";
    }
    if(cur_state == PlannerState::WAITING){
        state_res.state = "WAITING";
    }
    state_res.estimated_distance = distance(robot_pose.x, rtRRT->goal.position.x, robot_pose.y, rtRRT->goal.position.y);
    return true;
}

void rtrrtp::ExecuteThread(){
    while(ros::ok()){
        if(cur_state == PlannerState::ACTIVE){
            float timeStart = ros::Time::now().toSec();
            executeCycle();
            float timeEnd = ros::Time::now().toSec();
            if(cur_state == PlannerState::ACTIVE){
                ROS_INFO("Navigation Duration: %f", timeEnd-timeStart);
                std::cout << bold << green 
                            << "Arrived Goal" 
                            << reset << std::endl;
            }
            mtx.lock();
            newGoalReceived = false;
            rtRRT->newGoalReceived = false;
            cur_state = PlannerState::WAITING;  // Reset flag
            mtx.unlock();
        }
    }
}

void rtrrtp::stayCurrentPose(){
    way_point.header.frame_id = global_frame;
    way_point.header.stamp = ros::Time::now();
    way_point.point.x = robot_pose.x;
    way_point.point.y = robot_pose.y;
    way_point.point.z = robot_pose.z;
    pub_waypoint.publish(way_point);
}

void rtrrtp::executeCycle(){
    if(updatedPose){
        // Navigate to final goal
        // bool record_gen_time = false;
	    // float record_generation_start = ros::Time::now().toSec();
        int random_counter = 0;
        float disToGoal = distance(robot_pose.x, rtRRT->goal.position.x, robot_pose.y, rtRRT->goal.position.y);
        while(cur_state == PlannerState::ACTIVE && disToGoal > goalTolerance && ros::ok()){
            // For Initializing Tree with enough nodes and edge
            while(rtRRT->node_set.size()<=minTreeNodeNum && ros::ok()){
                rtRRT->executeTreeUpdate();
            }
            // Select navigation type: goal or subgoal
            float timeKeeper = ros::Time::now().toSec();
            selectNavType();
            if(!globalFrontier.empty()){
                validateFrontier();
            }

            if(useSubGoal && !useFinalGoal && rtRRT->current_target_node==NULL){
                while((ros::Time::now().toSec() - timeKeeper) < 1.0){
                    // For exploring env, prolong the time for tree growing 
                    rtRRT->executeTreeUpdate();
                }
                //! Random node as target exp2
                // random_counter--;
                // if(random_counter < -5){
                //     while(rtRRT->current_target_node==NULL){
                //         globalFrontier.clear();
                //         rtRRT->getFrontierNode();
                //         if(rtRRT->frontierNodes.size()!=0){
                //             int index = round((((double)rand()) / ((double)RAND_MAX))*(rtRRT->frontierNodes.size()-1));
                //             for(auto it=rtRRT->frontierNodes.begin(); it!=rtRRT->frontierNodes.end(); it++){
                //                 index--;
                //                 if(index<=0){
                //                     rtRRT->current_target_node = (*it).first;
                //                     break;
                //                 }
                //             }
                //         }
                //     }
                //     random_counter = 1;
                // }
                // else{
                    //! Ours frontier method
                rtRRT->getFrontierNode();
                for(auto iter1=rtRRT->frontierNodes.begin(); iter1!=rtRRT->frontierNodes.end(); iter1++){
                    // No found, add to global frontier
                    if(globalFrontier.find((*iter1).first) == globalFrontier.end()){
                        globalFrontier.insert(*iter1);
                    }
                    // Found, add to global frontier after delete operation
                    else{
                        globalFrontier.erase((*iter1).first);
                        globalFrontier.insert(*iter1);
                    }
                }
                // Find best frontier as target node
                float totalDis = 0;
                float totalGradient = 0;
                for(auto iter2=globalFrontier.begin(); iter2!=globalFrontier.end(); iter2++){
                    // Except nodes in closeList
                    auto res = find(closeList.begin(), closeList.end(), (*iter2).first);
                    if(res != closeList.end()){
                        continue;
                    }
                    totalDis += (*iter2).first->disToRoot;
                    totalGradient += (*iter2).first->gradientToRoot;
                }
                float best_cost = inf;
                rtrrt_ns::Node *best_node = NULL;
                for(auto iter3=globalFrontier.begin(); iter3!=globalFrontier.end(); iter3++){
                    // Except nodes in closeList
                    // auto res = find(closeList.begin(), closeList.end(), (*iter3).first);
                    // if(res != closeList.end()){
                    //     continue;
                    // }
                    //TODO evaluation Func should be desiged independently
                    float distance_heuristic = util::distance((*iter3).first->state.x, final_goal.pose.position.x, (*iter3).first->state.y, final_goal.pose.position.y);
                    float costVal = costFunc((*iter3).first->disToRoot, distance_heuristic, (*iter3).first->gradientToRoot, totalDis, totalGradient);
                    //! Nearest node for Goal (Greedy)
                    // float costVal = util::distance((*iter3).first->state.x, final_goal.pose.position.x, (*iter3).first->state.y, final_goal.pose.position.y);
                    //! Max info method
                    // float costVal = -1 * 0.9 * util::getInfoGain((*iter3).first->state.x, (*iter3).first->state.y, rtRRT->full_grid_map, "elevation") + 0.1* distance_heuristic;
                    //! Nearest frontier for robot
                    // float costVal = util::distance((*iter3).first->state.x, robot_pose.x, (*iter3).first->state.y, robot_pose.y);
                    if(costVal < best_cost){
                        best_cost = costVal;
                        best_node = (*iter3).first;
                    }
                }
                rtRRT->current_target_node = best_node;
                // }
            }
            // If have node close enough to final_goal, it will become target
            if(!useSubGoal && useFinalGoal){
                // Update frontiers
                rtRRT->getFrontierNode();
                for(auto iter1=rtRRT->frontierNodes.begin(); iter1!=rtRRT->frontierNodes.end(); iter1++){
                    // No found, add to global frontier
                    if(globalFrontier.find((*iter1).first) == globalFrontier.end()){
                        globalFrontier.insert(*iter1);
                    }
                    // Found, add to global frontier after delete operation
                    else{
                        globalFrontier.erase((*iter1).first);
                        globalFrontier.insert(*iter1);
                    }
                }

                rtrrt_ns::Node *closeNode = rtRRT->findTarget();
                if(closeNode != NULL){
                    rtRRT->current_target_node = closeNode;
                }
            }
            rtRRT->executeTreeUpdate();
            // Driving to target_node
            float DisToTarget = inf;
            if(cur_state!=PlannerState::WAITING){
                target_node = rtRRT->current_target_node;
                DisToTarget = util::distance(robot_pose.x, target_node->state.x, robot_pose.y, target_node->state.y);
            }
            bool state = ros::ok();
            while(cur_state == PlannerState::ACTIVE && DisToTarget > goalTolerance && state && target_node!=NULL && target_node->nodeState!=NodeState::OBSTACLE){
                rtRRT->executeTreeUpdate();
                if(target_node != NULL){
                    // Found target node, output path
                    // if(!record_gen_time){
                    //     float record_generation_end = ros::Time::now().toSec();
                    //     std::cout << "The first path generation time: " << record_generation_end-record_generation_start << std::endl;
                    //     record_gen_time = true;
                    // }
                    searchPath();
                }
                if(target_node!=NULL && cur_state!=PlannerState::WAITING){
                    DisToTarget = util::distance(robot_pose.x, target_node->state.x, robot_pose.y, target_node->state.y);
                    rtRRT->executeTreeUpdate();
                }
                state = ros::ok();
            }
            if(useSubGoal && !useFinalGoal && rtRRT->current_target_node!=NULL){
                closeList.push_back(rtRRT->current_target_node);
            }
            useFinalGoal = false;
            useSubGoal = false;
            target_node = NULL;
            rtRRT->current_target = NULL;
            rtRRT->current_target_node = NULL;
            // ros::Duration(0.001).sleep(); // Control rrt grow speed
            disToGoal = distance(robot_pose.x, rtRRT->goal.position.x, robot_pose.y, rtRRT->goal.position.y);
        }
        if(cur_state == PlannerState::ACTIVE){
            std::cout << "Node Num: " << rtRRT->node_set.size() << ", Edge Num: " << rtRRT->edge_set.size() << std::endl;
            std::cout << "Executed Path node num: " << Path_node.size() << std::endl;
        }
        // Reset all resource
        target_node = NULL;
        rtRRT->current_target = NULL;
        rtRRT->current_target_node = NULL;
    }
}

void rtrrtp::searchPath(){
    // Empty selectedBranch_nodes set for updating path
    selectedBranch_nodes.clear();
    if(target_node != NULL){
        rtrrt_ns::Node *current_iterator = target_node;
        while(current_iterator != NULL){
            // According to find parent one by one until retrace root node(positive sequence)
            if(selectedBranch_nodes.empty()){
                selectedBranch_nodes.push_back(current_iterator);
            }
            else{
                selectedBranch_nodes.insert(selectedBranch_nodes.begin(), current_iterator);
            }
            current_iterator = current_iterator->parent;
        }
        if(!selectedBranch_nodes.empty()){
            publishPath();
            publishWaypoint();
        }
    }
    // Change rtrrt root
    if(!selectedBranch_nodes.empty()){
        if(selectedBranch_nodes.size()>1){
            float dis = util::distance(robot_pose.x, selectedBranch_nodes[1]->state.x, robot_pose.y, selectedBranch_nodes[1]->state.y);
            if(dis < changRootTolerance){
                rtRRT->changeRoot(selectedBranch_nodes[1]);
                Path_node.push_back(rtRRT->root);
            }
        }
        else{
            float dis = util::distance(robot_pose.x, selectedBranch_nodes[0]->state.x, robot_pose.y, selectedBranch_nodes[0]->state.y);
            if(dis < changRootTolerance){
                rtRRT->changeRoot(selectedBranch_nodes[0]);
                Path_node.push_back(rtRRT->root);
            }
        }
    }
}

void rtrrtp::publishPath(){
    rtrrt_path.header.frame_id = global_frame;
    rtrrt_path.header.stamp = ros::Time::now();
    rtrrt_path.poses.clear();
    geometry_msgs::PoseStamped single_pose;
    if(selectedBranch_nodes.size()>1){
        // Add robot pose to rtrrt_path
        single_pose.header.frame_id = global_frame;
        single_pose.header.stamp = ros::Time::now();
        single_pose.pose.position.x = robot_pose.x;
        single_pose.pose.position.y = robot_pose.y;
        single_pose.pose.position.z = robot_pose.z; 
        tf::Quaternion tf_q = tf::createQuaternionFromYaw(robot_pose.yaw).normalized();
        single_pose.pose.orientation.x = tf_q.getX();
        single_pose.pose.orientation.y = tf_q.getY();
        single_pose.pose.orientation.z = tf_q.getZ();
        single_pose.pose.orientation.w = tf_q.getW();
        rtrrt_path.poses.push_back(single_pose);
        // Add rtrrt node except root node
        for(auto iter=selectedBranch_nodes.begin()+1; iter!=selectedBranch_nodes.end(); iter++){
            single_pose.header.frame_id = global_frame;
            single_pose.header.stamp = ros::Time::now();
            // Get position
            single_pose.pose.position.x = (*iter)->state.x;
            single_pose.pose.position.y = (*iter)->state.y;
            single_pose.pose.position.z = robot_pose.z;
            // Get quaternion
            if(iter == selectedBranch_nodes.end()-1){
                float deltY = (*iter)->state.y - (*(iter-1))->state.y;
                float deltX = (*iter)->state.x - (*(iter-1))->state.x;
                float yaw_angle = atan2(deltY, deltX);
                tf::Quaternion tf_q = tf::createQuaternionFromYaw(yaw_angle).normalized();
                single_pose.pose.orientation.x = tf_q.getX();
                single_pose.pose.orientation.y = tf_q.getY();
                single_pose.pose.orientation.z = tf_q.getZ();
                single_pose.pose.orientation.w = tf_q.getW();    
                rtrrt_path.poses.push_back(single_pose);

                // Add final_goal pose to path
                // single_pose.header.frame_id = global_frame;
                // single_pose.header.stamp = ros::Time::now();
                // // Get position
                // single_pose.pose.position.x = rtRRT->current_target->x;
                // single_pose.pose.position.y = rtRRT->current_target->y;
                // single_pose.pose.position.z = robot_pose.z;
                // rtrrt_path.poses.push_back(single_pose);
            }
            else{
                float deltY = (*(iter+1))->state.y - (*iter)->state.y;
                float deltX = (*(iter+1))->state.x - (*iter)->state.x;
                float yaw_angle = atan2(deltY, deltX);
                tf::Quaternion tf_q = tf::createQuaternionFromYaw(yaw_angle).normalized();
                single_pose.pose.orientation.x = tf_q.getX();
                single_pose.pose.orientation.y = tf_q.getY();
                single_pose.pose.orientation.z = tf_q.getZ();
                single_pose.pose.orientation.w = tf_q.getW();    
                rtrrt_path.poses.push_back(single_pose);
            }
        }
    }
    else{
        // Add robot pose to rtrrt_path
        single_pose.header.frame_id = global_frame;
        single_pose.header.stamp = ros::Time::now();
        single_pose.pose.position.x = robot_pose.x;
        single_pose.pose.position.y = robot_pose.y;
        single_pose.pose.position.z = robot_pose.z; 
        tf::Quaternion tf_q = tf::createQuaternionFromYaw(robot_pose.yaw).normalized();
        single_pose.pose.orientation.x = tf_q.getX();
        single_pose.pose.orientation.y = tf_q.getY();
        single_pose.pose.orientation.z = tf_q.getZ();
        single_pose.pose.orientation.w = tf_q.getW();
        rtrrt_path.poses.push_back(single_pose);
        // Add final_goal pose to path
        single_pose.header.frame_id = global_frame;
        single_pose.header.stamp = ros::Time::now();
        // Get position
        single_pose.pose.position.x = target_node->state.x;
        single_pose.pose.position.y = target_node->state.y;
        single_pose.pose.position.z = target_node->state.z;
        rtrrt_path.poses.push_back(single_pose);
    }
    pub_rtrrt_path.publish(rtrrt_path);
}

void rtrrtp::publishWaypoint(){
        float disTopoint = util::distance(robot_pose.x, selectedBranch_nodes[1]->state.x, robot_pose.y, selectedBranch_nodes[1]->state.y);
        if(disTopoint > waypointTolerance){
            if(rtrrt_path.poses.size()>2){
                way_point.header.frame_id = global_frame;
                way_point.header.stamp = ros::Time::now();
                way_point.point.x = rtrrt_path.poses[1].pose.position.x;
                way_point.point.y = rtrrt_path.poses[1].pose.position.y;
                way_point.point.z = rtrrt_path.poses[1].pose.position.z;
                pub_waypoint.publish(way_point);
            }
            else{
                way_point.header.frame_id = global_frame;
                way_point.header.stamp = ros::Time::now();
                way_point.point.x = rtrrt_path.poses.back().pose.position.x;
                way_point.point.y = rtrrt_path.poses.back().pose.position.y;
                way_point.point.z = rtrrt_path.poses.back().pose.position.z;
                pub_waypoint.publish(way_point);
            }
            
        }
}

void rtrrtp::selectNavType(){
    // Find near nodes of final goal
    float totalDistance = 0;
    float totalGradient = 0;
    float min_cost = inf;
    rtrrt_ns::Node *best_node = NULL;
    kdres* near_nodes = rtRRT->getNeiborNodes(final_goal.pose.position.x, final_goal.pose.position.y, findNearNodeThre);
    double pos[3];
    int near_nodes_size = kd_res_size(near_nodes);
    if (near_nodes_size <= 0)
    {
        useSubGoal = true;
        useFinalGoal = false;
        kd_res_free(near_nodes);
        return;
    }
    else{
        while(!kd_res_end(near_nodes)){
            rtrrt_ns::Node *node = (rtrrt_ns::Node*)kd_res_item(near_nodes, pos);
            // Except OBSTACLE nodes
            if(node->nodeState == NodeState::OBSTACLE){
                kd_res_next(near_nodes);
                continue;
            }
            totalDistance += node->disToRoot;
            totalGradient += node->gradientToRoot;
            kd_res_next(near_nodes);
        }
        kd_res_rewind(near_nodes);
        // Iterate all neibor nodes
        while(!kd_res_end(near_nodes)){
            useFinalGoal = true;
            useSubGoal = false;

            rtrrt_ns::Node *near = (rtrrt_ns::Node*)kd_res_item(near_nodes, pos);
            // Except OBSTACLE nodes
            if(near->nodeState == NodeState::OBSTACLE){
                kd_res_next(near_nodes);
                continue;
            }
            float distance_heuristic = util::distance(near->state.x, final_goal.pose.position.x, near->state.y, final_goal.pose.position.y);
            float costVal = costFunc(near->disToRoot, distance_heuristic, near->gradientToRoot, totalDistance, totalGradient);
            // std::cout << "costVal: " << costVal << std::endl;
            if(costVal < min_cost){
                min_cost = costVal;
                best_node = near;
            }
            kd_res_next(near_nodes);			
        }
        kd_res_free(near_nodes);
        // Drive to final goal surround
        rtRRT->current_target_node = best_node;
    }
}

void rtrrtp::validateFrontier(){
    // Iterate all global frontier
    auto node=globalFrontier.begin();
    while(node!=globalFrontier.end() && ros::ok()){
        rtrrt_ns::Node *nearest_obstacle = rtRRT->getObstacleNode((*node).first->state.x, (*node).first->state.y);
        float dis = util::distance((*node).first->state.x, nearest_obstacle->state.x, (*node).first->state.y, nearest_obstacle->state.y);
        if(dis <= inflationRadius){
            node = globalFrontier.erase(node);
            continue;
        }
        Position frontier_pos{(*node).first->state.x, (*node).first->state.y};
        if(!rtRRT->full_grid_map.isInside(frontier_pos)){
            node++;
            continue;
        }
        // If frontier node is not leaf, remove it
        if((*node).first->nodeState != NodeState::LEAF){
            node = globalFrontier.erase(node); // For erase method will return next iterator after resleased
            continue;
        }
        // Check frontier information gain
        kdres* near_nodes = rtRRT->getNeiborNodes((*node).first->state.x, (*node).first->state.y, exploredAreaRadius);
        int neibor_nodes_size = kd_res_size(near_nodes);
        // For avoiding explored area
        if(neibor_nodes_size < exploredAreaThre){
            Position leaf_posi((*node).first->state.x, (*node).first->state.y);
            Index leaf_index;
            Length leng(frontierRadius, frontierRadius);
            bool isSuccessful;
            auto sub_map = rtRRT->full_grid_map.getSubmap(leaf_posi, leng, leaf_index ,isSuccessful);
            Size submap_size = sub_map.getSize();
            // No enough information gain will be erase
            if(submap_size[0] == submap_size[1]){
                node = globalFrontier.erase(node);
                continue;      
            }
            else{
                float information_gain = util::getInfoGain((*node).first->state.x, (*node).first->state.y, rtRRT->full_grid_map, "elevation");
                if(information_gain < informationThre){
                    node = globalFrontier.erase(node);
                    continue;
                }
            }
        }
        else{
            node = globalFrontier.erase(node);
            continue;
        }
        node++;
    }

}

void rtrrtp::visualizationCB(const ros::TimerEvent &event){
	if(pub_selected_branch.getNumSubscribers()==0){
		return; // if all topics of visualizing tree item have no subscriber, then don't publish
	}
    visualization_msgs::MarkerArray selectedBranch_marker;
    visualization_msgs::Marker selectedBranch_edge_marker;
    visualization_msgs::Marker selectedBranch_node_marker;
    visualization_msgs::Marker ref_line_marker;
    visualization_msgs::Marker goal_marker;
    visualization_msgs::Marker waypoint_marker;
    visualization_msgs::Marker target_node_marker;
    visualization_msgs::Marker global_frontier_marker;

    // Edge Marker
	selectedBranch_edge_marker.header.frame_id = global_frame;
	selectedBranch_edge_marker.header.stamp = ros::Time::now();
	selectedBranch_edge_marker.type = selectedBranch_edge_marker.LINE_LIST;
	selectedBranch_edge_marker.action = selectedBranch_edge_marker.ADD;
	selectedBranch_edge_marker.lifetime = ros::Duration();
	selectedBranch_edge_marker.color.a = 1.0;
	selectedBranch_edge_marker.color.r = 0.0 / 255.0;
	selectedBranch_edge_marker.color.g = 255.0 / 255.0;
	selectedBranch_edge_marker.color.b = 255.0 / 255.0;
	selectedBranch_edge_marker.scale.x = 0.1;
	selectedBranch_edge_marker.scale.y = 0.1;
	selectedBranch_edge_marker.scale.z = 0.1;
    selectedBranch_edge_marker.id = 0;
	selectedBranch_edge_marker.pose.orientation.w = 1;
	selectedBranch_edge_marker.points.clear();
    // Node Marker
	selectedBranch_node_marker.header.frame_id = global_frame;
	selectedBranch_node_marker.header.stamp = ros::Time::now();
	selectedBranch_node_marker.type = selectedBranch_node_marker.SPHERE_LIST;
	selectedBranch_node_marker.action = selectedBranch_node_marker.ADD;
	selectedBranch_node_marker.lifetime = ros::Duration();
	selectedBranch_node_marker.color.a = 1.0;
	selectedBranch_node_marker.color.r = 255.0 / 255.0;
	selectedBranch_node_marker.color.g = 255.0 / 255.0;
	selectedBranch_node_marker.color.b = 0.0 / 255.0;
	selectedBranch_node_marker.scale.x = 0.2;
	selectedBranch_node_marker.scale.y = 0.2;
	selectedBranch_node_marker.scale.z = 0.2;
    selectedBranch_node_marker.id = 1;
	selectedBranch_node_marker.pose.orientation.w = 1;
	selectedBranch_node_marker.points.clear();
    // Reference Line
	ref_line_marker.header.frame_id = global_frame;
	ref_line_marker.header.stamp = ros::Time::now();
	ref_line_marker.type = ref_line_marker.LINE_LIST;
	ref_line_marker.action = ref_line_marker.ADD;
	ref_line_marker.lifetime = ros::Duration();
	ref_line_marker.color.a = 1.0;
	ref_line_marker.color.r = 255.0 / 255.0;
	ref_line_marker.color.g = 0.0 / 255.0;
	ref_line_marker.color.b = 0.0 / 255.0;
	ref_line_marker.scale.x = 0.1;
	ref_line_marker.scale.y = 0.1;
	ref_line_marker.scale.z = 0.1;
    ref_line_marker.id = 0;
	ref_line_marker.pose.orientation.w = 1;
	ref_line_marker.points.clear();

    // Goal Marker
	goal_marker.header.frame_id = global_frame;
	goal_marker.header.stamp = ros::Time::now();
	goal_marker.type = goal_marker.SPHERE;
	goal_marker.action = goal_marker.ADD;
	goal_marker.lifetime = ros::Duration();
	goal_marker.color.a = 1.0;
	goal_marker.color.r = 255.0 / 255.0;
	goal_marker.color.g = 0.0 / 255.0;
	goal_marker.color.b = 237.0 / 255.0;
	goal_marker.scale.x = 0.5;
	goal_marker.scale.y = 0.5;
	goal_marker.scale.z = 0.5;
	goal_marker.pose.orientation.w = 1;
    // Waypoint Marker
	waypoint_marker.header.frame_id = global_frame;
	waypoint_marker.header.stamp = ros::Time::now();
	waypoint_marker.type = waypoint_marker.SPHERE;
	waypoint_marker.action = waypoint_marker.ADD;
	waypoint_marker.lifetime = ros::Duration();
	waypoint_marker.color.a = 1.0;
	waypoint_marker.color.r = 43.0 / 255.0;
	waypoint_marker.color.g = 57.0 / 255.0;
	waypoint_marker.color.b = 255.0 / 255.0;
	waypoint_marker.scale.x = 0.3;
	waypoint_marker.scale.y = 0.3;
	waypoint_marker.scale.z = 0.3;
	waypoint_marker.pose.orientation.w = 1;

    // Target Marker
    if(target_node != NULL){
        target_node_marker.header.frame_id = global_frame;
        target_node_marker.header.stamp = ros::Time::now();
        target_node_marker.type = target_node_marker.SPHERE;
        target_node_marker.action = target_node_marker.ADD;
        target_node_marker.lifetime = ros::Duration();
        target_node_marker.color.a = 1.0;
        target_node_marker.color.r = 0.0 / 255.0;
        target_node_marker.color.g = 255.0 / 255.0;
        target_node_marker.color.b = 0.0 / 255.0;
        target_node_marker.scale.x = 0.5;
        target_node_marker.scale.y = 0.5;
        target_node_marker.scale.z = 0.5;
        target_node_marker.pose.orientation.x = 0;
        target_node_marker.pose.orientation.y = 0;
        target_node_marker.pose.orientation.z = 0;
        target_node_marker.pose.orientation.w = 1;
        target_node_marker.pose.position.x = target_node->state.x;
        target_node_marker.pose.position.y = target_node->state.y;
        target_node_marker.pose.position.z = target_node->state.z;
        pub_target_marker.publish(target_node_marker);
    }

    // Global frontier Marker
    if(!globalFrontier.empty()){
        global_frontier_marker.header.frame_id = global_frame;
        global_frontier_marker.header.stamp = ros::Time::now();
        global_frontier_marker.type = global_frontier_marker.SPHERE_LIST;
        global_frontier_marker.action = global_frontier_marker.ADD;
        global_frontier_marker.lifetime = ros::Duration();
        global_frontier_marker.color.a = 1.0;
        global_frontier_marker.color.r = 129.0 / 255.0;
        global_frontier_marker.color.g = 208.0 / 255.0;
        global_frontier_marker.color.b = 241.0 / 255.0;
        global_frontier_marker.scale.x = 0.2;
        global_frontier_marker.scale.y = 0.2;
        global_frontier_marker.scale.z = 0.2;
        global_frontier_marker.pose.orientation.w = 1;
        global_frontier_marker.points.clear();
        for(auto i=globalFrontier.begin(); i!=globalFrontier.end(); i++){
            geometry_msgs::Point single_point;
            single_point.x = i->first->state.x;
            single_point.y = i->first->state.y;
            single_point.z = i->first->state.z;
            global_frontier_marker.points.push_back(single_point);
        }
        pub_global_frontier.publish(global_frontier_marker);
    }

    // Add data information
    if(!selectedBranch_nodes.empty()){
        for(auto iter=selectedBranch_nodes.begin(); iter!=selectedBranch_nodes.end()-1; iter++){
            geometry_msgs::Point single_point;
            single_point.x = (*iter)->state.x;
            single_point.y = (*iter)->state.y;
            single_point.z = (*iter)->state.z;
            selectedBranch_node_marker.points.push_back(single_point);
            selectedBranch_edge_marker.points.push_back(single_point);
            single_point.x = (*(iter+1))->state.x;
            single_point.y = (*(iter+1))->state.y;
            single_point.z = (*(iter+1))->state.z;        
            selectedBranch_edge_marker.points.push_back(single_point);
        }
        selectedBranch_marker.markers.push_back(selectedBranch_node_marker);
        selectedBranch_marker.markers.push_back(selectedBranch_edge_marker);
        pub_selected_branch.publish(selectedBranch_marker);
    }

    if(!selectedBranch_nodes.empty() && target_node!=NULL){
        float segment_length = 0.3;
        float disToGoal = util::distance(target_node->state.x, final_goal.pose.position.x, target_node->state.y, final_goal.pose.position.y);
        geometry_msgs::Point line_point;
        // It will connect directly if distance is short
        if(disToGoal <= segment_length*2){
            line_point.x = target_node->state.x;
            line_point.y = target_node->state.y;
            line_point.z = target_node->state.z;
            ref_line_marker.points.push_back(line_point);
            line_point = final_goal.pose.position;
            ref_line_marker.points.push_back(line_point); 
            pub_ref_line.publish(ref_line_marker);    
        }
        else{   // Connect using virtual line form
            float Theta = atan2(final_goal.pose.position.y-target_node->state.y, final_goal.pose.position.x-target_node->state.x);
            float segment_deltX = segment_length*cos(Theta);
            float segment_deltY = segment_length*sin(Theta);
            float PointX = target_node->state.x;
            float PointY = target_node->state.y;
            float PointZ = target_node->state.z;
            float last_disToEnd = inf;
            float curr_disToEnd = util::distance(PointX, final_goal.pose.position.x, PointY, final_goal.pose.position.y);
            while(curr_disToEnd <= last_disToEnd){
                last_disToEnd = curr_disToEnd;
                line_point.x = PointX;
                line_point.y = PointY;
                line_point.z = PointZ;
                ref_line_marker.points.push_back(line_point); 
                PointX += segment_deltX;
                PointY += segment_deltY;
                curr_disToEnd = util::distance(PointX, final_goal.pose.position.x, PointY, final_goal.pose.position.y);
            }
            if(ref_line_marker.points.size()%2 != 0){
                ref_line_marker.points.push_back(*(ref_line_marker.points.end()-1));
            }
            pub_ref_line.publish(ref_line_marker);
        }
        
    }

    goal_marker.pose = final_goal.pose;
    pub_goal_marker.publish(goal_marker);
    waypoint_marker.pose.position = way_point.point;
    pub_waypoint_marker.publish(waypoint_marker);
}

void rtrrtp::updateRobotPoseCB(const ros::TimerEvent &event){
    rtRRT->updateRobotPose();
    robot_pose = rtRRT->robot_pose;
    updatedPose = true;
}

void rtrrtp::goalHandler(const geometry_msgs::PoseStampedConstPtr &goal_point){
	if(!updatedPose)
	{
		ROS_WARN("Cannot get the robot pose.");
		return;
	}
	final_goal.pose.position.x = goal_point->pose.position.x;
	final_goal.pose.position.y = goal_point->pose.position.y;
	final_goal.pose.position.z = goal_point->pose.position.z+0.5;
	final_goal.pose.orientation = goal_point->pose.orientation;
    // rtRRT->updateGoal(final_goal);
	rtRRT->goal.position.x = goal_point->pose.position.x;
	rtRRT->goal.position.y = goal_point->pose.position.y;
	rtRRT->goal.position.z = goal_point->pose.position.z;
	rtRRT->goal.orientation = goal_point->pose.orientation;

    printf("\033[2J\033[1;1H"); // Clear all print msg
	std::cout << bold << magenta
              << "Received Goal: " << rtRRT->goal.position.x << ", " << rtRRT->goal.position.y 
              << reset << std::endl;
    if(cur_state == PlannerState::ACTIVE){
        mtx.lock();
        target_node = NULL;
        useSubGoal = false;
        useFinalGoal = false;
        cur_state = PlannerState::WAITING; // Set flag to stop
        mtx.unlock();
        stayCurrentPose(); // Stop locomotion
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Wait for ending executeCycle
    }

    rtRRT->newGoalReceived = true;
	newGoalReceived = true;

    std::cout << green
              << "Start path planning"
              << reset << std::endl;
        cur_state = PlannerState::ACTIVE;   // Active planner
}


