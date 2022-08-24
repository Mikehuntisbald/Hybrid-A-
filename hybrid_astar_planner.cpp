#include "hybrid_astar_planner.h"
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(astar_planner::HybridAstarPlanner, nav_core::BaseGlobalPlanner)

namespace astar_planner{
    HybridAstarPlanner::HybridAstarPlanner(){}
    // Don't need this constructor here. Movebase will call initialize.
    /*HybridAstarPlanner::HybridAstarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        initialize(name, costmap_ros);
    }*/

    void HybridAstarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        if(!initialized_){
            divide=8;
            range=M_PI/2;
            chordl=0.2;///0.13;
            phase = range/divide;
            ROS_INFO("initializing");
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            resolution=costmap_->getResolution();
            width = costmap_->getSizeInCellsX();
            height = costmap_->getSizeInCellsY();
            map_size = width * height;
            OGM.resize(map_size);

            vector<unsigned int> coo;
            int count;
            for (int i = 0; i < width; i++)
            {
                for (int j = 0; j < height; j++)
                {
                    //ROS_INFO("getting cost");
                    unsigned int cost = costmap_->getCost(i, j);
                    if(cost>0){
                        coo.push_back(cost);
                        count++;
                    }
                    //get_cost << cost << endl;
                    //cout << "i:, j:" << cost << endl;
                    if (cost <= 50)
                        OGM[j * width + i] = true;
                    else{
                        //ROS_INFO("%d is false",j * width + i);
                        OGM[j * width + i] = false;
                    }
                }
            }
            /*auto it = coo.begin();
            for(int i=0; i<count/2 ; i++){
                it++;
            }
            unsigned int mid;
            mid = *it;
            ROS_INFO("%d",mid);*/

            frame_id_ = costmap_ros->getGlobalFrameID();

            ros::NodeHandle private_nh("~/" + name);

            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

            initialized_ = true;
        }
        else
            ROS_WARN("This planner has already been initialized... doing nothing");
    }
    //start is current pose. goal is provided by user, both of which are global pose. plan is where plan is stored.
    bool HybridAstarPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
        if (hypot(start.pose.position.x - goal.pose.position.x, start.pose.position.y - goal.pose.position.y)<(0.35)||(getGoalAngleDifference(
            getSe3(start), getSe3(goal))>(M_PI/2))) {

            ///||(getGoalAngleDifference(
                    ///getSe3(start), getSe3(goal))>(M_PI/3))
            if (!initialized_) {
                ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
                return false;
            }
            //world coordinate to map coordinate to index
            ROS_INFO("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y,
                     goal.pose.position.x, goal.pose.position.y);
            double wx = start.pose.position.x;
            double wy = start.pose.position.y;
            unsigned int start_x, start_y;
            costmap_->worldToMap(wx, wy, start_x, start_y);
            int start_index = costmap_->getIndex(start_x, start_y);


            wx = goal.pose.position.x;
            wy = goal.pose.position.y;

            unsigned int goal_x, goal_y;
            costmap_->worldToMap(wx, wy, goal_x, goal_y);
            int goal_index = costmap_->getIndex(goal_x, goal_y);

            //meijibayong
            //create 2 vectors the sequence is index.
            vector<float> gCost(map_size, infinity);
            vector<int> cameFrom(map_size, -1);
            //vector <vector<geometry_msgs::PoseStamped>> trajectory(map_size);

            //openlist
            multiset <Node> openlist;

            gCost[start_index] = 0;
            //Initialize currentNode and put it in openlist
            Node currentNode;
            currentNode.index = start_index;
            currentNode.cost = gCost[start_index] + 0;
            openlist.insert(currentNode);

            plan.clear();

            while (!openlist.empty()) {
                // Take the element from the top(begin is the least cost)
                currentNode = *openlist.begin();
                //Delete the element from the top
                openlist.erase(openlist.begin());
                if (currentNode.index == goal_index) {
                    break;
                }
                // Get neighbors
                vector<int> neighborIndexes = get_neighbors(currentNode.index);

                for (int i = 0; i < neighborIndexes.size(); i++) {
                    //if not expanded
                    if (cameFrom[neighborIndexes[i]] == -1) {
                        //predecessor cost+ movecost
                        unsigned int nx, ny;
                        costmap_->indexToCells(neighborIndexes[i], nx, ny);
                        gCost[neighborIndexes[i]] =
                                gCost[currentNode.index] + getMoveCost(currentNode.index, neighborIndexes[i]);
                        Node nextNode;
                        nextNode.index = neighborIndexes[i];
                        //nextNode.cost = gCost[neighborIndexes[i]];    //Dijkstra Algorithm
                        //manhattan distance
                        nextNode.cost = gCost[neighborIndexes[i]] +
                                        getHeuristic(neighborIndexes[i], goal_index);    //A* Algorithm
                        cameFrom[neighborIndexes[i]] = currentNode.index;
                        //put expanded node into openlist
                        openlist.insert(nextNode);
                    }
                }
            }
            //if goal does not come from any node
            if (cameFrom[goal_index] == -1) {
                cout << "Goal not reachable, failed making a global path." << endl;
                return false;
            }

            if (start_index == goal_index)
                return false;
            //Finding the best path, take note of the index
            vector<int> bestPath;
            currentNode.index = goal_index;
            while (currentNode.index != start_index) {
                bestPath.push_back(cameFrom[currentNode.index]);
                currentNode.index = cameFrom[currentNode.index];
            }
            reverse(bestPath.begin(), bestPath.end());
            vector<geometry_msgs::PoseStamped> bestplan;
            ros::Time plan_time = ros::Time::now();
            for (int i : bestPath) {
                unsigned int tmp1, tmp2;
                costmap_->indexToCells(i, tmp1, tmp2);
                double x, y;
                costmap_->mapToWorld(tmp1, tmp2, x, y);

                geometry_msgs::PoseStamped pose;
                pose.header.stamp = plan_time;
                pose.header.frame_id = costmap_ros_->getGlobalFrameID();
                pose.pose.position.x = x;
                pose.pose.position.y = y;
                pose.pose.position.z = 0.0;

                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;

                bestplan.push_back(pose);
            }
            CalcSpline(bestplan,plan);
            plan.push_back(goal);
            publishPlan(plan);
            return true;

        } else {
            if (!initialized_) {
                ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
                return false;
            }
            ROS_INFO("start Hybrid A*");
            ///generate all possible trajectories to neighbors, divide+1 in total
            vector<vector<Eigen::Isometry3d>> trajectory;
            trajectoryGen(trajectory);
            //refine(trajectory);

            geometry_msgs::PoseStamped robot_vel;
            odom_helper_.getRobotVel(robot_vel);
            //world coordinate to map coordinate to index
            ROS_INFO("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f. Distance is %f", start.pose.position.x, start.pose.position.y,
                     goal.pose.position.x, goal.pose.position.y, hypot(start.pose.position.x-goal.pose.position.x,start.pose.position.y-goal.pose.position.y));
            double wx = start.pose.position.x;
            double wy = start.pose.position.y;
            unsigned int start_x, start_y;
            costmap_->worldToMap(wx, wy, start_x, start_y);
            int start_index = costmap_->getIndex(start_x, start_y);


            wx = goal.pose.position.x;
            wy = goal.pose.position.y;

            unsigned int goal_x, goal_y;
            costmap_->worldToMap(wx, wy, goal_x, goal_y);
            int goal_index = costmap_->getIndex(goal_x, goal_y);
            ///transform dataformat and discretise the angle
            Eigen::Isometry3d startSE3 = getSe3(start);
            Eigen::Isometry3d goalSE3 = getSe3(goal);

            //openlistNcloselist
            multiset <Node> openlist;
            multiset <Node> closelist;


            //gCost[start_index] = 0;
            //Initialize currentNode and put it in openlist
            Node currentNode;
            currentNode.index = start_index;
            currentNode.pose = startSE3;
            currentNode.gCost = 0;
            currentNode.hCost = getHeuristic(currentNode.index,goal_index);
            currentNode.cost=currentNode.gCost+currentNode.hCost;
            //currentNode.v=robot_vel.pose.position.x,currentNode.w=robot_vel.pose.position.y;

            openlist.insert(currentNode);

            plan.clear();

            Node lastnode;
            unsigned int indx;
            map<unsigned int, Node> cl;
            while (!openlist.empty()) {
                // Take the element from the top(begin is the least cost)
                currentNode = *openlist.begin();
                closelist.insert(currentNode);
                cl.insert(pair<unsigned int, Node>(currentNode.index, currentNode));
                //Delete the element from the top
                openlist.erase(openlist.begin());
                currentNode.pose = discretise(currentNode.pose);
                if (getHeuristic(currentNode.index, goal_index) < 7) {
                    ROS_INFO("Got Goal!!!!!!!!!!!!!!!!!!!!!!!!!!");
                    indx = goal_index;
                    lastnode.index = goal_index;
                    lastnode.pose = goalSE3;
                    lastnode.pre = &currentNode;
                    lastnode.camefrom = currentNode.index;
                    //ROS_INFO("index = %d, preindex = %d", lastnode.index, lastnode.pre->index);
                    tf2::Quaternion q;
                    q.setEuler(goalSE3.rotation().eulerAngles(2, 1, 0)[0], 0, 0);
                    geometry_msgs::PoseStamped pose;
                    pose.pose.position.x = goalSE3.translation()[0];
                    pose.pose.position.y = goalSE3.translation()[1];
                    pose.pose.position.z = 0;
                    tf2::convert(q, pose.pose.orientation);
                    lastnode.trajectory.push_back(pose);
                    closelist.insert(lastnode);
                    cl.insert(pair<unsigned int, Node>(lastnode.index, lastnode));
                    break;
                }

                ///update neighbors
                for (int i = 0; i < trajectory.size(); ++i) {
                    Node nextNode;
                    //nextNode.trajectory = trajectory[i];
                    nextNode.pose = currentNode.pose * trajectory[i].back();
                    ROS_INFO("x=%f, y=%f", nextNode.pose.translation()[0], nextNode.pose.translation()[1]);
                    unsigned int x, y;
                    costmap_->worldToMap(nextNode.pose.translation()[0], nextNode.pose.translation()[1], x, y);
                    nextNode.index = costmap_->getIndex(x, y);

                    /*multiset<Node>::iterator it;
                    it = find_if(closelist.begin(), closelist.end(), [nextNode](Node node) {
                        return (node.index == nextNode.index);
                    });
                    if (it != closelist.end()) {
                        ROS_INFO("Already Expanded");
                        continue;
                    }*/
                    multimap<unsigned int, Node>::iterator it;
                    it = cl.find(nextNode.index);
                    if (it != cl.end()) {
                        ROS_INFO("Already Expanded");
                        continue;
                    }
                    if(!OGM[nextNode.index]){
                        ROS_INFO("Obstacle");
                        closelist.insert(nextNode);
                        cl.insert(pair<unsigned int, Node>(nextNode.index, nextNode));

                        continue;
                    }

                    multiset<Node>::iterator itor;
                    itor = find_if(openlist.begin(), openlist.end(), [nextNode](Node node) {
                        return (node.index == nextNode.index);
                    });

                    if (itor != openlist.end()) {
                        //ROS_INFO("Collision!");
                        //double w = 0.26 * ((divide/2) * phase) / (range / 2);
                        //if(getGoalAngleDifference(currentNode.pose,goalSE3)>(M_PI/3));
                        double v = getVP((divide/2) * phase, chordl, range);
                        nextNode.gCost = ((range / 2) / 0.26 * v) / 0.05;
                        nextNode.pre = &currentNode;
                        nextNode.camefrom = currentNode.index;
                        nextNode.hCost = getHeuristic(nextNode.index, goal_index);
                        Node tmp = *itor;
                        if (tmp.getcost() > nextNode.getcost()) {
                            openlist.erase(itor);
                            for (int j = 0; j < trajectory[i].size(); ++j) {
                                Eigen::Isometry3d tmp;
                                tmp = currentNode.pose * trajectory[i][j];
                                tf2::Quaternion q;
                                q.setEuler(tmp.rotation().eulerAngles(2, 1, 0)[0], 0, 0);
                                geometry_msgs::PoseStamped pose;
                                pose.pose.position.x = tmp.translation()[0];
                                pose.pose.position.y = tmp.translation()[1];
                                pose.pose.position.z = 0;
                                tf2::convert(q, pose.pose.orientation);
                                nextNode.trajectory.push_back(pose);
                            }
                            reverse(nextNode.trajectory.begin(), nextNode.trajectory.end());
                            openlist.insert(nextNode);
                            ROS_INFO("Collision! Old Node Replaced");
                            continue;
                        } else {
                            ROS_INFO("Collision! Not Replaced");
                            continue;
                        }
                    }

                    //double w = 0.26 * ((divide/2) * phase) / (range / 2);
                    double v = getVP((divide/2) * phase, chordl, range);
                    nextNode.gCost = ((range / 2) / 0.26 * v) / 0.05;
                    nextNode.pre = &currentNode;
                    nextNode.camefrom = currentNode.index;
                    nextNode.hCost = getHeuristic(nextNode.index, goal_index);
                    //nextNode.hCost = getHeuristic(nextNode.pose, goalSE3, v, w);
                    for (int j = 0; j < trajectory[i].size(); ++j) {
                        Eigen::Isometry3d tmp;
                        tmp = currentNode.pose * trajectory[i][j];
                        tf2::Quaternion q;
                        q.setEuler(tmp.rotation().eulerAngles(2, 1, 0)[0], 0, 0);
                        geometry_msgs::PoseStamped pose;
                        pose.pose.position.x = tmp.translation()[0];
                        pose.pose.position.y = tmp.translation()[1];
                        pose.pose.position.z = 0;
                        tf2::convert(q, pose.pose.orientation);
                        nextNode.trajectory.push_back(pose);
                    }
                    reverse(nextNode.trajectory.begin(), nextNode.trajectory.end());
                    //round(currentNode.pose.translation()[0]);
                    openlist.insert(nextNode);
                    ROS_INFO("New Node Put in Openlist");
                }
            }

/*
                // Get neighbors
                vector<double> movingcost;
                std::vector <std::vector<Eigen::Isometry3d>> trajectory(39,
                                                                        std::vector<Eigen::Isometry3d>(
                                                                                11));
                getTrajectory(currentNode.pose,v_window,w_window,movingcost,trajectory);
                //refine(trajectory,movingcost);


                for (int i = 0; i < trajectory.size(); ++i) {
                    Node nextNode;
                    unsigned int x,y;
                    costmap_->worldToMap(trajectory[i].back().translation()[0],trajectory[i].back().translation()[1],x,y);
                    nextNode.index = costmap_->getIndex(x,y);
                    nextNode.pose = trajectory[i].back();
                    int vnum=floor(i/9.00);
                    int wnum=floor(i%9);
                    nextNode.v = v_window[vnum], nextNode.w = w_window[wnum];
                    double heur = getHeuristic(nextNode.index,goal_index);
                    double cost=gCost[currentNode.index]
                            +movingcost[i]
                            //+0.01*costmap_->getCost(x, y)
                            //+ heur;
                    //ROS_INFO("travelled=%f", fabs(heur-getHeuristic(start_index,goal_index)));
                            +getHeuristic(nextNode.pose,goalSE3,nextNode.v,nextNode.w);
                    if(cost<=gCost[nextNode.index]+heuristic[nextNode.index]){
                        heuristic[nextNode.index] = getHeuristic(nextNode.pose,goalSE3,nextNode.v,nextNode.w);
                        gCost[nextNode.index] = gCost[currentNode.index]
                                                 +movingcost[i];
                        cameFrom[nextNode.index].index= currentNode.index;
                        nextNode.cost = cost;
                        reverse(trajectory[i].begin(), trajectory[i].end());
                        for (int j = 0; j < trajectory[i].size(); ++j) {
                            tf2::Quaternion q;
                            q.setEuler(trajectory[i][j].rotation().eulerAngles(2,1,0)[0],0,0);
                            geometry_msgs::PoseStamped pose;
                            pose.pose.position.x = trajectory[i][j].translation()[0];
                            pose.pose.position.y = trajectory[i][j].translation()[1];
                            pose.pose.position.z = 0;
                            tf2::convert(q,pose.pose.orientation);
                            cameFrom[nextNode.index].trajectory.push_back(pose);
                        }
                        //put expanded node into openlist
                        openlist.insert(nextNode);
                    }else{
                        //ROS_INFO("Denied");
                        continue;
                    }
                }
            }
            */
            //if goal does not come from any node
            if (lastnode.pre == NULL) {
                cout << "Goal not reachable, failed making a global path." << endl;
                return false;
            }

            //Finding the best path, take note of the index
            /*vector<int> bestPath;
            vector<geometry_msgs::PoseStamped> bestTraj;
            currentNode.index = goal_index;
            while (currentNode.index != start_index) {
                bestPath.push_back(cameFrom[currentNode.index].index);
                for (int i = 0; i < cameFrom[currentNode.index].trajectory.size(); ++i) {
                    bestTraj.push_back(cameFrom[currentNode.index].trajectory[i]);
                }
                currentNode.index = cameFrom[currentNode.index].index;
            }*/
            vector<geometry_msgs::PoseStamped> bestTraj;
            map<unsigned int, Node>::iterator it;
            it = cl.find(indx);
            /*multiset<Node>::iterator nodeptr;
            nodeptr = find_if(closelist.begin(), closelist.end(), [indx](Node node) {
                return (node.index == indx);
            });*/
            //nodeptr = &lastnode;
            //ROS_INFO("index = %d, preindex = %d", nodeptr->index, nodeptr->pre->index);
            while(it->second.index!=start_index){
                //getHeuristic(nodeptr->index,start_index);
                /*for (int i = 0; i < nodeptr->trajectory.size(); ++i) {
                    bestTraj.push_back(nodeptr->trajectory[i]);
                }
                nodeptr = nodeptr->pre;
                ROS_INFO("index = %d, preindex = %d", nodeptr->index, nodeptr->pre->index);*/
                for (int i = 0; i < it->second.trajectory.size(); ++i) {
                    bestTraj.push_back(it->second.trajectory[i]);
                }
                it = cl.find(it->second.camefrom);
            }
            bestTraj.push_back(start);
            //bestPath.pop_back();
            //reverse(bestPath.begin(), bestPath.end());
            reverse(bestTraj.begin(), bestTraj.end());

            //for (int i = 0; i < bestTraj.size(); ++i) {
              //  ROS_INFO("from start %f", hypot(bestTraj[i].pose.position.x-start.pose.position.x,bestTraj[i].pose.position.y-start.pose.position.y));
            //}
            ros::Time plan_time = ros::Time::now();
            for (int i = 0; i < bestTraj.size(); i++) {

                bestTraj[i].header.stamp = plan_time;
                bestTraj[i].header.frame_id = costmap_ros_->getGlobalFrameID();

                plan.push_back(bestTraj[i]);
            }
            plan.push_back(goal);
            publishPlan(plan);
            return true;
        }
    }

    bool HybridAstarPlanner::CalcSpline(vector<geometry_msgs::PoseStamped> &bestTraj, vector<geometry_msgs::PoseStamped> &smoothTraj){
        ROS_INFO("Smoothing!!!");
        int order = floor(bestTraj.size()/4.0);
        double dt = 1.0 / order;
        vector<std::pair<double, double>> traj;
        for (int m = 0; m <= order; ++m) {
            pair<double, double> pair;
            pair.first = 0;
            pair.second = 0;
            for (int i = 0; i <= order; ++i) {
                pair.first += CalcTerm(order, bestTraj[4 * i], i, m*dt).first;
                pair.second += CalcTerm(order, bestTraj[4 * i], i, m*dt).second;
            }
            geometry_msgs::PoseStamped pose;
            ros::Time plan_time = ros::Time::now();

            pose.header.stamp = plan_time;
            pose.header.frame_id = costmap_ros_->getGlobalFrameID();
            pose.pose.position.x = pair.first;
            pose.pose.position.y = pair.second;
            pose.pose.position.z = 0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            smoothTraj.push_back(pose);
        }
        return true;
    }

    Eigen::Isometry3d HybridAstarPlanner::getSe3(geometry_msgs::PoseStamped current_pose_) {
        Eigen::Quaterniond q(current_pose_.pose.orientation.w, current_pose_.pose.orientation.x,
                             current_pose_.pose.orientation.y, round(current_pose_.pose.orientation.z/phase)*phase);
        Eigen::Matrix3d mtr = q.toRotationMatrix();
        Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
        T1.rotate(mtr);
        T1.pretranslate(Eigen::Vector3d(current_pose_.pose.position.x, current_pose_.pose.position.y,
                                        current_pose_.pose.position.z));
        return T1;
    }


    Eigen::Isometry3d HybridAstarPlanner::discretise(Eigen::Isometry3d& pose){
        Eigen::Vector3d vec(round(pose.translation()[0]/resolution)*resolution,round(pose.translation()[1]/resolution)*resolution,0);
        Eigen::Isometry3d tmp = Eigen::Isometry3d::Identity();
        tmp.rotate(pose.rotation());
        tmp.pretranslate(vec);
        //pose.translation()<<(round(pose.translation()[0]/resolution)*resolution,round(pose.translation()[1]/resolution)*resolution,0);//round(pose.translation()[2]/resolution)*resolution);
        //ROS_INFO("after discretisation x=%f, y=%f", tmp.translation()[0], tmp.translation()[1]);
        return tmp;
    }

    bool HybridAstarPlanner::trajectoryGen(vector<vector<Eigen::Isometry3d>> &pose){
        vector<double> angle;
        double t = (range/2)/0.26;
        for (int i = 0; i < (divide+1); ++i) {
            angle.push_back(-(range/2/phase)*phase+i*phase);
        }

        for (int i = 0; i < angle.size(); ++i) {
            double w = 0.26*angle[i]/(range/2);
            double v = getVP(angle[i], chordl, range);
            ROS_INFO("Velocity is %f and %f", v, w);
            //cost = v * t;
            double dt = t/20;
            Eigen::Isometry3d tmp = Eigen::Isometry3d::Identity();
            Eigen::AngleAxisd rotation_vector(w * dt, Eigen::Vector3d(0, 0, 1));
            Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
            T.rotate(rotation_vector);
            T.pretranslate(Eigen::Vector3d(v * dt, 0, 0));
            //movingcost[i * w_window.size() + o] = 10*v_window[i] * dt / costmap_->getResolution();

            //tmp is BF trajectory point in SE3
            vector<Eigen::Isometry3d> trajectory;
            //j is trajectory point. iteration are 11 times
            for (int j = 0; j < 20; j++) {
                //intermidiate_gpose=pose*tmp;
                //trajectory[i * w_window.size() + o][j] = intermidiate_gpose;

                tmp = tmp * T;
                trajectory.push_back(tmp);
                ///did not include the start, need to push back the origin
            }
            //ROS_INFO("x=%f,y=%f",trajectory.back().translation()[0],trajectory.back().translation()[1]);
            pose.push_back(trajectory);
        }
        ROS_INFO("Trajectory Successfully Generated");
        return true;
    }



    bool HybridAstarPlanner::getTrajectory(const Eigen::Isometry3d &pose, vector<double>& v_window, vector<double>& w_window, vector<double>& movingcost, vector<vector<Eigen::Isometry3d>>& trajectory){
        double dt = 0.03;
        movingcost.resize(trajectory.size());
        //vector<vector<geometry_msgs::PoseStamped>>::iterator it = trajectory.begin();
        for (int i = 0; i < v_window.size(); i++) {
            Eigen::Isometry3d tmp = Eigen::Isometry3d::Identity();
            for (int o = 0; o < w_window.size(); o++) {
                //breakdistance[i*11+o] = window[i].pose.position.x * window[i].pose.position.x/(2*acc_lim_x);
                //Initialize homogenous matrix T, every T right multiplication stands for a dt
                Eigen::AngleAxisd rotation_vector(w_window[o] * dt, Eigen::Vector3d(0, 0, 1));
                Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
                T.rotate(rotation_vector);
                T.pretranslate(Eigen::Vector3d(v_window[i] * dt, 0, 0));
                movingcost[i * w_window.size() + o] = 10*v_window[i] * dt / resolution;

                //tmp is BF trajectory point in SE3
                Eigen::Isometry3d intermidiate_gpose;
                //j is trajectory point. iteration are 11 times
                for (int j = 0; j < trajectory[0].size(); j++) {
                    intermidiate_gpose=pose*tmp;
                    trajectory[i * w_window.size() + o][j] = intermidiate_gpose;

                    tmp = tmp * T;
                }
            }
        }
        unsigned int x, y;
        vector<double>::iterator itt = movingcost.begin();
        for (vector<vector<Eigen::Isometry3d>>::iterator it = trajectory.begin(); it != trajectory.end(); ) {
            vector<Eigen::Isometry3d> &w = *it;
            costmap_->worldToMap(w.back().translation()[0],w.back().translation()[1],x,y);
            int intermediate_index = costmap_->getIndex(x,y);
            if (!OGM[intermediate_index]){
                it=trajectory.erase(it);
                itt=movingcost.erase(itt);
            }else{
                ++it;
                ++itt;
            }
            //ROS_INFO("%d",static_cast<int>(trajectory.size()));
        }
        return true;
    }
    double HybridAstarPlanner::getVP(double ang, double chordl, double range){
        double angle = fabs(ang);
        if(angle==0){
            return chordl/(range/2/0.26);
        }
        double v;
        v=(chordl*0.26*(angle/(range/2))/2)/ sin(angle/2);
        return v;
    }
    double HybridAstarPlanner::getVN(double ang, double chordl, double range){
        double angle = fabs(ang);
        if(angle==0){
            return chordl/(range/2/0.26);
        }
        double v;
        v=-(chordl*0.26*(angle/(range/2))/2)/ sin(angle/2);
        return v;
    }
    /*double getYaw(Eigen::Isometry3d pose){
        return pose.rotation().eulerAngles(2,1,0)[0];
    }*/
    
    
    bool HybridAstarPlanner::refine(vector<vector<Eigen::Isometry3d>>& trajectory){
        unsigned int x, y, p ,q;
        //vector<double>::iterator itt = movingcost.begin();
        for (vector<vector<Eigen::Isometry3d>>::iterator it = trajectory.begin(); it != trajectory.end(); ) {
            vector<Eigen::Isometry3d> &w = *it;
            costmap_->worldToMap(w.back().translation()[0],w.back().translation()[1],x,y);
            costmap_->worldToMap(w[w.size()/2].translation()[0],w[w.size()/2].translation()[1],p,q);
            int intermediate_index = costmap_->getIndex(x,y);
            int indexx = costmap_->getIndex(p,q);
            if (!OGM[intermediate_index]||!OGM[indexx]){
                ROS_INFO("fuckedup");
                it=trajectory.erase(it);
                //itt=movingcost.erase(itt);
            }else{
                ++it;
                //++itt;
            }
            //ROS_INFO("%d",static_cast<int>(trajectory.size()));
        }
        return true;
    }

    double HybridAstarPlanner::getMoveCost(int firstIndex, int secondIndex)
    {
        unsigned int tmp1, tmp2;
        costmap_->indexToCells(firstIndex, tmp1, tmp2);
        int firstXCord = tmp1,firstYCord = tmp2;
        costmap_->indexToCells(secondIndex, tmp1, tmp2);
        int secondXCord = tmp1, secondYCord = tmp2;

        int difference = abs(firstXCord - secondXCord) + abs(firstYCord - secondYCord);
        // Error checking
        if(difference != 1 && difference != 2){
            ROS_ERROR("Astar global planner: Error in getMoveCost - difference not valid");
            return 1.0;
        }
        if(difference == 1)
            return 1.0;
        else
            return 1.414;
    }

    double HybridAstarPlanner::getHeuristic(int cell_index, int goal_index)
    {
        unsigned int tmp1, tmp2;
        costmap_->indexToCells(cell_index, tmp1, tmp2);
        int startX = tmp1, startY = tmp2;
        costmap_->indexToCells(goal_index, tmp1, tmp2);
        int goalX = tmp1, goalY = tmp2;
        //ROS_INFO("distance is %f",max(abs(goalY - startY) , abs(goalX - startX))-min(abs(goalY - startY) , abs(goalX - startX))+1.414*min(abs(goalY - startY) , abs(goalX - startX)));
        //return max(abs(goalY - startY) , abs(goalX - startX))-min(abs(goalY - startY) , abs(goalX - startX))+1.414*min(abs(goalY - startY) , abs(goalX - startX));//absolute tight
        //if((abs(goalY - startY) + abs(goalX - startX))<20){
            //ROS_INFO("distance is %f",max(abs(goalY - startY) , abs(goalX - startX))-min(abs(goalY - startY) , abs(goalX - startX))+1.414*min(abs(goalY - startY) , abs(goalX - startX)));
        //}
        return max(abs(goalY - startY) , abs(goalX - startX))-min(abs(goalY - startY) , abs(goalX - startX))+1.414*min(abs(goalY - startY) , abs(goalX - startX));//absolute tight
    }

    double HybridAstarPlanner::getHeuristic(const Eigen::Isometry3d& pose,const Eigen::Isometry3d& goal, double v, double w)
    {
        double dt = 0.03;
        double startX = pose.translation()[0], startY = pose.translation()[1];
        double goalX = goal.translation()[0], goalY = goal.translation()[1];
        //deviation
        Eigen::AngleAxisd rotation_vector(w * dt, Eigen::Vector3d(0, 0, 1));
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.rotate(rotation_vector);
        T.pretranslate(Eigen::Vector3d(v * dt, 0, 0));
        Eigen::Isometry3d tmp = T;
        for (int i = 0; i < 9; ++i) {
            tmp= tmp*T;
        }

        double devX = tmp.translation()[0], devY = tmp.translation()[1];

        T = Eigen::Isometry3d::Identity();
        double yaw = pose.rotation().eulerAngles(2,1,0)[0];
        double bian = hypot(startX-goalX,startY-goalY);
        double goal_th;
        if(goalY-startY<0){
            goal_th = -acos((goalX-startX)/bian);
        }else
            goal_th = acos((goalX-startX)/bian);
        double angle= angles::shortest_angular_distance(yaw, goal_th);
        ROS_INFO("Deviation angle is %f", -angle);
        Eigen::AngleAxisd rotation_vector2(angle , Eigen::Vector3d(0, 0, 1));
        T.rotate(rotation_vector2);
        T.translate(Eigen::Vector3d(0.26*10*dt,0,0));
        double X = T.translation()[0], Y = T.translation()[1];
        return (hypot(startX-goalX,startY-goalY)+sqrt((devX-X)*(devX-X)+(devY-Y)*(devY-Y)))/resolution;
    }

    double HybridAstarPlanner::getGoalAngleDifference(const Eigen::Isometry3d& pose,const Eigen::Isometry3d& goal){
        double startX = pose.translation()[0], startY = pose.translation()[1];
        double goalX = goal.translation()[0], goalY = goal.translation()[1];
        double yaw = pose.rotation().eulerAngles(2,1,0)[0];
        double bian = hypot(startX-goalX,startY-goalY);
        double goal_th;
        if(goalY-startY<0){
            goal_th = -acos((goalX-startX)/bian);
        }else
            goal_th = acos((goalX-startX)/bian);
        double angle= fabs(angles::shortest_angular_distance(yaw, goal_th));
        return angle;
    }
    bool HybridAstarPlanner::isInBounds(int x, int y)
    {
        if( x < 0 || y < 0 || x >= height || y >= width)
            return false;
        return true;
    }

    vector<int> HybridAstarPlanner::get_neighbors(int current_cell)
    {
        vector<int> neighborIndexes;

        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                unsigned tmp1, tmp2;
                costmap_->indexToCells(current_cell, tmp1, tmp2);
                int nextX = tmp1 + i;
                int nextY = tmp2 + j;
                int nextIndex = costmap_->getIndex(nextX, nextY);
                //not currentnode & isinbound & no obstacle
                if(!( i == 0 && j == 0) && isInBounds(nextX, nextY) && OGM[nextIndex])
                {
                    neighborIndexes.push_back(nextIndex);
                }
            }
        }
        return neighborIndexes;
    }


    void HybridAstarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
        if (!initialized_) {
            ROS_ERROR(
                    "This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return;
        }

        //create a message for the plan
        nav_msgs::Path gui_path;
        gui_path.poses.resize(path.size());

        gui_path.header.frame_id = frame_id_;
        gui_path.header.stamp = ros::Time::now();

        // Extract the plan in world co-ordinates, we assume the path is all in the same frame
        for (unsigned int i = 0; i < path.size(); i++) {
            gui_path.poses[i] = path[i];
        }

        plan_pub_.publish(gui_path);
    }
}
/*void swap(geometry_msgs::PoseStamped &a, geometry_msgs::PoseStamped &b)
{
    geometry_msgs::PoseStamped tmp;
    tmp.pose.position.x= a.pose.position.x;
    tmp.pose.position.y= a.pose.position.y;
    tmp.pose.position.z= a.pose.position.z;
    tf2::convert(a.pose.orientation,tmp.pose.orientation);
    a.pose.position.x= b.pose.position.x;
    a.pose.position.y= b.pose.position.y;
    a.pose.position.z= b.pose.position.z;
    tf2::convert(b.pose.orientation,a.pose.orientation);
    b.pose.position.x= tmp.pose.position.x;
    b.pose.position.y= tmp.pose.position.y;
    b.pose.position.z= tmp.pose.position.z;
    tf2::convert(tmp.pose.orientation,b.pose.orientation);
}*/

// Required for multiset sorting
bool operator <(const Node& x,const Node& y) {
    return (x.gCost+x.hCost) < (y.gCost+y.hCost);
}