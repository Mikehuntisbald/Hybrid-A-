#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <tf2/utils.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <angles/angles.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <tf2_ros/buffer.h>
#include <move_base/move_base.h>

#include <vector>
#include <queue>
#define infinity 1.0e10
using namespace std;


struct Node{
  double cost;
  double gCost;///
  double hCost;///
  int index;
  Eigen::Isometry3d pose;
  int angle;///
  unsigned int x;///
  unsigned int y;///
  int camefrom;///
  Node* pre;///
  double v;
  double w;
  vector<geometry_msgs::PoseStamped> trajectory;///

  double getcost(){
      return gCost+hCost;
  }
};

struct Camefrom{
    int index;
    vector<geometry_msgs::PoseStamped> trajectory;
};
base_local_planner::OdometryHelperRos odom_helper_;

double getYaw(Eigen::Isometry3d pose);
namespace astar_planner {
    class AstarPlanner : public nav_core::BaseGlobalPlanner{
        public:
            AstarPlanner();
            AstarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            
       /**
       * @brief  Initialization function for the DijstraPlanner
       * @param  name The name of this planner
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose 
       * @param goal The goal pose 
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& start,
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
      unsigned int divide;
      double phase;
      double resolution;
      double movecost;
      double range;
      double chordl;
      int width;
      int height;
      int map_size;
      vector<bool> OGM;
      Eigen::Isometry3d discretise(Eigen::Isometry3d &pose);
      double getVP(double ang, double chordl, double range);
      double getVN(double ang, double chordl, double range);
      bool trajectoryGen(vector<vector<Eigen::Isometry3d>> &pose);
      double getHeuristic(int cell_index, int goal_index);
      double getHeuristic(const Eigen::Isometry3d& pose,const Eigen::Isometry3d& goal, double v, double w);
      vector<int> get_neighbors(int current_cell);
      double getMoveCost(int firstIndex, int secondIndex);

      bool isInBounds(int x, int y);    
      /**
      * @brief  Publish a path for visualization purposes
      */
      void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

      Eigen::Isometry3d getSe3(geometry_msgs::PoseStamped current_pose_);
      double getGoalAngleDifference(const Eigen::Isometry3d& pose,const Eigen::Isometry3d& goal);
      bool getTrajectory(const Eigen::Isometry3d& pose, vector<double>& v_window, vector<double>& w_window, vector<double>& movingcost, vector<vector<Eigen::Isometry3d>>& trajectory);
      bool refine(vector<vector<Eigen::Isometry3d>>& trajectory);
      ros::Publisher plan_pub_;
      std::string frame_id_;
	  bool initialized_;
	  costmap_2d::Costmap2DROS* costmap_ros_;
	  costmap_2d::Costmap2D* costmap_;
    };
};
