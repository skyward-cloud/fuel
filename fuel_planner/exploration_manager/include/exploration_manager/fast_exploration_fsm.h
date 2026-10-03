#ifndef _FAST_EXPLORATION_FSM_H_
#define _FAST_EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <bspline/msg/bspline.hpp>

#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <thread>

using Eigen::Vector3d;
using std::vector;
using std::shared_ptr;
using std::unique_ptr;
using std::string;

namespace fast_planner {
class FastPlannerManager;
class FastExplorationManager;
class PlanningVisualization;
struct FSMParam;
struct FSMData;

enum EXPL_STATE {
  INIT,
  WAIT_TRIGGER,
  PLAN_TRAJ,
  PUB_TRAJ,
  EXEC_TRAJ,
  PLAN_RETURN,
  FINISH
};

class FastExplorationFSM {
private:
  /* planning utils */
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<FastExplorationManager> expl_manager_;
  shared_ptr<PlanningVisualization> visualization_;

  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  EXPL_STATE state_;

  bool classic_;

  /* ROS utils */
  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr trigger_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr replan_pub_, new_pub_;
  rclcpp::Publisher<bspline::msg::Bspline>::SharedPtr bspline_pub_;

  /* helper functions */
  int callExplorationPlanner();
  int callReturnHomePlanner();
  void fillNewestTraj();
  void prepareStartState();
  void transitState(EXPL_STATE new_state, string pos_call);

  /* ROS functions */
  void FSMCallback();
  void safetyCallback();
  void frontierCallback();
  void triggerCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void visualize();
  void clearVisMarker();

public:
  FastExplorationFSM(/* args */) {
  }
  ~FastExplorationFSM() {
  }

  void init(const rclcpp::Node::SharedPtr& node);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif
