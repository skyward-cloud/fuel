#include "bspline/non_uniform_bspline.h"
#include "nav_msgs/msg/odometry.hpp"
#include "bspline/msg/bspline.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "px4_bridge_msgs/msg/realtime_control.hpp"
#include "std_msgs/msg/empty.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <poly_traj/polynomial_traj.h>
#include <active_perception/perception_utils.h>

#include <plan_manage/backward.hpp>
namespace backward {
backward::SignalHandling sh;
}
using fast_planner::NonUniformBspline;
using fast_planner::Polynomial;
using fast_planner::PolynomialTraj;
using fast_planner::PerceptionUtils;

rclcpp::Node::SharedPtr node;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cmd_vis_pub, traj_pub;
rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub;
rclcpp::Publisher<px4_bridge_msgs::msg::RealtimeControl>::SharedPtr rt_cmd_pub;
rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_pub;
nav_msgs::msg::Odometry odom;
quadrotor_msgs::msg::PositionCommand cmd;
px4_bridge_msgs::msg::RealtimeControl rt_cmd;

// Info of generated traj
vector<NonUniformBspline> traj_;
double traj_duration_;
rclcpp::Time start_time_;
int traj_id_;
int pub_traj_id_;

shared_ptr<PerceptionUtils> percep_utils_;

// Info of replan
bool receive_traj_ = false;
double replan_time_;

// Executed traj, commanded and real ones
vector<Eigen::Vector3d> traj_cmd_, traj_real_;

// Data for benchmark comparison
rclcpp::Time start_time, end_time, last_time;
double energy;

// Loop correction
Eigen::Matrix3d R_loop;
Eigen::Vector3d T_loop;
bool isLoopCorrection = false;

// 与 super FsmRos2::pubCmdTimerCallback 相同：跟轨迹时发布平坦输出。
void publishRealtimeControl(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel,
                                     const Eigen::Vector3d& acc, const Eigen::Vector3d& jer,
                                     double yaw, double yawdot) {
  if (!rt_cmd_pub) return;
  rt_cmd.enable = true;
  rt_cmd.has_command = true;
  rt_cmd.use_position = true;
  rt_cmd.use_velocity = true;
  rt_cmd.use_acceleration = true;
  rt_cmd.use_jerk = true;
  rt_cmd.use_yaw_rate = true;
  rt_cmd.x = static_cast<float>(pos(0));
  rt_cmd.y = static_cast<float>(pos(1));
  rt_cmd.z = static_cast<float>(pos(2));
  rt_cmd.vx = static_cast<float>(vel(0));
  rt_cmd.vy = static_cast<float>(vel(1));
  rt_cmd.vz = static_cast<float>(vel(2));
  rt_cmd.ax = static_cast<float>(acc(0));
  rt_cmd.ay = static_cast<float>(acc(1));
  rt_cmd.az = static_cast<float>(acc(2));
  // rt_cmd.jerk_x = static_cast<float>(jer(0));
  // rt_cmd.jerk_y = static_cast<float>(jer(1));
  // rt_cmd.jerk_z = static_cast<float>(jer(2));
  rt_cmd.yaw = static_cast<float>(yaw);
  rt_cmd.yaw_rate = static_cast<float>(yawdot);
  rt_cmd_pub->publish(rt_cmd);
}

void heartbeatTimerCallback() {
  if (!heartbeat_pub) return;
  heartbeat_pub->publish(std_msgs::msg::Empty());
}

double calcPathLength(const vector<Eigen::Vector3d>& path) {
  if (path.empty()) return 0;
  double len = 0.0;
  for (int i = 0; i < path.size() - 1; ++i) {
    len += (path[i + 1] - path[i]).norm();
  }
  return len;
}

void displayTrajWithColor(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color,
                          int id) {
  visualization_msgs::msg::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = node->now();
  mk.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::msg::Marker::DELETE;
  mk.id = id;
  traj_pub->publish(mk);

  mk.action = visualization_msgs::msg::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;
  geometry_msgs::msg::Point pt;
  for (int i = 0; i < int(path.size()); i++) {
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    mk.points.push_back(pt);
  }
  traj_pub->publish(mk);
  rclcpp::sleep_for(std::chrono::milliseconds(1));
}

void drawFOV(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2) {
  visualization_msgs::msg::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = node->now();
  mk.id = 0;
  mk.ns = "current_pose";
  mk.type = visualization_msgs::msg::Marker::LINE_LIST;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.color.a = 1.0;
  mk.scale.x = 0.04;
  mk.scale.y = 0.04;
  mk.scale.z = 0.04;

  // Clean old marker
  mk.action = visualization_msgs::msg::Marker::DELETE;
  cmd_vis_pub->publish(mk);

  if (list1.size() == 0) return;

  // Pub new marker
  geometry_msgs::msg::Point pt;
  for (int i = 0; i < int(list1.size()); ++i) {
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);

    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);
  }
  mk.action = visualization_msgs::msg::Marker::ADD;
  cmd_vis_pub->publish(mk);
}

void drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id,
             const Eigen::Vector4d& color) {
  visualization_msgs::msg::Marker mk_state;
  mk_state.header.frame_id = "world";
  mk_state.header.stamp = node->now();
  mk_state.id = id;
  mk_state.type = visualization_msgs::msg::Marker::ARROW;
  mk_state.action = visualization_msgs::msg::Marker::ADD;

  mk_state.pose.orientation.w = 1.0;
  mk_state.scale.x = 0.1;
  mk_state.scale.y = 0.2;
  mk_state.scale.z = 0.3;

  geometry_msgs::msg::Point pt;
  pt.x = pos(0);
  pt.y = pos(1);
  pt.z = pos(2);
  mk_state.points.push_back(pt);

  pt.x = pos(0) + vec(0);
  pt.y = pos(1) + vec(1);
  pt.z = pos(2) + vec(2);
  mk_state.points.push_back(pt);

  mk_state.color.r = color(0);
  mk_state.color.g = color(1);
  mk_state.color.b = color(2);
  mk_state.color.a = color(3);

  cmd_vis_pub->publish(mk_state);
}

void replanCallback(const std_msgs::msg::Empty::ConstSharedPtr& msg) {
  // Informed of new replan, end the current traj after some time
  // No traj yet: start_time_ may still be default SYSTEM_TIME and cannot
  // be subtracted from node->now() (ROS_TIME).
  if (!receive_traj_) return;
  const double time_out = 0.3;
  rclcpp::Time time_now = node->now();
  double t_stop = (time_now - start_time_).seconds() + time_out + replan_time_;
  traj_duration_ = min(t_stop, traj_duration_);
}

void newCallback(const std_msgs::msg::Empty::ConstSharedPtr& msg) {
  // Clear the executed traj data
  traj_cmd_.clear();
  traj_real_.clear();
}

void odomCallbck(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  if (msg->child_frame_id == "X" || msg->child_frame_id == "O") return;
  odom = *msg;
  traj_real_.push_back(
      Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

  if (traj_real_.size() > 10000) traj_real_.erase(traj_real_.begin(), traj_real_.begin() + 1000);
}

void pgTVioCallback(const geometry_msgs::msg::Pose::ConstSharedPtr& msg) {
  // World to odom
  Eigen::Quaterniond q =
      Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  R_loop = q.toRotationMatrix();
  T_loop << msg->position.x, msg->position.y, msg->position.z;

  // cout << "R_loop: " << R_loop << endl;
  // cout << "T_loop: " << T_loop << endl;
}

void visCallback() {
  // Draw the executed traj (desired state)
  // displayTrajWithColor(traj_cmd_, 0.05, Eigen::Vector4d(1, 0, 0, 1), pub_traj_id_);
  // displayTrajWithColor(traj_cmd_, 0.05, Eigen::Vector4d(0, 1, 0, 1), pub_traj_id_);
  displayTrajWithColor(traj_cmd_, 0.05, Eigen::Vector4d(0, 0, 1, 1), pub_traj_id_);

  // displayTrajWithColor(traj_real_, 0.03, Eigen::Vector4d(0.925, 0.054, 0.964,
  // 1),
  //                      1);
}

void bsplineCallback(const bspline::msg::Bspline::ConstSharedPtr& msg) {
  // Received traj should have ascending traj_id
  if (msg->traj_id <= traj_id_) {
    RCLCPP_ERROR(node->get_logger(), "out of order bspline.");
    return;
  }

  // Parse the msg
  Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);
  Eigen::VectorXd knots(msg->knots.size());
  for (int i = 0; i < msg->knots.size(); ++i) {
    knots(i) = msg->knots[i];
  }
  for (int i = 0; i < msg->pos_pts.size(); ++i) {
    pos_pts(i, 0) = msg->pos_pts[i].x;
    pos_pts(i, 1) = msg->pos_pts[i].y;
    pos_pts(i, 2) = msg->pos_pts[i].z;
  }
  NonUniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  for (int i = 0; i < msg->yaw_pts.size(); ++i)
    yaw_pts(i, 0) = msg->yaw_pts[i];
  NonUniformBspline yaw_traj(yaw_pts, 3, msg->yaw_dt);
  start_time_ = rclcpp::Time(msg->start_time.sec, msg->start_time.nanosec, node->get_clock()->get_clock_type());
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());
  traj_.push_back(yaw_traj);
  traj_.push_back(yaw_traj.getDerivative());
  traj_.push_back(traj_[2].getDerivative());
  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;

  // Record the start time of flight
  if (start_time.nanoseconds() == 0) {
    RCLCPP_WARN(node->get_logger(), "start flight");
    start_time = node->now();
  }
}

void cmdCallback() {
  // No publishing before receive traj data
  if (!receive_traj_) return;

  rclcpp::Time time_now = node->now();
  double t_cur = (time_now - start_time_).seconds();
  Eigen::Vector3d pos, vel, acc, jer = Eigen::Vector3d::Zero();
  double yaw = 0.0, yawdot = 0.0;

  if (t_cur < traj_duration_ && t_cur >= 0.0) {
    // Current time within range of planned traj
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);
    yaw = traj_[3].evaluateDeBoorT(t_cur)[0];
    yawdot = traj_[4].evaluateDeBoorT(t_cur)[0];
    jer = traj_[5].evaluateDeBoorT(t_cur);
  } else if (t_cur >= traj_duration_) {
    // Current time exceed range of planned traj
    // keep publishing the final position and yaw
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();
    yaw = traj_[3].evaluateDeBoorT(traj_duration_)[0];
    yawdot = 0.0;

    // Report info of the whole flight
    double len = calcPathLength(traj_cmd_);
    double flight_t = (end_time - start_time).seconds();
    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                         "flight time: %lf, path length: %lf, mean vel: %lf, energy is: % lf ", flight_t,
                         len, len / flight_t, energy);
  } else {
    cout << "[Traj server]: invalid time." << endl;
  }

  if (isLoopCorrection) {
    pos = R_loop.transpose() * (pos - T_loop);
    vel = R_loop.transpose() * vel;
    acc = R_loop.transpose() * acc;

    Eigen::Vector3d yaw_dir(cos(yaw), sin(yaw), 0);
    yaw_dir = R_loop.transpose() * yaw_dir;
    yaw = atan2(yaw_dir[1], yaw_dir[0]);
    jer = R_loop.transpose() * jer;
  }

  cmd.header.stamp = time_now;
  cmd.trajectory_id = traj_id_;
  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);
  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);
  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);
  cmd.yaw = yaw;
  cmd.yaw_dot = yawdot;
  pos_cmd_pub->publish(cmd);
  publishRealtimeControl(pos, vel, acc, jer, yaw, yawdot);

  // Draw cmd
  // Eigen::Vector3d dir(cos(yaw), sin(yaw), 0.0);
  // drawCmd(pos, 2 * dir, 2, Eigen::Vector4d(1, 1, 0, 0.7));
  // drawCmd(pos, vel, 0, Eigen::Vector4d(0, 1, 0, 1));
  // drawCmd(pos, acc, 1, Eigen::Vector4d(0, 0, 1, 1));
  // drawCmd(pos, pos_err, 3, Eigen::Vector4d(1, 1, 0, 0.7));
  percep_utils_->setPose(pos, yaw);
  vector<Eigen::Vector3d> l1, l2;
  percep_utils_->getFOV(l1, l2);
  drawFOV(l1, l2);

  // Record info of the executed traj
  if (traj_cmd_.size() == 0) {
    // Add the first position
    traj_cmd_.push_back(pos);
  } else if ((pos - traj_cmd_.back()).norm() > 1e-6) {
    // Add new different commanded position
    traj_cmd_.push_back(pos);
    if (last_time.nanoseconds() > 0) {
      double dt = (time_now - last_time).seconds();
      energy += jer.squaredNorm() * dt;
    }
    end_time = node->now();
  }
  last_time = time_now;

  // if (traj_cmd_.size() > 100000)
  //   traj_cmd_.erase(traj_cmd_.begin(), traj_cmd_.begin() + 1000);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  node = std::make_shared<rclcpp::Node>("traj_server");

  // Initialize times with the same clock type as node->now() (RCL_ROS_TIME)
  const auto clock_type = node->get_clock()->get_clock_type();
  start_time_ = rclcpp::Time(0, 0, clock_type);
  start_time = rclcpp::Time(0, 0, clock_type);
  end_time = rclcpp::Time(0, 0, clock_type);
  last_time = rclcpp::Time(0, 0, clock_type);

  auto bspline_sub = node->create_subscription<bspline::msg::Bspline>(
      "planning/bspline", 10, bsplineCallback);
  auto replan_sub = node->create_subscription<std_msgs::msg::Empty>(
      "planning/replan", 10, replanCallback);
  auto new_sub = node->create_subscription<std_msgs::msg::Empty>(
      "planning/new", 10, newCallback);
  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
      "/odom_world", 50, odomCallbck);
  auto pg_T_vio_sub = node->create_subscription<geometry_msgs::msg::Pose>(
      "/loop_fusion/pg_T_vio", 10, pgTVioCallback);

  cmd_vis_pub = node->create_publisher<visualization_msgs::msg::Marker>("planning/position_cmd_vis", 10);
  pos_cmd_pub = node->create_publisher<quadrotor_msgs::msg::PositionCommand>("/position_cmd", 50);
  rt_cmd_pub = node->create_publisher<px4_bridge_msgs::msg::RealtimeControl>(
      "/px4_bridge/in/realtime_control", 10);
  const rclcpp::QoS heartbeat_qos(rclcpp::QoS(1).best_effort().keep_last(1).durability_volatile());
  heartbeat_pub = node->create_publisher<std_msgs::msg::Empty>("/super_planner/heartbeat", heartbeat_qos);
  traj_pub = node->create_publisher<visualization_msgs::msg::Marker>("planning/travel_traj", 10);

  auto cmd_timer = node->create_wall_timer(std::chrono::milliseconds(10), cmdCallback);
  auto vis_timer = node->create_wall_timer(std::chrono::milliseconds(250), visCallback);
  auto heartbeat_timer = node->create_wall_timer(std::chrono::milliseconds(1000), heartbeatTimerCallback);

  if (!node->has_parameter("traj_server/pub_traj_id")) {

    node->declare_parameter("traj_server/pub_traj_id", -1);

  }
  node->get_parameter("traj_server/pub_traj_id", pub_traj_id_);
  if (!node->has_parameter("fsm/replan_time")) {
    node->declare_parameter("fsm/replan_time", 0.1);
  }
  node->get_parameter("fsm/replan_time", replan_time_);
  if (!node->has_parameter("loop_correction/isLoopCorrection")) {
    node->declare_parameter("loop_correction/isLoopCorrection", false);
  }
  node->get_parameter("loop_correction/isLoopCorrection", isLoopCorrection);

  Eigen::Vector3d init_pos;
  if (!node->has_parameter("traj_server/init_x")) {
    node->declare_parameter("traj_server/init_x", 0.0);
  }
  node->get_parameter("traj_server/init_x", init_pos[0]);
  if (!node->has_parameter("traj_server/init_y")) {
    node->declare_parameter("traj_server/init_y", 0.0);
  }
  node->get_parameter("traj_server/init_y", init_pos[1]);
  if (!node->has_parameter("traj_server/init_z")) {
    node->declare_parameter("traj_server/init_z", 0.0);
  }
  node->get_parameter("traj_server/init_z", init_pos[2]);

  RCLCPP_WARN(node->get_logger(), "[Traj server]: init...");
  rclcpp::sleep_for(std::chrono::seconds(1));

  // Control parameter
  cmd.kx = { 5.7, 5.7, 6.2 };
  cmd.kv = { 3.4, 3.4, 4.0 };

  std::cout << start_time.seconds() << std::endl;
  std::cout << end_time.seconds() << std::endl;

  cmd.header.stamp = node->now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;
  cmd.position.x = init_pos[0];
  cmd.position.y = init_pos[1];
  cmd.position.z = init_pos[2];
  cmd.velocity.x = 0.0;
  cmd.velocity.y = 0.0;
  cmd.velocity.z = 0.0;
  cmd.acceleration.x = 0.0;
  cmd.acceleration.y = 0.0;
  cmd.acceleration.z = 0.0;
  cmd.yaw = 0.0;
  cmd.yaw_dot = 0.0;

  percep_utils_.reset(new PerceptionUtils(node));

  // Initialization for exploration, move upward and downward
  for (int i = 0; i < 100; ++i) {
    cmd.position.z += 0.01;
    pos_cmd_pub->publish(cmd);
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  for (int i = 0; i < 100; ++i) {
    cmd.position.z -= 0.01;
    pos_cmd_pub->publish(cmd);
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  // rclcpp::sleep_for(std::chrono::seconds(1));
  // for (int i = 0; i < 100; ++i)
  // {
  //   cmd.position.x -= 0.01;
  //   pos_cmd_pub->publish(cmd);
  //   rclcpp::sleep_for(std::chrono::milliseconds(10));
  // }

  R_loop = Eigen::Quaterniond(1, 0, 0, 0).toRotationMatrix();
  T_loop = Eigen::Vector3d(0, 0, 0);

  RCLCPP_WARN(node->get_logger(), "[Traj server]: ready.");
  rclcpp::spin(node);

  return 0;
}
