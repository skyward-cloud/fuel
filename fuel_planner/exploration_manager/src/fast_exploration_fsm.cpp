
#include <plan_manage/planner_manager.h>
#include <exploration_manager/fast_exploration_manager.h>
#include <traj_utils/planning_visualization.h>

#include <exploration_manager/fast_exploration_fsm.h>
#include <exploration_manager/expl_data.h>
#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>

using Eigen::Vector4d;

namespace fast_planner {
void FastExplorationFSM::init(const rclcpp::Node::SharedPtr& node) {
  node_ = node;
  fp_.reset(new FSMParam);
  fd_.reset(new FSMData);

  /*  Fsm param  */
  if (!node->has_parameter("fsm/thresh_replan1")) {
    node->declare_parameter("fsm/thresh_replan1", -1.0);
  }
  node->get_parameter("fsm/thresh_replan1", fp_->replan_thresh1_);
  if (!node->has_parameter("fsm/thresh_replan2")) {
    node->declare_parameter("fsm/thresh_replan2", -1.0);
  }
  node->get_parameter("fsm/thresh_replan2", fp_->replan_thresh2_);
  if (!node->has_parameter("fsm/thresh_replan3")) {
    node->declare_parameter("fsm/thresh_replan3", -1.0);
  }
  node->get_parameter("fsm/thresh_replan3", fp_->replan_thresh3_);
  if (!node->has_parameter("fsm/replan_time")) {
    node->declare_parameter("fsm/replan_time", -1.0);
  }
  node->get_parameter("fsm/replan_time", fp_->replan_time_);
  if (!node->has_parameter("fsm/return_home")) {
    node->declare_parameter("fsm/return_home", false);
  }
  node->get_parameter("fsm/return_home", fp_->return_home_);
  if (!node->has_parameter("fsm/return_home_thresh")) {
    node->declare_parameter("fsm/return_home_thresh", 0.5);
  }
  node->get_parameter("fsm/return_home_thresh", fp_->return_home_thresh_);

  /* Initialize main modules */
  expl_manager_.reset(new FastExplorationManager);
  expl_manager_->initialize(node);
  visualization_.reset(new PlanningVisualization(node));

  planner_manager_ = expl_manager_->planner_manager_;
  state_ = EXPL_STATE::INIT;
  fd_->have_odom_ = false;
  fd_->state_str_ = { "INIT", "WAIT_TRIGGER", "PLAN_TRAJ", "PUB_TRAJ", "EXEC_TRAJ", "PLAN_RETURN",
                      "FINISH" };
  fd_->static_state_ = true;
  fd_->trigger_ = false;
  fd_->returning_home_ = false;

  /* Ros sub, pub and timer */
  exec_timer_ = node->create_wall_timer(std::chrono::duration<double>(0.01),
                                        std::bind(&FastExplorationFSM::FSMCallback, this));
  safety_timer_ = node->create_wall_timer(std::chrono::duration<double>(0.05),
                                          std::bind(&FastExplorationFSM::safetyCallback, this));
  frontier_timer_ = node->create_wall_timer(std::chrono::duration<double>(0.5),
                                            std::bind(&FastExplorationFSM::frontierCallback, this));

  trigger_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 1,
      std::bind(&FastExplorationFSM::triggerCallback, this, std::placeholders::_1));
  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
      "/odom_world", 1,
      std::bind(&FastExplorationFSM::odometryCallback, this, std::placeholders::_1));

  // replan_pub_ = node->create_publisher<std_msgs::msg::Empty>("/planning/replan", 10);
  // new_pub_ = node->create_publisher<std_msgs::msg::Empty>("/planning/new", 10);
  bspline_pub_ = node->create_publisher<bspline::msg::Bspline>("/planning/bspline", 10);

  RCLCPP_INFO(node_->get_logger(), "FSM return_home: %s, thresh: %.2f",
              fp_->return_home_ ? "true" : "false", fp_->return_home_thresh_);
}

void FastExplorationFSM::prepareStartState() {
  if (fd_->static_state_) {
    fd_->start_pt_ = fd_->odom_pos_;
    fd_->start_vel_ = fd_->odom_vel_;
    fd_->start_acc_.setZero();

    fd_->start_yaw_(0) = fd_->odom_yaw_;
    fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;
  } else {
    LocalTrajData* info = &planner_manager_->local_data_;
    double t_r = (node_->now() - info->start_time_).seconds() + fp_->replan_time_;

    fd_->start_pt_ = info->position_traj_.evaluateDeBoorT(t_r);
    fd_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
    fd_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_r);
    fd_->start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
    fd_->start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_r)[0];
    fd_->start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
  }
}

void FastExplorationFSM::fillNewestTraj() {
  auto info = &planner_manager_->local_data_;
  rclcpp::Time time_r = node_->now() + rclcpp::Duration::from_seconds(fp_->replan_time_);
  info->start_time_ = (node_->now() - time_r).seconds() > 0 ? node_->now() : time_r;

  bspline::msg::Bspline bspline;
  bspline.order = planner_manager_->pp_.bspline_degree_;
  bspline.start_time = info->start_time_;
  bspline.traj_id = info->traj_id_;
  Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
  for (int i = 0; i < pos_pts.rows(); ++i) {
    geometry_msgs::msg::Point pt;
    pt.x = pos_pts(i, 0);
    pt.y = pos_pts(i, 1);
    pt.z = pos_pts(i, 2);
    bspline.pos_pts.push_back(pt);
  }
  Eigen::VectorXd knots = info->position_traj_.getKnot();
  for (int i = 0; i < knots.rows(); ++i) {
    bspline.knots.push_back(knots(i));
  }
  Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
  for (int i = 0; i < yaw_pts.rows(); ++i) {
    double yaw = yaw_pts(i, 0);
    bspline.yaw_pts.push_back(yaw);
  }
  bspline.yaw_dt = info->yaw_traj_.getKnotSpan();
  fd_->newest_traj_ = bspline;
}

void FastExplorationFSM::FSMCallback() {
  RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                              "[FSM]: state: " << fd_->state_str_[int(state_)]);

  switch (state_) {
    case INIT: {
      // Wait for odometry ready
      if (!fd_->have_odom_) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "no odom.");
        return;
      }
      // Go to wait trigger when odom is ok
      transitState(WAIT_TRIGGER, "FSM");
      break;
    }

    case WAIT_TRIGGER: {
      // Do nothing but wait for trigger
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "wait for trigger.");
      break;
    }

    case FINISH: {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "finish exploration.");
      break;
    }

    case PLAN_TRAJ: {
      prepareStartState();

      // Inform traj_server the replanning
      // replan_pub_->publish(std_msgs::msg::Empty());
      int res = callExplorationPlanner();
      if (res == SUCCEED) {
        transitState(PUB_TRAJ, "FSM");
      } else if (res == NO_FRONTIER) {
        fd_->static_state_ = true;
        clearVisMarker();
        if (fp_->return_home_) {
          fd_->returning_home_ = true;
          RCLCPP_INFO(node_->get_logger(), "No frontier, returning home.");
          transitState(PLAN_RETURN, "FSM");
        } else {
          transitState(FINISH, "FSM");
        }
      } else if (res == FAIL) {
        // Still in PLAN_TRAJ state, keep replanning
        RCLCPP_WARN(node_->get_logger(), "plan fail");
        fd_->static_state_ = true;
      }
      break;
    }

    case PLAN_RETURN: {
      if ((fd_->odom_pos_ - fd_->home_pos_).norm() < fp_->return_home_thresh_ &&
          fd_->odom_vel_.norm() < 0.2) {
        RCLCPP_INFO(node_->get_logger(), "Arrived home.");
        fd_->returning_home_ = false;
        fd_->static_state_ = true;
        transitState(FINISH, "FSM");
        break;
      }

      prepareStartState();
      int res = callReturnHomePlanner();
      if (res == SUCCEED) {
        transitState(PUB_TRAJ, "FSM");
      } else {
        RCLCPP_WARN(node_->get_logger(), "return home plan fail, retrying.");
        fd_->static_state_ = true;
      }
      break;
    }

    case PUB_TRAJ: {
      double dt = (node_->now() -
                   rclcpp::Time(fd_->newest_traj_.start_time.sec, fd_->newest_traj_.start_time.nanosec,
                                node_->get_clock()->get_clock_type()))
                      .seconds();
      if (dt > 0) {
        bspline_pub_->publish(fd_->newest_traj_);
        fd_->static_state_ = false;
        transitState(EXEC_TRAJ, "FSM");

        thread vis_thread(&FastExplorationFSM::visualize, this);
        vis_thread.detach();
      }
      break;
    }

    case EXEC_TRAJ: {
      LocalTrajData* info = &planner_manager_->local_data_;
      double t_cur = (node_->now() - info->start_time_).seconds();
      double time_to_end = info->duration_ - t_cur;

      if (fd_->returning_home_) {
        if ((fd_->odom_pos_ - fd_->home_pos_).norm() < fp_->return_home_thresh_ &&
            fd_->odom_vel_.norm() < 0.2) {
          RCLCPP_INFO(node_->get_logger(), "Arrived home.");
          fd_->returning_home_ = false;
          fd_->static_state_ = true;
          transitState(FINISH, "FSM");
          return;
        }
        if (time_to_end < fp_->replan_thresh1_) {
          transitState(PLAN_RETURN, "FSM");
          RCLCPP_WARN(node_->get_logger(), "Return replan: traj fully executed");
          return;
        }
        if (t_cur > fp_->replan_thresh3_) {
          transitState(PLAN_RETURN, "FSM");
          RCLCPP_WARN(node_->get_logger(), "Return replan: periodic call");
        }
        break;
      }

      // Replan if traj is almost fully executed
      if (time_to_end < fp_->replan_thresh1_) {
        transitState(PLAN_TRAJ, "FSM");
        RCLCPP_WARN(node_->get_logger(), "Replan: traj fully executed=================================");
        return;
      }
      // Replan if next frontier to be visited is covered
      if (t_cur > fp_->replan_thresh2_ && expl_manager_->frontier_finder_->isFrontierCovered()) {
        transitState(PLAN_TRAJ, "FSM");
        RCLCPP_WARN(node_->get_logger(), "Replan: cluster covered=====================================");
        return;
      }
      // Replan after some time
      if (t_cur > fp_->replan_thresh3_ && !classic_) {
        transitState(PLAN_TRAJ, "FSM");
        RCLCPP_WARN(node_->get_logger(), "Replan: periodic call=======================================");
      }
      break;
    }
  }
}

int FastExplorationFSM::callExplorationPlanner() {
  int res = expl_manager_->planExploreMotion(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,
                                             fd_->start_yaw_);
  classic_ = false;

  // int res = expl_manager_->classicFrontier(fd_->start_pt_, fd_->start_yaw_[0]);
  // classic_ = true;

  // int res = expl_manager_->rapidFrontier(fd_->start_pt_, fd_->start_vel_, fd_->start_yaw_[0],
  // classic_);

  if (res == SUCCEED) {
    fillNewestTraj();
  }
  return res;
}

int FastExplorationFSM::callReturnHomePlanner() {
  int res = expl_manager_->planReturnHome(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,
                                          fd_->start_yaw_, fd_->home_pos_, fd_->home_yaw_);
  if (res == SUCCEED) {
    fillNewestTraj();
  }
  return res;
}

void FastExplorationFSM::visualize() {
  auto info = &planner_manager_->local_data_;
  auto plan_data = &planner_manager_->plan_data_;
  auto ed_ptr = expl_manager_->ed_;

  // Draw updated box
  // Vector3d bmin, bmax;
  // planner_manager_->edt_environment_->sdf_map_->getUpdatedBox(bmin, bmax);
  // visualization_->drawBox((bmin + bmax) / 2.0, bmax - bmin, Vector4d(0, 1, 0, 0.3), "updated_box", 0,
  // 4);

  // Draw frontier
  static int last_ftr_num = 0;
  for (int i = 0; i < ed_ptr->frontiers_.size(); ++i) {
    visualization_->drawCubes(ed_ptr->frontiers_[i], 0.1,
                              visualization_->getColor(double(i) / ed_ptr->frontiers_.size(), 0.4),
                              "frontier", i, 4);
    // visualization_->drawBox(ed_ptr->frontier_boxes_[i].first, ed_ptr->frontier_boxes_[i].second,
    //                         Vector4d(0.5, 0, 1, 0.3), "frontier_boxes", i, 4);
  }
  for (int i = ed_ptr->frontiers_.size(); i < last_ftr_num; ++i) {
    visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
    // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
    // "frontier_boxes", i, 4);
  }
  last_ftr_num = ed_ptr->frontiers_.size();
  // for (int i = 0; i < ed_ptr->dead_frontiers_.size(); ++i)
  //   visualization_->drawCubes(ed_ptr->dead_frontiers_[i], 0.1, Vector4d(0, 0, 0, 0.5), "dead_frontier",
  //                             i, 4);
  // for (int i = ed_ptr->dead_frontiers_.size(); i < 5; ++i)
  //   visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 0.5), "dead_frontier", i, 4);

  // Draw global top viewpoints info
  // visualization_->drawSpheres(ed_ptr->points_, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
  // visualization_->drawLines(ed_ptr->global_tour_, 0.07, Vector4d(0, 0.5, 0, 1), "global_tour", 0, 6);
  // visualization_->drawLines(ed_ptr->points_, ed_ptr->views_, 0.05, Vector4d(0, 1, 0.5, 1), "view", 0, 6);
  // visualization_->drawLines(ed_ptr->points_, ed_ptr->averages_, 0.03, Vector4d(1, 0, 0, 1),
  // "point-average", 0, 6);

  // Draw local refined viewpoints info
  // visualization_->drawSpheres(ed_ptr->refined_points_, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
  // visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->refined_views_, 0.05,
  //                           Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
  // visualization_->drawLines(ed_ptr->refined_tour_, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
  // visualization_->drawLines(ed_ptr->refined_views1_, ed_ptr->refined_views2_, 0.04, Vector4d(0, 0, 0,
  // 1),
  //                           "refined_view", 0, 6);
  // visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->unrefined_points_, 0.05, Vector4d(1, 1,
  // 0, 1),
  //                           "refine_pair", 0, 6);
  // for (int i = 0; i < ed_ptr->n_points_.size(); ++i)
  //   visualization_->drawSpheres(ed_ptr->n_points_[i], 0.1,
  //                               visualization_->getColor(double(ed_ptr->refined_ids_[i]) /
  //                               ed_ptr->frontiers_.size()),
  //                               "n_points", i, 6);
  // for (int i = ed_ptr->n_points_.size(); i < 15; ++i)
  //   visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 0, 1), "n_points", i, 6);

  // Draw trajectory
  // visualization_->drawSpheres({ ed_ptr->next_goal_ }, 0.3, Vector4d(0, 1, 1, 1), "next_goal", 0, 6);
  visualization_->drawBspline(info->position_traj_, 0.1, Vector4d(1.0, 0.0, 0.0, 1), false, 0.15,
                              Vector4d(1, 1, 0, 1));
  // visualization_->drawSpheres(plan_data->kino_path_, 0.1, Vector4d(1, 0, 1, 1), "kino_path", 0, 0);
  // visualization_->drawLines(ed_ptr->path_next_goal_, 0.05, Vector4d(0, 1, 1, 1), "next_goal", 1, 6);
}

void FastExplorationFSM::clearVisMarker() {
  // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
  // visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "global_tour", 0, 6);
  // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
  // visualization_->drawLines({}, {}, 0.05, Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
  // visualization_->drawLines({}, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
  // visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 1, 1), "B-Spline", 0, 0);

  // visualization_->drawLines({}, {}, 0.03, Vector4d(1, 0, 0, 1), "current_pose", 0, 6);
}

void FastExplorationFSM::frontierCallback() {
  static int delay = 0;
  if (++delay < 5) return;

  if (state_ == WAIT_TRIGGER || state_ == FINISH) {
    auto ft = expl_manager_->frontier_finder_;
    auto ed = expl_manager_->ed_;
    ft->searchFrontiers();
    ft->computeFrontiersToVisit();
    ft->updateFrontierCostMatrix();

    ft->getFrontiers(ed->frontiers_);
    ft->getFrontierBoxes(ed->frontier_boxes_);

    // Draw frontier and bounding box
    for (int i = 0; i < ed->frontiers_.size(); ++i) {
      visualization_->drawCubes(ed->frontiers_[i], 0.1,
                                visualization_->getColor(double(i) / ed->frontiers_.size(), 0.4),
                                "frontier", i, 4);
      // visualization_->drawBox(ed->frontier_boxes_[i].first, ed->frontier_boxes_[i].second,
      // Vector4d(0.5, 0, 1, 0.3),
      //                         "frontier_boxes", i, 4);
    }
    for (int i = ed->frontiers_.size(); i < 50; ++i) {
      visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
      // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
      // "frontier_boxes", i, 4);
    }
  }

  // if (!fd_->static_state_)
  // {
  //   static double astar_time = 0.0;
  //   static int astar_num = 0;
  //   auto t1 = node_->now();

  //   planner_manager_->path_finder_->reset();
  //   planner_manager_->path_finder_->setResolution(0.4);
  //   if (planner_manager_->path_finder_->search(fd_->odom_pos_, Vector3d(-5, 0, 1)))
  //   {
  //     auto path = planner_manager_->path_finder_->getPath();
  //     visualization_->drawLines(path, 0.05, Vector4d(1, 0, 0, 1), "astar", 0, 6);
  //     auto visit = planner_manager_->path_finder_->getVisited();
  //     visualization_->drawCubes(visit, 0.3, Vector4d(0, 0, 1, 0.4), "astar-visit", 0, 6);
  //   }
  //   astar_num += 1;
  //   astar_time = (node_->now() - t1).seconds();
  //   RCLCPP_WARN(node_->get_logger(), "Average astar time: %lf", astar_time);
  // }
}

void FastExplorationFSM::triggerCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (msg->pose.position.z < -0.1) return;
  if (state_ != WAIT_TRIGGER) return;
  fd_->trigger_ = true;
  fd_->home_pos_ = fd_->odom_pos_;
  fd_->home_yaw_ = fd_->odom_yaw_;
  fd_->returning_home_ = false;
  RCLCPP_INFO(node_->get_logger(), "Triggered! Home set to [%.2f, %.2f, %.2f], yaw %.2f",
              fd_->home_pos_(0), fd_->home_pos_(1), fd_->home_pos_(2), fd_->home_yaw_);
  transitState(PLAN_TRAJ, "triggerCallback");
}

void FastExplorationFSM::safetyCallback() {
  if (state_ == EXPL_STATE::EXEC_TRAJ) {
    // Check safety and trigger replan if necessary
    double dist;
    bool safe = planner_manager_->checkTrajCollision(dist);
    if (!safe) {
      RCLCPP_WARN(node_->get_logger(), "Replan: collision detected==================================");
      if (fd_->returning_home_) {
        transitState(PLAN_RETURN, "safetyCallback");
      } else {
        transitState(PLAN_TRAJ, "safetyCallback");
      }
    }
  }
}

void FastExplorationFSM::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  fd_->odom_pos_(0) = msg->pose.pose.position.x;
  fd_->odom_pos_(1) = msg->pose.pose.position.y;
  fd_->odom_pos_(2) = msg->pose.pose.position.z;

  fd_->odom_vel_(0) = msg->twist.twist.linear.x;
  fd_->odom_vel_(1) = msg->twist.twist.linear.y;
  fd_->odom_vel_(2) = msg->twist.twist.linear.z;

  fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
  fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
  fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
  fd_->odom_orient_.z() = msg->pose.pose.orientation.z;

  Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
  fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0));

  fd_->have_odom_ = true;
}

void FastExplorationFSM::transitState(EXPL_STATE new_state, string pos_call) {
  int pre_s = int(state_);
  state_ = new_state;
  cout << "[" + pos_call + "]: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)]
       << endl;
}
}  // namespace fast_planner
