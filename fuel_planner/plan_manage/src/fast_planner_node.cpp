#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <plan_manage/kino_replan_fsm.h>
#include <plan_manage/topo_replan_fsm.h>

#include <plan_manage/backward.hpp>
namespace backward {
backward::SignalHandling sh;
}

using namespace fast_planner;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("fast_planner_node");

  int planner;
  if (!nh->has_parameter("planner_node/planner")) {
    nh->declare_parameter("planner_node/planner", -1);
  }
  nh->get_parameter("planner_node/planner", planner);

  TopoReplanFSM topo_replan;
  KinoReplanFSM kino_replan;

  if (planner == 1) {
    kino_replan.init(nh);
  } else if (planner == 2) {
    topo_replan.init(nh);
  } else if (planner == 3) {
    RCLCPP_ERROR(nh->get_logger(),
                 "LocalExploreFSM source not available; choose planner 1 or 2.");
  }

  rclcpp::sleep_for(std::chrono::seconds(1));
  rclcpp::spin(nh);

  return 0;
}
