#include <rclcpp/rclcpp.hpp>
#include <exploration_manager/fast_exploration_fsm.h>

#include <plan_manage/backward.hpp>
namespace backward {
backward::SignalHandling sh;
}

using namespace fast_planner;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("exploration_node");

  FastExplorationFSM expl_fsm;
  expl_fsm.init(node);

  rclcpp::sleep_for(std::chrono::seconds(1));
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
