#include <cstdlib>
#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "arch_nav/arch_nav.hpp"
#include "arch_nav_flight_plan_executor/flight_plan_executor_node.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // Bootstrap params are read via a throwaway node before the real node is
  // constructed, since ArchNav::create() must run before we can build
  // FlightPlanExecutorNode (it needs a live ArchNavApi&).
  auto bootstrap = std::make_shared<rclcpp::Node>("arch_nav_flight_plan_executor_bootstrap");
  bootstrap->declare_parameter("config", "");
  bootstrap->declare_parameter("driver", "");

  const auto config_path = bootstrap->get_parameter("config").as_string();
  const auto driver_name = bootstrap->get_parameter("driver").as_string();

  if (!config_path.empty()) {
    setenv("ARCH_NAV_DRIVER_CONFIG", config_path.c_str(), 1);
  }
  if (!driver_name.empty()) {
    setenv("ARCH_NAV_DRIVER", driver_name.c_str(), 1);
  }

  try {
    RCLCPP_INFO(bootstrap->get_logger(), "Initializing arch-nav");
    auto arch_nav = arch_nav::ArchNav::create();

    auto node = std::make_shared<arch_nav_flight_plan_executor::FlightPlanExecutorNode>(
        arch_nav->api());

    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(bootstrap->get_logger(), "Startup failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
