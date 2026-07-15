#ifndef ARCH_NAV_FLIGHT_PLAN_EXECUTOR__FLIGHT_PLAN_EXECUTOR_NODE_HPP_
#define ARCH_NAV_FLIGHT_PLAN_EXECUTOR__FLIGHT_PLAN_EXECUTOR_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "arch_nav/arch_nav_api.hpp"
#include "arch_nav_flight_plan_executor/flight_plan_executor.hpp"
#include "arch_nav_ros2_interfaces/action/execute_flight_plan.hpp"
#include "arch_nav_ros2_interfaces/msg/vehicle_telemetry.hpp"

namespace arch_nav_flight_plan_executor {

class FlightPlanExecutorNode : public rclcpp::Node {
 public:
  using ExecuteFlightPlan = arch_nav_ros2_interfaces::action::ExecuteFlightPlan;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteFlightPlan>;

  explicit FlightPlanExecutorNode(arch_nav::ArchNavApi& api);

 private:
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const ExecuteFlightPlan::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);

  void publish_telemetry();
  std::string resolve_vehicle_id() const;
  uint8_t executor_state_to_wire(FlightPlanExecutor::Step step) const;

  arch_nav::ArchNavApi& api_;
  FlightPlanExecutor executor_;
  std::string vehicle_id_;

  std::shared_ptr<GoalHandle> active_goal_handle_;
  std::string active_mission_id_;

  rclcpp_action::Server<ExecuteFlightPlan>::SharedPtr action_server_;
  rclcpp::Publisher<arch_nav_ros2_interfaces::msg::VehicleTelemetry>::SharedPtr telemetry_pub_;
  rclcpp::TimerBase::SharedPtr telemetry_timer_;
};

}  // namespace arch_nav_flight_plan_executor

#endif  // ARCH_NAV_FLIGHT_PLAN_EXECUTOR__FLIGHT_PLAN_EXECUTOR_NODE_HPP_
