#ifndef ARCH_NAV_FLIGHT_PLAN_EXECUTOR__FLIGHT_PLAN_EXECUTOR_HPP_
#define ARCH_NAV_FLIGHT_PLAN_EXECUTOR__FLIGHT_PLAN_EXECUTOR_HPP_

#include <cstddef>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "arch_nav/arch_nav_api.hpp"
#include "arch_nav_flight_plan_executor/flight_plan.hpp"

namespace arch_nav_flight_plan_executor {

// Adapted from arch_nav_json_flight_plan's MissionExecutor: same tick-driven
// state machine driving the kernel API, generalized to be re-invocable for a
// new plan after completion, cancelable mid-flight, and callback-driven
// (rather than log-only) so a ROS2 action server can surface feedback/result.
class FlightPlanExecutor {
 public:
  enum class Step {
    IDLE,
    WAITING_ARM,
    ARMING,
    WAITING_IDLE_TO_START_OP,
    WAITING_OPERATION_COMPLETE,
    CANCELING,
    DONE,
  };

  using ProgressCallback = std::function<void(
      std::size_t op_index, std::size_t op_count, const std::string& op_type,
      float progress_current, float progress_target, const std::string& detail)>;
  using CompletionCallback = std::function<void(bool success, const std::string& message)>;

  FlightPlanExecutor(arch_nav::ArchNavApi& api, rclcpp::Node& node);

  bool is_busy() const;
  Step current_step() const;
  std::size_t completed_operations() const;

  // Returns false without changing state if a plan is already in flight.
  bool start(const FlightPlan& plan, ProgressCallback on_progress, CompletionCallback on_complete);

  // No-op if no plan is in flight. Otherwise cancels the running operation
  // (if any) and reports completion once the cancellation settles.
  void request_cancel();

 private:
  void on_tick();
  bool start_next_operation();
  void finish(bool success, const std::string& message);
  void on_operation_complete(const arch_nav::report::OperationReport& report);
  void on_operation_progress(const arch_nav::report::OperationReport& report);

  arch_nav::ArchNavApi& api_;
  rclcpp::Node& node_;
  FlightPlan plan_;
  Step step_{Step::IDLE};
  std::size_t next_op_index_{0};
  std::size_t executing_op_index_{0};
  std::size_t completed_count_{0};
  bool cancel_requested_{false};
  ProgressCallback on_progress_;
  CompletionCallback on_complete_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace arch_nav_flight_plan_executor

#endif  // ARCH_NAV_FLIGHT_PLAN_EXECUTOR__FLIGHT_PLAN_EXECUTOR_HPP_
