#include "arch_nav_flight_plan_executor/flight_plan_executor_node.hpp"

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include "arch_nav_flight_plan_executor/flight_plan_parser.hpp"

namespace arch_nav_flight_plan_executor {

namespace {
constexpr char kNamespacePrefix[] = "/uav_";
}  // namespace

FlightPlanExecutorNode::FlightPlanExecutorNode(arch_nav::ArchNavApi& api)
: rclcpp::Node("flight_plan_executor"),
  api_(api),
  executor_(api, *this)
{
  declare_parameter("vehicle_id", "");
  declare_parameter("telemetry_rate_hz", 5.0);

  vehicle_id_ = resolve_vehicle_id();
  RCLCPP_INFO(get_logger(), "Flight plan executor starting for vehicle_id='%s'", vehicle_id_.c_str());

  action_server_ = rclcpp_action::create_server<ExecuteFlightPlan>(
      this,
      "~/execute_flight_plan",
      [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const ExecuteFlightPlan::Goal> goal) {
        return handle_goal(uuid, goal);
      },
      [this](const std::shared_ptr<GoalHandle> goal_handle) {
        return handle_cancel(goal_handle);
      },
      [this](const std::shared_ptr<GoalHandle> goal_handle) {
        handle_accepted(goal_handle);
      });

  telemetry_pub_ = create_publisher<arch_nav_ros2_interfaces::msg::VehicleTelemetry>(
      "~/telemetry", rclcpp::QoS(5));

  const double rate_hz = get_parameter("telemetry_rate_hz").as_double();
  const auto period = std::chrono::duration<double>(1.0 / rate_hz);
  telemetry_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      [this]() { publish_telemetry(); });
}

std::string FlightPlanExecutorNode::resolve_vehicle_id() const
{
  const std::string ns = get_namespace();
  if (ns.rfind(kNamespacePrefix, 0) == 0) {
    return ns.substr(std::string(kNamespacePrefix).size());
  }

  const std::string fallback = get_parameter("vehicle_id").as_string();
  if (!fallback.empty()) {
    return fallback;
  }

  RCLCPP_WARN(get_logger(),
      "Node namespace '%s' does not match '/uav_<id>' and no 'vehicle_id' "
      "parameter was set - vehicle_id will be empty", ns.c_str());
  return "";
}

rclcpp_action::GoalResponse FlightPlanExecutorNode::handle_goal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const ExecuteFlightPlan::Goal> goal)
{
  if (executor_.is_busy()) {
    RCLCPP_WARN(get_logger(), "Rejecting goal '%s': a flight plan is already executing (active mission '%s')",
        goal->mission_id.c_str(), active_mission_id_.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  try {
    FlightPlanParser::parse(goal->flight_plan_json);
  } catch (const std::exception& e) {
    RCLCPP_WARN(get_logger(), "Rejecting goal '%s': invalid flight_plan_json: %s",
        goal->mission_id.c_str(), e.what());
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_logger(), "Accepting goal '%s'", goal->mission_id.c_str());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FlightPlanExecutorNode::handle_cancel(
    const std::shared_ptr<GoalHandle> /*goal_handle*/)
{
  RCLCPP_INFO(get_logger(), "Cancel requested for active mission '%s'", active_mission_id_.c_str());
  executor_.request_cancel();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void FlightPlanExecutorNode::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();

  FlightPlan plan;
  try {
    plan = FlightPlanParser::parse(goal->flight_plan_json);
  } catch (const std::exception& e) {
    // Already validated in handle_goal; this should not happen, but guard anyway.
    auto result = std::make_shared<ExecuteFlightPlan::Result>();
    result->success = false;
    result->message = std::string("invalid flight_plan_json: ") + e.what();
    result->operations_completed = 0;
    goal_handle->abort(result);
    return;
  }

  active_goal_handle_ = goal_handle;
  active_mission_id_ = goal->mission_id;

  const bool started = executor_.start(
      plan,
      [this](std::size_t op_index, std::size_t op_count, const std::string& op_type,
             float progress_current, float progress_target, const std::string& detail) {
        if (!active_goal_handle_) return;
        auto feedback = std::make_shared<ExecuteFlightPlan::Feedback>();
        feedback->operation_index = static_cast<uint8_t>(op_index);
        feedback->operation_count = static_cast<uint8_t>(op_count);
        feedback->operation_type = op_type;
        feedback->progress_current = progress_current;
        feedback->progress_target = progress_target;
        feedback->detail = detail;
        active_goal_handle_->publish_feedback(feedback);
      },
      [this](bool success, const std::string& message) {
        if (!active_goal_handle_) return;
        auto result = std::make_shared<ExecuteFlightPlan::Result>();
        result->success = success;
        result->message = message;
        result->operations_completed = static_cast<uint8_t>(executor_.completed_operations());

        auto goal_handle = active_goal_handle_;
        active_goal_handle_.reset();
        active_mission_id_.clear();

        if (message == "canceled") {
          goal_handle->canceled(result);
        } else if (success) {
          goal_handle->succeed(result);
        } else {
          goal_handle->abort(result);
        }
      });

  if (!started) {
    // Should not happen since handle_goal already checked is_busy(), guard anyway.
    auto result = std::make_shared<ExecuteFlightPlan::Result>();
    result->success = false;
    result->message = "executor busy";
    result->operations_completed = 0;
    active_goal_handle_.reset();
    active_mission_id_.clear();
    goal_handle->abort(result);
  }
}

uint8_t FlightPlanExecutorNode::executor_state_to_wire(FlightPlanExecutor::Step step) const
{
  switch (step) {
    case FlightPlanExecutor::Step::IDLE: return 0;
    case FlightPlanExecutor::Step::WAITING_ARM: return 1;
    case FlightPlanExecutor::Step::ARMING: return 2;
    case FlightPlanExecutor::Step::WAITING_IDLE_TO_START_OP: return 3;
    case FlightPlanExecutor::Step::WAITING_OPERATION_COMPLETE: return 4;
    case FlightPlanExecutor::Step::CANCELING: return 5;
    case FlightPlanExecutor::Step::DONE: return 6;
    default: return 0;
  }
}

void FlightPlanExecutorNode::publish_telemetry()
{
  const auto pos = api_.global_position();
  const auto kin = api_.kinematics();
  const auto status = api_.vehicle_status();

  arch_nav_ros2_interfaces::msg::VehicleTelemetry msg;
  msg.header.stamp = now();
  msg.vehicle_id = vehicle_id_;

  msg.latitude = pos.lat;
  msg.longitude = pos.lon;
  msg.altitude = pos.alt;
  msg.heading_rad = kin.heading;
  msg.vx = kin.vx;
  msg.vy = kin.vy;
  msg.vz = kin.vz;

  msg.arm_state = static_cast<int8_t>(status.arm_state);
  msg.control_state = static_cast<int8_t>(status.control_state);
  msg.operation_status = static_cast<int8_t>(api_.operation_status());

  msg.executor_state = executor_state_to_wire(executor_.current_step());
  msg.active_mission_id = active_mission_id_;

  telemetry_pub_->publish(msg);
}

}  // namespace arch_nav_flight_plan_executor
