#include "arch_nav_flight_plan_executor/flight_plan_executor.hpp"

#include "arch_nav/model/report/takeoff_report.hpp"
#include "arch_nav/model/report/waypoint_report.hpp"

namespace arch_nav_flight_plan_executor {

using arch_nav::constants::CommandResponse;
using arch_nav::constants::OperationStatus;
using arch_nav::report::ReportStatus;

namespace {

const char* op_type_to_cstr(FlightPlanOperation::Type type) {
  switch (type) {
    case FlightPlanOperation::Type::TAKEOFF: return "TAKEOFF";
    case FlightPlanOperation::Type::CHANGE_YAW: return "CHANGE_YAW";
    case FlightPlanOperation::Type::WAYPOINT_FOLLOWING: return "WAYPOINT_FOLLOWING";
    case FlightPlanOperation::Type::LAND: return "LAND";
    case FlightPlanOperation::Type::SET_ROI: return "SET_ROI";
    default: return "UNKNOWN";
  }
}

}  // namespace

FlightPlanExecutor::FlightPlanExecutor(arch_nav::ArchNavApi& api, rclcpp::Node& node)
: api_(api), node_(node)
{
  api_.on_operation_complete([this](const arch_nav::report::OperationReport& report) {
    on_operation_complete(report);
  });
  api_.on_operation_progress([this](const arch_nav::report::OperationReport& report) {
    on_operation_progress(report);
  });
}

bool FlightPlanExecutor::is_busy() const
{
  return step_ != Step::IDLE && step_ != Step::DONE;
}

FlightPlanExecutor::Step FlightPlanExecutor::current_step() const
{
  return step_;
}

std::size_t FlightPlanExecutor::completed_operations() const
{
  return completed_count_;
}

bool FlightPlanExecutor::start(
    const FlightPlan& plan, ProgressCallback on_progress, CompletionCallback on_complete)
{
  if (is_busy()) {
    return false;
  }

  plan_ = plan;
  step_ = Step::WAITING_ARM;
  next_op_index_ = 0;
  executing_op_index_ = 0;
  completed_count_ = 0;
  cancel_requested_ = false;
  on_progress_ = std::move(on_progress);
  on_complete_ = std::move(on_complete);

  RCLCPP_INFO(node_.get_logger(), "Flight plan loaded (%zu operations) - waiting for vehicle ready",
      plan_.operations.size());

  timer_ = node_.create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { on_tick(); });

  return true;
}

void FlightPlanExecutor::request_cancel()
{
  if (!is_busy()) {
    return;
  }

  cancel_requested_ = true;

  if (step_ == Step::WAITING_OPERATION_COMPLETE) {
    RCLCPP_INFO(node_.get_logger(), "Cancel requested - aborting running operation");
    api_.cancel_operation();
    step_ = Step::CANCELING;
  } else if (step_ != Step::CANCELING) {
    finish(false, "canceled");
  }
}

void FlightPlanExecutor::finish(bool success, const std::string& message)
{
  RCLCPP_INFO(node_.get_logger(), "Flight plan finished: success=%d message=%s",
      success, message.c_str());

  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }
  step_ = Step::DONE;
  cancel_requested_ = false;

  auto callback = std::move(on_complete_);
  on_complete_ = nullptr;
  on_progress_ = nullptr;
  if (callback) {
    callback(success, message);
  }
}

bool FlightPlanExecutor::start_next_operation()
{
  if (next_op_index_ >= plan_.operations.size()) {
    finish(true, "completed");
    return true;
  }

  const auto& op = plan_.operations[next_op_index_];
  executing_op_index_ = next_op_index_;
  CommandResponse response = CommandResponse::DENIED;

  switch (op.type) {
    case FlightPlanOperation::Type::TAKEOFF:
      RCLCPP_INFO(node_.get_logger(), "[op %zu/%zu] takeoff: height=%.2f m",
          next_op_index_ + 1, plan_.operations.size(), op.takeoff_height);
      response = api_.takeoff(op.takeoff_height, op.frame);
      break;

    case FlightPlanOperation::Type::CHANGE_YAW:
      RCLCPP_INFO(node_.get_logger(), "[op %zu/%zu] change_yaw: yaw=%.3f rad",
          next_op_index_ + 1, plan_.operations.size(), op.yaw_rad);
      response = api_.change_yaw(op.yaw_rad, op.frame);
      break;

    case FlightPlanOperation::Type::WAYPOINT_FOLLOWING:
      RCLCPP_INFO(node_.get_logger(), "[op %zu/%zu] waypoint_following: %zu waypoints",
          next_op_index_ + 1, plan_.operations.size(), op.waypoints.size());
      response = api_.waypoint_following(op.waypoints, op.frame);
      break;

    case FlightPlanOperation::Type::LAND:
      RCLCPP_INFO(node_.get_logger(), "[op %zu/%zu] land",
          next_op_index_ + 1, plan_.operations.size());
      response = api_.land();
      break;

    case FlightPlanOperation::Type::SET_ROI:
      RCLCPP_INFO(node_.get_logger(), "[op %zu/%zu] set_roi: lat=%.6f lon=%.6f alt=%.1f",
          next_op_index_ + 1, plan_.operations.size(),
          op.roi_target.lat, op.roi_target.lon, op.roi_target.alt);
      response = api_.set_roi(op.roi_target, op.frame);
      break;
  }

  if (response != CommandResponse::ACCEPTED) {
    finish(false, "operation " + std::to_string(next_op_index_ + 1) + " rejected");
    return false;
  }

  ++next_op_index_;

  // set_roi is a fire-and-forget command - no on_operation_complete callback
  // will fire for it, so it counts as completed immediately and we go
  // straight back to waiting for idle to start the next operation.
  if (op.type == FlightPlanOperation::Type::SET_ROI) {
    ++completed_count_;
    step_ = Step::WAITING_IDLE_TO_START_OP;
  } else {
    step_ = Step::WAITING_OPERATION_COMPLETE;
  }
  return true;
}

void FlightPlanExecutor::on_tick()
{
  const auto status = api_.operation_status();

  switch (step_) {
    case Step::WAITING_ARM:
      if (status == OperationStatus::DISARMED) {
        RCLCPP_INFO(node_.get_logger(), "Vehicle connected - arming");
        api_.arm();
        step_ = Step::ARMING;
      }
      break;

    case Step::ARMING:
      if (status == OperationStatus::IDLE) {
        RCLCPP_INFO(node_.get_logger(), "Armed - starting flight plan operations");
        step_ = Step::WAITING_IDLE_TO_START_OP;
      }
      break;

    case Step::WAITING_IDLE_TO_START_OP:
      if (status == OperationStatus::IDLE) {
        start_next_operation();
      }
      break;

    case Step::WAITING_OPERATION_COMPLETE:
    case Step::CANCELING:
    case Step::IDLE:
    case Step::DONE:
      break;
  }
}

void FlightPlanExecutor::on_operation_progress(const arch_nav::report::OperationReport& report)
{
  if (step_ != Step::WAITING_OPERATION_COMPLETE && step_ != Step::CANCELING) {
    return;
  }
  if (!on_progress_) {
    return;
  }

  const auto& op = plan_.operations[executing_op_index_];
  const char* op_type = op_type_to_cstr(op.type);

  if (const auto* r = dynamic_cast<const arch_nav::report::TakeoffReport*>(&report)) {
    on_progress_(executing_op_index_, plan_.operations.size(), op_type,
        static_cast<float>(r->execution_state().current_altitude.load()),
        static_cast<float>(r->execution_state().target_altitude.load()),
        "altitude progress");
  } else if (const auto* r = dynamic_cast<const arch_nav::report::WaypointReport*>(&report)) {
    on_progress_(executing_op_index_, plan_.operations.size(), op_type,
        static_cast<float>(r->execution_state().current_waypoint.load()),
        static_cast<float>(r->execution_state().total_waypoints.load()),
        "waypoint progress");
  } else {
    on_progress_(executing_op_index_, plan_.operations.size(), op_type, 0.0f, 0.0f, "");
  }
}

void FlightPlanExecutor::on_operation_complete(const arch_nav::report::OperationReport& report)
{
  if (step_ == Step::CANCELING) {
    finish(false, "canceled");
    return;
  }

  if (step_ != Step::WAITING_OPERATION_COMPLETE) {
    return;
  }

  if (report.status() == ReportStatus::ABORTED) {
    finish(false, "operation aborted or vehicle lost control");
    return;
  }

  if (report.status() == ReportStatus::FAILED) {
    finish(false, "operation failed");
    return;
  }

  ++completed_count_;
  step_ = Step::WAITING_IDLE_TO_START_OP;
}

}  // namespace arch_nav_flight_plan_executor
