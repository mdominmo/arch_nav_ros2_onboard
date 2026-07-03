#include "arch_nav_flight_plan_executor/flight_plan_parser.hpp"

#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

namespace arch_nav_flight_plan_executor {
namespace {

arch_nav::constants::ReferenceFrame parse_reference_frame(const std::string& frame) {
  using arch_nav::constants::ReferenceFrame;
  if (frame == "GLOBAL_WGS84") return ReferenceFrame::GLOBAL_WGS84;
  if (frame == "LOCAL_NED") return ReferenceFrame::LOCAL_NED;
  if (frame == "LOCAL_ENU") return ReferenceFrame::LOCAL_ENU;
  if (frame == "BODY_FCS") return ReferenceFrame::BODY_FCS;
  throw std::invalid_argument("Unsupported reference frame: " + frame);
}

FlightPlanOperation parse_takeoff_op(const nlohmann::json& payload) {
  FlightPlanOperation op;
  op.type = FlightPlanOperation::Type::TAKEOFF;
  op.takeoff_height = payload.at("height").get<double>();
  op.frame = parse_reference_frame(payload.value("frame", std::string("LOCAL_NED")));
  return op;
}

FlightPlanOperation parse_change_yaw_op(const nlohmann::json& payload) {
  FlightPlanOperation op;
  op.type = FlightPlanOperation::Type::CHANGE_YAW;
  op.yaw_rad = payload.at("yaw_rad").get<double>();
  op.frame = parse_reference_frame(payload.value("frame", std::string("BODY_FCS")));
  return op;
}

FlightPlanOperation parse_waypoint_following_op(const nlohmann::json& payload) {
  FlightPlanOperation op;
  op.type = FlightPlanOperation::Type::WAYPOINT_FOLLOWING;
  const nlohmann::json* wp_array = nullptr;
  if (payload.is_array()) {
    op.frame = arch_nav::constants::ReferenceFrame::GLOBAL_WGS84;
    wp_array = &payload;
  } else if (payload.is_object()) {
    op.frame = parse_reference_frame(payload.value("frame", std::string("GLOBAL_WGS84")));
    wp_array = &payload.at("items");
  } else {
    throw std::invalid_argument("waypoints payload must be array or object");
  }

  for (const auto& wp : *wp_array) {
    op.waypoints.emplace_back(
        wp.at("latitude").get<double>(),
        wp.at("longitude").get<double>(),
        wp.at("altitude").get<double>());
  }
  return op;
}

FlightPlanOperation parse_land_op(const nlohmann::json&) {
  FlightPlanOperation op;
  op.type = FlightPlanOperation::Type::LAND;
  return op;
}

FlightPlanOperation parse_set_roi_op(const nlohmann::json& payload) {
  FlightPlanOperation op;
  op.type = FlightPlanOperation::Type::SET_ROI;
  op.roi_target = arch_nav::vehicle::GlobalPosition{
      payload.at("latitude").get<double>(),
      payload.at("longitude").get<double>(),
      payload.at("altitude").get<double>()};
  op.frame = parse_reference_frame(payload.value("frame", std::string("GLOBAL_WGS84")));
  return op;
}

}  // namespace

FlightPlan FlightPlanParser::parse(const std::string& json_text)
{
  nlohmann::json json;
  try {
    json = nlohmann::json::parse(json_text);
  } catch (const nlohmann::json::parse_error& e) {
    throw std::invalid_argument(std::string("Invalid JSON: ") + e.what());
  }

  if (!json.is_object() || !json.contains("operations") || !json.at("operations").is_array()) {
    throw std::invalid_argument("Flight plan must contain an 'operations' array");
  }

  FlightPlan plan;
  const auto& operations = json.at("operations");

  for (std::size_t i = 0; i < operations.size(); ++i) {
    const auto& entry = operations.at(i);
    if (!entry.is_object() || entry.size() != 1) {
      throw std::invalid_argument(
          "Operation at index " + std::to_string(i) +
          " must be an object with a single operation key");
    }

    const auto it = entry.begin();
    const std::string op_name = it.key();
    const auto& payload = it.value();

    try {
      if (op_name == "takeoff") {
        plan.operations.push_back(parse_takeoff_op(payload));
      } else if (op_name == "change_yaw") {
        plan.operations.push_back(parse_change_yaw_op(payload));
      } else if (op_name == "waypoints") {
        plan.operations.push_back(parse_waypoint_following_op(payload));
      } else if (op_name == "land") {
        plan.operations.push_back(parse_land_op(payload));
      } else if (op_name == "set_roi") {
        plan.operations.push_back(parse_set_roi_op(payload));
      } else {
        throw std::invalid_argument(
            "Unsupported operation '" + op_name + "' at index " + std::to_string(i));
      }
    } catch (const nlohmann::json::exception& e) {
      throw std::invalid_argument(
          "Operation '" + op_name + "' at index " + std::to_string(i) + ": " + e.what());
    }
  }

  return plan;
}

}  // namespace arch_nav_flight_plan_executor
