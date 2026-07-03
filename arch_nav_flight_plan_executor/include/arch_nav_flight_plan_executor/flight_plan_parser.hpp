#ifndef ARCH_NAV_FLIGHT_PLAN_EXECUTOR__FLIGHT_PLAN_PARSER_HPP_
#define ARCH_NAV_FLIGHT_PLAN_EXECUTOR__FLIGHT_PLAN_PARSER_HPP_

#include <string>

#include "arch_nav_flight_plan_executor/flight_plan.hpp"

namespace arch_nav_flight_plan_executor {

class FlightPlanParser {
 public:
  // Parses the same JSON schema as arch_nav_json_flight_plan mission files,
  // from an in-memory string rather than a file. Throws std::invalid_argument
  // on any schema violation.
  static FlightPlan parse(const std::string& json_text);
};

}  // namespace arch_nav_flight_plan_executor

#endif  // ARCH_NAV_FLIGHT_PLAN_EXECUTOR__FLIGHT_PLAN_PARSER_HPP_
