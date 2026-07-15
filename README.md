# arch_nav_ros2_onboard

ROS 2 onboard packages for `arch_nav_gcs` — the per-UAV side of the ground control station, meant to run inside a per-vehicle Docker container alongside the `arch_nav` kernel and a MAVSDK driver.

## Packages

- **`arch_nav_flight_plan_executor`** — accepts a flight plan dynamically over a ROS 2 action (`ExecuteFlightPlan`) and drives it against the `arch_nav` kernel API, publishing live telemetry (`VehicleTelemetry`) throughout. Modeled on [`arch_nav_json_flight_plan`](https://github.com/mdominmo/arch_nav_json_flight_plan) as a template (same JSON mission schema, same kernel-driving state-machine pattern) but with no dependency on it: this node supports runtime mission dispatch and cancellation instead of a static `mission_file` parameter read once at startup, and exposes progress/telemetry over ROS 2 instead of only logging it.

## Dependencies

Depends on `arch_nav_ros2_interfaces` (the ROS 2 msg/action definitions) as a **colcon workspace sibling** — it's owned by the [`arch_nav_ros2_interfaces`](../arch_nav_ros2_interfaces) repo (which `arch_nav_gcs`'s `gcs_backend` also depends on), not by this repo. All three repos must be checked out side by side under the same `arch_nav_ws/src/` for a colcon build to resolve it, exactly like `arch_nav_mavsdk_px4_driver` depends on `arch_nav` today.

Also depends on the `arch_nav` kernel (`find_package(arch_nav CONFIG REQUIRED)`, installed system-wide — see [`arch_nav`](../arch_nav)'s build instructions) and a MAVSDK driver (`arch_nav_mavsdk_px4_driver` or `arch_nav_mavsdk_ardupilot_driver`) loaded dynamically at runtime via `ARCH_NAV_DRIVER`.

See `arch_nav_gcs/docs/topics.md` for the full ROS 2 naming convention, the JSON flight-plan schema, and the telemetry enum mappings this package implements.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select arch_nav_ros2_interfaces arch_nav_flight_plan_executor
```

(run from `arch_nav_ws/`, so colcon can see both this repo and `arch_nav_ros2_interfaces` under `src/`)

## Docker

Packaged via `arch_nav_gcs/docker/uav.Dockerfile` — see that repo's `docker/README.md` for build/run instructions (the build context spans all three repos: this one, `arch_nav_gcs`, and `arch_nav_ros2_interfaces`).
