cmake_minimum_required(VERSION 3.8)
project(api_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/POI.msg"

  "srv/FindShortestPathGPS.srv"
  "srv/FindShortestPathPOI.srv"
  "srv/StringResponse.srv"
  "srv/POICall.srv"
  "srv/POIInfo.srv"
  "srv/POIReturns.srv"
  "srv/Route.srv"
  "srv/RouteGPS.srv"
  "srv/GPSPoint.srv"
  "srv/POILink.srv"
  "srv/SaveRoute.srv"
  "srv/RouteInfo.srv"
  "srv/GPSPoints.srv"
  "srv/GPSRoutePoints.srv"
  "srv/ExportData.srv"

  "action/RouteAction.action"
  "action/GPSAction.action"
  
  DEPENDENCIES 
    builtin_interfaces std_msgs sensor_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
