cmake_minimum_required(VERSION 3.5)
project(interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

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
find_package(api_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(rosidl_default_generators REQUIRED)
set(msg_files_misc
  "msg/PTZPosition.msg"
  "msg/ThermalCamera.msg"
  "msg/SwiftNavOutput.msg"
  "msg/DPad.msg"
  "msg/ImageCapture.msg"
  "msg/ModemInfoMSG.msg"
)

set (msg_files_bunker
  "msg/BunkerActuatorState.msg"
  "msg/BunkerRCState.msg"
  "msg/BunkerStatus.msg"
)
set(msg_files_scout
  "msg/ScoutActuatorState.msg"
  "msg/ScoutLightCmd.msg"
  "msg/ScoutLightState.msg"
  "msg/ScoutRCState.msg"
  "msg/ScoutStatus.msg"
)

set (msg_files_hunter
  "msg/HunterActuatorState.msg"
  "msg/HunterLightCmd.msg"
  "msg/HunterLightState.msg"
  "msg/HunterRCState.msg"
  "msg/HunterStatus.msg"
)

set (msg_files_ranger
  "msg/RangerActuatorState.msg"
  "msg/RangerRCState.msg"
  "msg/RangerSetting.msg"
  "msg/RangerStatus.msg"
)

set (msg_files_tracer
  "msg/TracerActuatorState.msg"
  "msg/TracerLightCmd.msg"
  "msg/TracerLightState.msg"
  "msg/TracerRCState.msg"
  "msg/TracerStatus.msg"
)

set (action_files
  "action/MoveTo.action"
  "action/VideoCapture.action"
  "action/PTZPosition.action"
)

set (srv_files
  "srv/ResetPTZ.srv"
  "srv/ImageCapture.srv"
  "srv/ListParameters.srv"
  "srv/ReadParameter.srv"
  "srv/SaveParameters.srv"
  "srv/WriteParameter.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files_misc}

  ${msg_files_bunker}

  ${msg_files_scout}

  ${msg_files_hunter}

  ${msg_files_ranger}

  ${msg_files_tracer}

  "msg/WiBoticInfo.msg"

  ${action_files}

  ${srv_files}
  DEPENDENCIES 
    builtin_interfaces std_msgs sensor_msgs action_msgs api_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
