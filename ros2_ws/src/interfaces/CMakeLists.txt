cmake_minimum_required(VERSION 3.8)
project(interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Annotation2D.msg"
  "msg/GroundedWaypoint.msg"
  "msg/EpisodeStatus.msg"
  "msg/EpisodeResult.msg"
  "msg/Collision.msg"
  "srv/SetEpisode.srv"
  # World control. These are what let a ROS-only client set up a scenario without
  # reaching for the sidecar socket - see todo.md R-01.
  "srv/ResetVehicle.srv"
  "srv/SpawnTraffic.srv"
  "srv/SetWeather.srv"
  "srv/DestroyActors.srv"
  "srv/SetCameraPose.srv"
  "srv/ChaseRecording.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
