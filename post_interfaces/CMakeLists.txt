cmake_minimum_required(VERSION 3.5)
project(post_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Parcel.msg"
  "msg/KeyValue.msg"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

