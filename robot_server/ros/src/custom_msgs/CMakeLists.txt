cmake_minimum_required(VERSION 3.0.2)
project(custom_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

# 메시지 파일 등록
add_message_files(
  FILES
  TaskPath.msg
)

# 메시지 생성 규칙
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# catkin 패키지 설정
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

