cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Bağımlılıkları bul
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_robot_interfaces REQUIRED)


# 1. İlk düğümün
add_executable(cpp_node src/my_first_node.cpp)
ament_target_dependencies(cpp_node rclcpp)

# 2. Template düğümün (Dosya adının src içinde tam olarak bu olduğundan emin ol!)
add_executable(template_node src/template_node.cpp)
ament_target_dependencies(template_node rclcpp)

# 3. Robot News Station (Burada her yere 's' ekledim ki dosyanla tam uyuşsun)
add_executable(robot_news_stations src/robot_news_stations.cpp)
ament_target_dependencies(robot_news_stations rclcpp example_interfaces)

add_executable(smartphone src/smartphone.cpp)
ament_target_dependencies(smartphone rclcpp example_interfaces)

# Publisher için
add_executable(number_publisher src/number_publisher.cpp)
ament_target_dependencies(number_publisher rclcpp example_interfaces)

# Counter için
add_executable(number_counter src/number_counter.cpp)
ament_target_dependencies(number_counter rclcpp example_interfaces)

add_executable(add_two_ints_server src/add_two_ints_server.cpp)
ament_target_dependencies(add_two_ints_server rclcpp example_interfaces)

add_executable(add_two_ints_client_no_oop src/add_two_ints_client_no_oop.cpp)
ament_target_dependencies(add_two_ints_client_no_oop rclcpp example_interfaces)

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
ament_target_dependencies(add_two_ints_client rclcpp example_interfaces)

add_executable(reset_counter_client src/reset_counter_client.cpp)
ament_target_dependencies(reset_counter_client rclcpp example_interfaces )

add_executable(hw_status_publisher src/hardware_status_publisher.cpp)
ament_target_dependencies(hw_status_publisher rclcpp my_robot_interfaces)

# Kurulum (Install) - add_executable içindeki ilk isimlerle AYNI olmalı
install(TARGETS 
  cpp_node
  template_node
  robot_news_stations
  smartphone
  number_counter
  number_publisher
  add_two_ints_server
  add_two_ints_client_no_oop
  add_two_ints_client
  reset_counter_client
  hw_status_publisher

  DESTINATION lib/${PROJECT_NAME})

ament_package()