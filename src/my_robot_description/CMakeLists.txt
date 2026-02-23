cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# ROS 2'de dosyaların kurulduğu iki ana yer vardır:

#lib/ (Kütüphane/Programlar): Burası senin o derlediğin .cpp dosyalarının (binary/çalıştırılabilir dosyalar) gittiği yerdir. Sadece programlar burada durur.

#share/ (Paylaşılan Dosyalar): Burası program olmayan; resimler, URDF dosyaları, Launch dosyaları ve Rviz ayarları gibi "verilerin" durduğu yerdir.
install(
  DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}/

)
ament_package()
