cmake_minimum_required(VERSION 3.8)
project(zebra)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. FIND DEPENDENCIES
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# --- THE FIX: Use PkgConfig instead of find_package(LibSerial) ---
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)
# -----------------------------------------------------------------

# 2. BUILD THE C++ HARDWARE INTERFACE LIBRARY
add_library(zebra_hardware SHARED
  hardware/zebra_system.cpp
)

# Include directories:
# 1. Your local 'include' folder
# 2. The LibSerial includes found by PkgConfig
target_include_directories(zebra_hardware PRIVATE 
  include
  ${LIBSERIAL_INCLUDE_DIRS}
)

# Link against ROS 2 libraries
ament_target_dependencies(zebra_hardware
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

# --- THE FIX: Link using the PkgConfig libraries variable ---
target_link_libraries(zebra_hardware 
  ${LIBSERIAL_LIBRARIES}
)
# ------------------------------------------------------------

# 3. EXPORT THE PLUGIN
pluginlib_export_plugin_description_file(hardware_interface zebra_hardware_plugin.xml)

# 4. INSTALL THE C++ LIBRARY
install(TARGETS zebra_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 5. INSTALL HEADERS
# This assumes your file is at: src/zebra/include/zebra/arduino_comms.hpp
install(DIRECTORY hardware/include/ 
  DESTINATION include
)

# 6. INSTALL PLUGIN XML
install(FILES zebra_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# 7. INSTALL LAUNCH, CONFIG, URDF
install(
  DIRECTORY urdf launch config 
  DESTINATION share/${PROJECT_NAME}
)

# 8. INSTALL SCRIPTS
install(PROGRAMS
  scripts/odom.py 
  DESTINATION lib/${PROJECT_NAME}
)

# 9. EXPORT ROS DEPENDENCIES
ament_export_include_directories(include)
ament_export_libraries(zebra_hardware)
ament_export_dependencies(hardware_interface rclcpp rclcpp_lifecycle)

ament_package()