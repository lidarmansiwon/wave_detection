cmake_minimum_required(VERSION 3.5)
project(wave_detection)

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
find_package(rclcpp REQUIRED)       
find_package(std_msgs REQUIRED)    
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED) # cv_bridge 추가
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui) # OpenCV 추가 (필요한 모듈)
# find_package(visualization_msgs REQUIRED) # 마커 사용 시 추가

# ament_auto_add_library(wave_detection SHARED
#   src/wave_detection.cpp
# )

include_directories(include)

# Include install path
install(
  DIRECTORY include/
  DESTINATION include
)

add_executable(wave_detection src/main.cpp src/wave_detection.cpp)

ament_target_dependencies(wave_detection
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  OpenCV
  # visualization_msgs # 마커 사용 시 추가
)



# Set include directories
target_include_directories(wave_detection PUBLIC ${INCLUDE_DIRECTORIES})

install(TARGETS
  wave_detection

  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
