cmake_minimum_required(VERSION 3.8)
project(ros2_line_lbd)

# Must use C++17 to make it compatible with rclcpp
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

# Check C++17 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)

if(COMPILER_SUPPORTS_CXX17)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
  add_definitions(-DCOMPILEDWITHC17)
  message(STATUS "Using flag -std=c++17.")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV 4.2 REQUIRED)
find_package(Eigen3 3.3.0 REQUIRED) # This may be redundant


# Header file locations [C++ node]
include_directories(include) # Add .hpp, .h files from include/ros2_orb_slam3
include_directories(${EIGEN3_INCLUDE_DIRS}) # Include headers for eigen3
include_directories(${OpenCV_INCLUDE_DIRS}) # include headers for pangolin


# # line_lbd_lib [internal library header files]
# include_directories(
#   ${PROJECT_SOURCE_DIR}
# )

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  OpenCV
)

## line_lbd_lib as a shared library
add_library(line_lbd_lib SHARED
  libs/binary_descriptor.cpp 
  libs/binary_descriptor_matcher.cpp 
  libs/LSDDetector.cpp  
  libs/lsd.cpp 
  libs/bitarray.hpp  
  libs/bitops.hpp 
  libs/draw.cpp 
  libs/precomp.hpp 
  libs/types.hpp  
  class/line_lbd_allclass.cpp
)


# set_target_properties(line_lbd_lib PROPERTIES VERSION "${line_lbd_lib_VERSION}")

# Declare build dependencies [According to ChatGPT]
ament_target_dependencies(line_lbd_lib
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Redundant?
# # Link libraries
# target_link_libraries(line_lbd_lib
#   ${OpenCV_LIBS}
#   ${EIGEN3_LIBS}
#   -lboost_system
# )

# To activate its internal node
# # Declare C++ executibles
# add_executable(linelbd_detect_node 
#   src/detect_lines.cpp
# )

# ament_target_dependencies(linelbd_detect_node 
#   PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
# )

# target_link_libraries(linelbd_detect_node PUBLIC line_lbd_lib) # Link a node with the internal shared library

# Install all the header files in package/package/include
install (DIRECTORY include/ 
        DESTINATION include
)

# Install the node and library together
# install(TARGETS linelbd_detect_node line_lbd_lib
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
#   INCLUDES DESTINATION include/${PROJECT_NAME}
# )

install(
  TARGETS line_lbd_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Tell downstream packages where to find the headers
# ament_export_include_directories(include)

# Help downstream packages to find transitive dependencies
# ament_export_dependencies(
#   line_lbd_lib
#   ${THIS_PACKAGE_INCLUDE_DEPENDS}
# )

ament_export_libraries(line_lbd_lib)
ament_export_include_directories(include) # Tell downstream packages where to find the headers

ament_package()
