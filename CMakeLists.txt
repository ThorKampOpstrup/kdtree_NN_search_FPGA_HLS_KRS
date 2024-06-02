cmake_minimum_required(VERSION 3.5)
project(kdtree_NN_search)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "RELEASE")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(ament_vitis)

include_directories(
  include
)

add_library(kdtree
  src/test_kdtree.cpp
)


if (ROS_VITIS)
  find_package(vitis_common REQUIRED)
  find_package(OpenCL REQUIRED)

  add_executable(kdtree_NN_search_test
    src/kdtree_NN_search_standalone_test.cpp
  )
  target_include_directories(kdtree_NN_search_test PUBLIC include)
  target_link_libraries(kdtree_NN_search_test
    kdtree
    ${OpenCL_LIBRARY}
    # ${PCL_LIBRARIES}
    pthread
  )
  ament_target_dependencies(kdtree_NN_search_test
    rclcpp
    std_msgs
    vitis_common
  )

  if(ROS_ACCELERATION)
    # C simulation and synthesis
    vitis_hls_generate_tcl(
      PROJECT
        project_kdtree_NN_search
      SRC
        src/kdtree_NN_search.cpp
      HEADERS
        include
      TOPFUNCTION
      kdtree_NN_search
      CLOCK
        5
      SYNTHESIS
    )


    # vadd kernel
    vitis_acceleration_kernel(
      NAME kdtree_NN_search
      FILE src/kdtree_NN_search.cpp
      CONFIG src/kv260.cfg
      INCLUDE
        include
      TYPE
        # sw_emu
        # hw_emu
        hw
      LINK
      PACKAGE
    )
  endif()

  install(TARGETS
    kdtree_NN_search_test
    DESTINATION lib/${PROJECT_NAME}
  )

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()  # ROS_VITIS

ament_package()
