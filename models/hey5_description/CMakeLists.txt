cmake_minimum_required(VERSION 3.4.0)
project(hey5_description)

find_package(ament_cmake_auto REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# @TODO LICENSE.txt needs to be installed too
ament_auto_package(INSTALL_TO_SHARE config gazebo launch meshes robots urdf)
