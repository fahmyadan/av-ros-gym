cmake_minimum_required(VERSION 3.5)
project(sceneexp_utils)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_python_install_package(${PROJECT_NAME}
  SETUP_CFG
    ${PROJECT_NAME}/setup.cfg
  SCRIPTS_DESTINATION
    lib/${PROJECT_NAME}
)

# ament_auto_add_executable(
#   transform_from_odom
#   src/transform_from_odom.cpp
# )

install(
  DIRECTORY launch cone_maps
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
