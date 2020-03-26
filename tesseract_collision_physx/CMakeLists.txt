cmake_minimum_required(VERSION 3.5.0)
project(tesseract_collision_physx VERSION 0.1.0 LANGUAGES CXX)

find_package(Eigen3 REQUIRED)
find_package(console_bridge REQUIRED)
find_package(tesseract_geometry REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_collision REQUIRED)

# Testing set Physx root
set(PHYSX_ROOT_DIR /home/larmstrong/catkin_ws/trajopt_only_ws/src/PhysX/physx/install/linux)
#message(FATAL_ERROR ${PHYSX_ROOT_DIR})

link_directories(${PHYSX_ROOT_DIR}/PhysX/bin/linux.clang/debug)
set(PHYSX_LIBRARIES libPhysXExtensions_static_64.a libPhysX_static_64.a libPhysXPvdSDK_static_64.a libPhysXVehicle_static_64.a libPhysXCharacterKinematic_static_64.a libPhysXCooking_static_64.a libPhysXCommon_static_64.a libPhysXFoundation_static_64.a)
add_library(${PROJECT_NAME} SHARED
  src/tesseract_physx.cpp
  src/tesseract_simulation_event_callback.cpp
  src/physx_collision_object_wrapper.cpp
  src/utils.cpp
  src/physx_discrete_manager.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC tesseract::tesseract_collision_core tesseract::tesseract_geometry console_bridge ${PHYSX_LIBRARIES} ${CMAKE_DL_LIBS})
tesseract_target_compile_options(${PROJECT_NAME} PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${PHYSX_ROOT_DIR}/PhysX/include
    ${PHYSX_ROOT_DIR}/PxShared/include)

# Create target that profiles the collision checkers.
add_executable(${PROJECT_NAME}_demo src/test.cpp)
target_link_libraries(${PROJECT_NAME}_demo ${PROJECT_NAME})
tesseract_target_compile_options(${PROJECT_NAME}_demo PRIVATE)
tesseract_clang_tidy(${PROJECT_NAME}_demo)
target_compile_definitions(${PROJECT_NAME}_demo PRIVATE DATA_DIR="${CMAKE_SOURCE_DIR}/test")

# Create target that profiles the collision checkers.
add_executable(${PROJECT_NAME}_split_sim src/split_simulation.cpp)
target_link_libraries(${PROJECT_NAME}_split_sim ${PROJECT_NAME})
tesseract_target_compile_options(${PROJECT_NAME}_split_sim PRIVATE)
tesseract_clang_tidy(${PROJECT_NAME}_split_sim)
target_compile_definitions(${PROJECT_NAME}_split_sim PRIVATE DATA_DIR="${CMAKE_SOURCE_DIR}/test")

# Create target that profiles the collision checkers.
add_executable(${PROJECT_NAME}_tesseract_physx src/tesseract_physx_test.cpp)
target_link_libraries(${PROJECT_NAME}_tesseract_physx ${PROJECT_NAME})
tesseract_target_compile_options(${PROJECT_NAME}_tesseract_physx PRIVATE)
tesseract_clang_tidy(${PROJECT_NAME}_tesseract_physx)

tesseract_configure_package(${PROJECT_NAME} ${PROJECT_NAME}_demo ${PROJECT_NAME}_split_sim ${PROJECT_NAME}_tesseract_physx)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
 )

# Create an ament_index resource file to allow ROS2 ament_index_cpp to locate the installed path to this package.
# This is a workaround to let the ROS2 version of pluginlib find tesseract_collision's plugins, since tesseract_collision is a non-ROS CMake package.
# ADDITIONAL REQUIREMENT: The installed path must be added to the AMENT_PREFIX_PATH environment variable at runtime, which is outside the scope of CMakeLists.txt.
file(WRITE ${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages/${PROJECT_NAME} "")

if (TESSERACT_ENABLE_TESTING)
  enable_testing()
  tesseract_add_run_tests_target()
  add_subdirectory(test)
endif()
