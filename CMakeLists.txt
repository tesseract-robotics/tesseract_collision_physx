cmake_minimum_required(VERSION 3.5.0)
project(tesseract_collision_physx VERSION 0.1.0 LANGUAGES CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)
find_package(console_bridge REQUIRED)
find_package(tesseract_geometry REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_collision REQUIRED)
find_package(cmake_common_scripts REQUIRED)

if(NOT TARGET console_bridge::console_bridge)
  add_library(console_bridge::console_bridge INTERFACE IMPORTED)
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${console_bridge_INCLUDE_DIRS}")
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_LINK_LIBRARIES "${console_bridge_LIBRARIES}")
endif()

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

# Create imported target physx::physx
add_library(physx::physx INTERFACE IMPORTED)
set(PHYSX_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/physx_41/linux)
set(PHYSX_LIBRARY_DIR ${PHYSX_ROOT_DIR}/PhysX/bin/release)
set_target_properties(physx::physx PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${PHYSX_ROOT_DIR}/PhysX/include;${PHYSX_ROOT_DIR}/PxShared/include"
  INTERFACE_LINK_LIBRARIES "${PHYSX_LIBRARY_DIR}/libPhysXExtensions_static_64.a;${PHYSX_LIBRARY_DIR}/libPhysX_static_64.a;${PHYSX_LIBRARY_DIR}/libPhysXPvdSDK_static_64.a;${PHYSX_LIBRARY_DIR}/libPhysXVehicle_static_64.a;${PHYSX_LIBRARY_DIR}/libPhysXCharacterKinematic_static_64.a;${PHYSX_LIBRARY_DIR}/libPhysXCooking_static_64.a;${PHYSX_LIBRARY_DIR}/libPhysXCommon_static_64.a;${PHYSX_LIBRARY_DIR}/libPhysXFoundation_static_64.a;${PHYSX_LIBRARY_DIR}/libPhysXGpu_64.so"
)

add_library(${PROJECT_NAME}
  src/tesseract_physx.cpp
  src/tesseract_physx_scene.cpp
  src/tesseract_simulation_event_callback.cpp
  src/tesseract_simulation_filter_callback.cpp
  src/physx_collision_object_wrapper.cpp
  src/utils.cpp
  src/physx_discrete_manager.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC tesseract::tesseract_collision_core tesseract::tesseract_geometry console_bridge::console_bridge physx::physx ${CMAKE_DL_LIBS} ${CMAKE_THREAD_LIBS_INIT})
target_compile_options(${PROJECT_NAME} PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(${PROJECT_NAME} ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS})

# Create target that profiles the collision checkers.
add_executable(${PROJECT_NAME}_demo src/test.cpp)
target_link_libraries(${PROJECT_NAME}_demo ${PROJECT_NAME})
target_compile_options(${PROJECT_NAME}_demo PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE} ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(${PROJECT_NAME}_demo ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_demo PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_compile_definitions(${PROJECT_NAME}_demo PRIVATE DATA_DIR="${CMAKE_SOURCE_DIR}/test")

# Create target that profiles the collision checkers.
add_executable(${PROJECT_NAME}_split_sim src/split_simulation.cpp)
target_link_libraries(${PROJECT_NAME}_split_sim ${PROJECT_NAME})
target_compile_options(${PROJECT_NAME}_split_sim PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE} ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(${PROJECT_NAME}_split_sim ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_split_sim PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_compile_definitions(${PROJECT_NAME}_split_sim PRIVATE DATA_DIR="${CMAKE_SOURCE_DIR}/test")

# Create target that profiles the collision checkers.
add_executable(${PROJECT_NAME}_tesseract_physx src/tesseract_physx_test.cpp)
target_link_libraries(${PROJECT_NAME}_tesseract_physx ${PROJECT_NAME})
target_compile_options(${PROJECT_NAME}_tesseract_physx PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE} ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(${PROJECT_NAME}_tesseract_physx ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_tesseract_physx PRIVATE VERSION ${TESSERACT_CXX_VERSION})

configure_package(NAMESPACE tesseract TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_demo ${PROJECT_NAME}_split_sim ${PROJECT_NAME}_tesseract_physx)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
 )

# Install Third Party PhysX includes
install(DIRECTORY third_party/physx_41/linux/PhysX/include/
  DESTINATION include/${PROJECT_NAME}/PhysX
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
 )

# Install Third Party PhysX PxShared includes
install(DIRECTORY third_party/physx_41/linux/PxShared/include/
  DESTINATION include/${PROJECT_NAME}/PxShared
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
 )

# Install Third Party PhysX libraries
install(DIRECTORY third_party/physx_41/linux/PhysX/bin/release/
  DESTINATION lib
  FILES_MATCHING PATTERN "*.a" PATTERN "*.so"
  PATTERN ".svn" EXCLUDE
 )

# Create an ament_index resource file to allow ROS2 ament_index_cpp to locate the installed path to this package.
# This is a workaround to let the ROS2 version of pluginlib find tesseract_collision's plugins, since tesseract_collision is a non-ROS CMake package.
# ADDITIONAL REQUIREMENT: The installed path must be added to the AMENT_PREFIX_PATH environment variable at runtime, which is outside the scope of CMakeLists.txt.
file(WRITE ${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages/${PROJECT_NAME} "")

if (TESSERACT_ENABLE_TESTING)
  enable_testing()
  add_run_tests_target(ENABLE ${TESSERACT_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()

if (TESSERACT_ENABLE_BENCHMARKING)
  add_subdirectory(test/benchmarks)
endif()
