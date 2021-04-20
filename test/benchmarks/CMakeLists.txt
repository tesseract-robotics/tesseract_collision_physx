find_package(benchmark REQUIRED)
find_package(tesseract_scene_graph REQUIRED)

macro(add_benchmark benchmark_name benchmark_file)
  add_executable(${benchmark_name} ${benchmark_file})
  target_compile_options(${benchmark_name} PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE} ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_clang_tidy(${benchmark_name} ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(${benchmark_name} PRIVATE VERSION ${TESSERACT_CXX_VERSION})
  target_link_libraries(${benchmark_name}
      benchmark::benchmark
      tesseract::tesseract_collision_test_suite_benchmarks
      ${PROJECT_NAME}
      tesseract::tesseract_geometry
      console_bridge::console_bridge
      ${OCTOMAP_LIBRARIES}
      )
  target_include_directories(${benchmark_name} PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
  add_dependencies(${benchmark_name}
#    tesseract_collision_test_suite_benchmarks
    ${PROJECT_NAME})
  add_run_benchmark_target(${benchmark_name})
endmacro()

add_benchmark(${PROJECT_NAME}_physx_discrete_benchmarks physx_discrete_benchmarks.cpp)
