find_package(benchmark REQUIRED)
find_package(tesseract_scene_graph REQUIRED)

macro(add_benchmark benchmark_name benchmark_file)
  add_executable(${benchmark_name} ${benchmark_file})
  tesseract_target_compile_options(${benchmark_name} PRIVATE)
  tesseract_clang_tidy(${benchmark_name})
  target_link_libraries(${benchmark_name}
      benchmark::benchmark
      tesseract::tesseract_collision_test_suite_benchmarks
      ${PROJECT_NAME}
      tesseract::tesseract_geometry
      console_bridge
      ${Boost_LIBRARIES}
      ${OCTOMAP_LIBRARIES}
      )
  target_include_directories(${benchmark_name} PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
  add_dependencies(${benchmark_name}
#    tesseract_collision_test_suite_benchmarks
    ${PROJECT_NAME})
  tesseract_add_run_benchmark_target(${benchmark_name})
endmacro()

add_benchmark(${PROJECT_NAME}_physx_discrete_benchmarks physx_discrete_benchmarks.cpp)
