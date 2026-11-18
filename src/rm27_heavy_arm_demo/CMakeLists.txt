cmake_minimum_required(VERSION 3.20)
project(rm27_heavy_arm_demo VERSION 1.0.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tech_core_planner REQUIRED)

add_library(${PROJECT_NAME} STATIC src/rm27_heavy_arm_demo.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}
  PUBLIC tech_core_planner::tech_core_planner
  PRIVATE ament_index_cpp::ament_index_cpp)

add_executable(rm27_planner_demo src/main.cpp)
target_link_libraries(rm27_planner_demo PRIVATE ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} rm27_planner_demo PROPERTIES CXX_EXTENSIONS OFF)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib)
install(TARGETS rm27_planner_demo
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY model launch rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
install(FILES README.md source_manifest.json DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_index_cpp tech_core_planner)

ament_package()
