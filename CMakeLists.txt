cmake_minimum_required(VERSION 3.16)
project(bag_manual_synchronizer CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra)
endif()

# ---------------------------------------------------------------- ROS 2 deps
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)

# ---------------------------------------------------------------- GUI deps
find_package(OpenGL REQUIRED)
find_package(glfw3 QUIET)
if(NOT glfw3_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(GLFW REQUIRED IMPORTED_TARGET glfw3)
  add_library(glfw ALIAS PkgConfig::GLFW)
endif()

find_package(yaml-cpp REQUIRED)

# ------------------------------------------------------- Dear ImGui + ImPlot
# Fetched from upstream by default. To build offline, point these cache
# variables at existing checkouts:
#   cmake -DIMGUI_SOURCE_DIR=/path/to/imgui -DIMPLOT_SOURCE_DIR=/path/to/implot ..
set(IMGUI_SOURCE_DIR "" CACHE PATH "Existing Dear ImGui checkout (empty = download)")
set(IMPLOT_SOURCE_DIR "" CACHE PATH "Existing ImPlot checkout (empty = download)")

include(FetchContent)

if(IMGUI_SOURCE_DIR STREQUAL "")
  FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.91.5
    GIT_SHALLOW TRUE)
  FetchContent_MakeAvailable(imgui)
  set(_imgui_dir ${imgui_SOURCE_DIR})
else()
  set(_imgui_dir ${IMGUI_SOURCE_DIR})
endif()

if(IMPLOT_SOURCE_DIR STREQUAL "")
  FetchContent_Declare(implot
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG v0.16
    GIT_SHALLOW TRUE)
  FetchContent_MakeAvailable(implot)
  set(_implot_dir ${implot_SOURCE_DIR})
else()
  set(_implot_dir ${IMPLOT_SOURCE_DIR})
endif()

add_library(imgui STATIC
  ${_imgui_dir}/imgui.cpp
  ${_imgui_dir}/imgui_draw.cpp
  ${_imgui_dir}/imgui_tables.cpp
  ${_imgui_dir}/imgui_widgets.cpp
  ${_imgui_dir}/backends/imgui_impl_glfw.cpp
  ${_imgui_dir}/backends/imgui_impl_opengl3.cpp
  ${_imgui_dir}/misc/cpp/imgui_stdlib.cpp)
target_include_directories(imgui SYSTEM PUBLIC
  ${_imgui_dir} ${_imgui_dir}/backends ${_imgui_dir}/misc/cpp)
target_link_libraries(imgui PUBLIC glfw OpenGL::GL)

add_library(implot STATIC
  ${_implot_dir}/implot.cpp
  ${_implot_dir}/implot_items.cpp)
target_include_directories(implot SYSTEM PUBLIC ${_implot_dir})
target_link_libraries(implot PUBLIC imgui)

# ---------------------------------------------------------------- executable
add_executable(bag_manual_synchronizer
  src/main.cpp
  src/bag_loader.cpp
  src/bag_exporter.cpp
  src/sync_config.cpp
  src/align.cpp
  src/app.cpp)

target_include_directories(bag_manual_synchronizer PRIVATE src)

target_link_libraries(bag_manual_synchronizer PRIVATE
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage
  ${sensor_msgs_TARGETS}
  rcpputils::rcpputils
  rosidl_typesupport_introspection_cpp::rosidl_typesupport_introspection_cpp
  imgui
  implot
  yaml-cpp)

install(TARGETS bag_manual_synchronizer DESTINATION lib/${PROJECT_NAME})

ament_package()
