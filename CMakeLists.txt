cmake_minimum_required(VERSION 3.8)
project(imgui_app)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# Znajdź pakiety ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
# Znajdź OpenGL i GLFW
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(GLEW REQUIRED)

# Ścieżki do ImGui (zainstalowany w Dockerfile)
set(IMGUI_DIR "/ws/src/imgui_app/Thirdparty/imgui")
set(IMPLOT_DIR "/ws/src/imgui_app/Thirdparty/implot")

# Pliki źródłowe ImGui
set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

# Pliki źródłowe ImPlot
set(IMPLOT_SOURCES
    ${IMPLOT_DIR}/implot.cpp
    ${IMPLOT_DIR}/implot_items.cpp
)

# Stwórz executable
add_executable(imgui_visualizer
    src/main.cpp
    ${IMGUI_SOURCES}
    ${IMPLOT_SOURCES}
)

# Include directories
target_include_directories(imgui_visualizer PUBLIC
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    ${IMPLOT_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(imgui_visualizer
    OpenGL::GL
    glfw
    GLEW::GLEW
    ${OpenCV_LIBS}
)

# ROS 2 dependencies
ament_target_dependencies(imgui_visualizer
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf2_ros
    cv_bridge
)

# Install executable
install(TARGETS imgui_visualizer
    DESTINATION lib/${PROJECT_NAME}
)
# Zainstaluj launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
