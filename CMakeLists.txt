cmake_minimum_required(VERSION 3.8)
project(mbot_hardware)

option(USE_SANITIZERS "Enable various runtime sanitizers")

if(USE_SANITIZERS)
    set(SANITIZER_FLAGS "-fsanitize=undefined,leak,address -fstack-protector-all -static-libasan")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(libmbot REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(
    mbot_hardware
    SHARED
    src/mbot_serial.cpp
)
target_include_directories(
    mbot_hardware
    PUBLIC
    include
)

target_link_libraries(mbot_hardware ${SANITIZER_FLAGS} libmbot::mbot)

ament_target_dependencies(
    mbot_hardware
    hardware_interface
    rclcpp
    rclcpp_lifecycle
)

# prevent pluginlib from using boost
target_compile_definitions(mbot_hardware PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
    hardware_interface mbot_hardware.xml)

install(
    TARGETS
    mbot_hardware
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(
    DIRECTORY include/
    DESTINATION include
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)

    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)

    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_gmock REQUIRED)
    find_package(ros2_control_test_assets REQUIRED)

    ament_add_gmock(test_mbot_serial test/test_mbot_serial.cpp)
    target_include_directories(test_mbot_serial PRIVATE include)
    ament_target_dependencies(
        test_mbot_serial
        hardware_interface
        pluginlib
        ros2_control_test_assets
    )
endif()

ament_export_include_directories(
    include
)
ament_export_libraries(
    mbot_hardware
)
ament_export_dependencies(
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
)

ament_package()
