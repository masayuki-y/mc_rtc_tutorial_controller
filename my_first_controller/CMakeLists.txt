cmake_minimum_required(VERSION 3.1)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 11)

set(PROJECT_NAME MyFirstController)
set(PROJECT_DESCRIPTION "MyFirstController")
set(PROJECT_URL "")

project(${PROJECT_NAME} CXX)

find_package(mc_rtc REQUIRED)

add_subdirectory(src)

set(AROBASE "@")
configure_file(etc/MyFirstController.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/MyFirstController.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/MyFirstController.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")

