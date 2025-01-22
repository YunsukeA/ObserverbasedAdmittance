cmake_minimum_required(VERSION 3.1)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 11)

set(PROJECT_NAME ObserverbasedAdmittance)
set(PROJECT_DESCRIPTION "ObserverbasedAdmittance")
set(PROJECT_URL "")

project(${PROJECT_NAME} CXX)

# Check if the project is built inside mc_rtc
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

find_package(Eigen3 REQUIRED)

# include_directories(${CMAKE_SOURCE_DIR}/../mc_state_observation)
# target_link_libraries(${PROJECT_NAME} PUBLIC mc_state_observation::mc_state_observation)
add_subdirectory(src)

set(AROBASE "@")
configure_file(etc/ObserverbasedAdmittance.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/ObserverbasedAdmittance.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/ObserverbasedAdmittance.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")


