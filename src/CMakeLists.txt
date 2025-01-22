set(controller_SRC
  ObserverbasedAdmittance.cpp
)

set(controller_HDR
  ObserverbasedAdmittance.h
)

add_controller(${PROJECT_NAME} ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DObserverbasedAdmittance_EXPORTS")

target_link_libraries(${PROJECT_NAME} PRIVATE mc_rtc::mc_control Eigen3::Eigen)
