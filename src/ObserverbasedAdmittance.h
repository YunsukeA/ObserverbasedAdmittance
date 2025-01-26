#pragma once

#include <mc_control/MCController.h>
#include <mc_control/mc_controller.h>

#include "api.h"

struct ObserverbasedAdmittance_DLLAPI ObserverbasedAdmittance
    : public mc_control::MCController {
  ObserverbasedAdmittance(mc_rbdyn::RobotModulePtr rm, double dt,
                          const mc_rtc::Configuration &config);

  bool run() override;

  void reset(const mc_control::ControllerResetData &reset_data) override;

  // std::string &usingObserver_;
  int MaxContacts;
  std::string robot_;
  // stateObservation::KineticsObserver observer_;

  bool exportContactWrench_;
  bool exportExternalWrench_;
  Eigen::Vector6d getestimatedContactWrench_;
  Eigen::Vector6d getestimatedExternalWrench_;
};