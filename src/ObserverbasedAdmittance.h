#pragma once

#include "mc_state_observation/MCKineticsObserver.h"
#include <mc_control/MCController.h>
#include <mc_control/mc_controller.h>

#include "api.h"

struct ObserverbasedAdmittance_DLLAPI ObserverbasedAdmittance
    : public mc_control::MCController {
  ObserverbasedAdmittance(mc_rbdyn::RobotModulePtr rm, double dt,
                          const mc_rtc::Configuration &config);

  bool run(mc_control::MCController &ctl);

  void configure(const mc_control::MCController &ctl,
                 const mc_rtc::Configuration &config);

  void reset(const mc_control::ControllerResetData &reset_data) override;

  // std::string &usingObserver_;
  int MaxContacts;
  std::string robot_;
  // stateObservation::KineticsObserver observer_;

  bool exportContactWrench_;
  bool exportExternalWrench_;
};