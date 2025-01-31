#pragma once

#include <mc_control/MCController.h>
#include <mc_control/mc_controller.h>
#include <mc_tasks/EndEffectorTask.h>

#include "api.h"

struct ObserverbasedAdmittance_DLLAPI ObserverbasedAdmittance
    : public mc_control::MCController {
  ObserverbasedAdmittance(mc_rbdyn::RobotModulePtr rm, double dt,
                          const mc_rtc::Configuration &config);

  bool run() override;

  void reset(const mc_control::ControllerResetData &reset_data) override;

  std::shared_ptr<mc_tasks::EndEffectorTask> efTask;

  // std::string &usingObserver_;
  int MaxContacts;
  std::string robot_;
  // stateObservation::KineticsObserver observer_;

  bool exportContactWrench_;
  bool exportExternalWrench_;
  sva::ForceVecd estimatedContactWrench_0_surface;
  sva::ForceVecd estimatedContactWrench_1_surface;
  sva::ForceVecd estimatedContactWrench_2_surface;
  sva::ForceVecd estimatedContactWrench_3_surface;
  sva::ForceVecd estimatedExternalWrench_centroid;

  sva::ForceVecd estimatedContactWrench_0_sensor;
  sva::ForceVecd estimatedContactWrench_1_sensor;
  sva::ForceVecd estimatedContactWrench_2_sensor;
  sva::ForceVecd estimatedContactWrench_3_sensor;

  void addToGUI();
  void addTologger();

 public:
  void getestimatedContactWrench();
  void getestimatedExternalWrench();
  sva::ForceVecd wrenchTransformation(
      const sva::ForceVecd wrench, const std::string surface,
      const std::string forceSensor);  // surface frame to sensor frame

  void transformEstimatedWrenchs();

 protected:
  double t_ = 0;
};