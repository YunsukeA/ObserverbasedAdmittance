#include "ObserverbasedAdmittance.h"

#include <SpaceVecAlg/EigenTypedef.h>

ObserverbasedAdmittance::ObserverbasedAdmittance(
    mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config)
    : mc_control::MCController(rm, dt),
      getestimatedContactWrench_(Eigen::Vector6d::Zero()),
      getestimatedExternalWrench_(Eigen::Vector6d::Zero()) {
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(kinematicsConstraint);
  solver().addTask(postureTask);
  solver().setContacts({{}});

  MaxContacts = config("MaxContacts", 4);

  robot_ = config("robot", robot().name());
  if (config.has("exportValue")) {
    auto exportValueConfig = config("exportValue");
    if (exportValueConfig.has("exportContactWrench")) {
      exportValueConfig("exportContactWrench", exportContactWrench_);
    }
    if (exportValueConfig.has("exportExternalWrench")) {
      exportValueConfig("exportExternalWrench", exportExternalWrench_);
    }
  }
  mc_rtc::log::info("exportContactWrench_: {}", exportContactWrench_);
  mc_rtc::log::info("exportExternalWrench_: {}", exportExternalWrench_);

  mc_rtc::log::success("ObserverbasedAdmittance init done ");
}

bool ObserverbasedAdmittance::run() {
  if (exportContactWrench_) {
    for (int i = 0; i < MaxContacts; i++) {
      if (datastore().has(robot_ + "::estimatedContactWrench_" +
                          std::to_string(i))) {
        getestimatedContactWrench_ = datastore().get<Eigen::Vector6d>(
            robot_ + "::estimatedContactWrench_" + std::to_string(i));
      }

      mc_rtc::log::info("[DATASTORE]Exporting estimatedContactWrench_{}: \n {}",
                        i, getestimatedContactWrench_);
    }
  }
  if (exportExternalWrench_) {
    if (datastore().has(robot_ + "::estimatedExternalWrench")) {
      getestimatedExternalWrench_ = datastore().get<Eigen::Vector6d>(
          robot_ + "::estimatedExternalWrench");
    }
    mc_rtc::log::info("[DATASTORE]Exporting estimatedExternalWrench: \n {}",
                      getestimatedExternalWrench_);
  }
  return mc_control::MCController::run();
}

void ObserverbasedAdmittance::reset(
    const mc_control::ControllerResetData &reset_data) {
  mc_control::MCController::reset(reset_data);
}

CONTROLLER_CONSTRUCTOR("ObserverbasedAdmittance", ObserverbasedAdmittance)
