#include "ObserverbasedAdmittance.h"

ObserverbasedAdmittance::ObserverbasedAdmittance(
    mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config)
    : mc_control::MCController(rm, dt) {

  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(kinematicsConstraint);
  solver().addTask(postureTask);

  mc_rtc::log::success("ObserverbasedAdmittance init done ");
}

void ObserverbasedAdmittance::configure(const mc_control::MCController &ctl,
                                        const mc_rtc::Configuration &config) {
  MaxContacts = config("MaxContacts", 4);

  robot_ = config("robot", ctl.robot().name());
  if (config.has("exportValue")) {
    auto exportValueConfig = config("exportValue");
    if (exportValueConfig.has("exportContactWrench")) {
      exportValueConfig("exportContactWrench", exportContactWrench_);
    }
    if (exportValueConfig.has("exportExternalWrench")) {
      exportValueConfig("exportExternaltWrench", exportExternalWrench_);
    }
  }
}

bool ObserverbasedAdmittance::run(mc_control::MCController &ctl) {

  if (exportContactWrench_) {
    for (int i = 0; i < MaxContacts; i++) {
      //   if (ctl.datastore().has(
      //           robot_ + "::estimatedContactWrench_" +
      //           std::to_string(observer_.contactIndexNormal(i)))) {
      //   auto &getEstimatedConWrench =
      //     ctl.datastore().get<Eigen::Vector6d>(robot_ +
      //     "::estimatedContactWrench_" +
      //                         std::to_string(observer_.contactIndexNormal(i));

      //     mc_rtc::log::info("Estimated Wrench {}: {}", i,
      //     getEstimatedConWrench());
      //   }
      // };
    }
    if (exportExternalWrench_) {
      if (ctl.datastore().has(robot_ + "::estimatedExternalWrench")) {
        auto &getEstimatedExtWrench = ctl.datastore().get<Eigen::Vector6d>(
            robot_ + "::estimatedExternalWrench");

        mc_rtc::log::info("Estimated External Wrench: {}",
                          getEstimatedExtWrench.transpose());
      }
      return mc_control::MCController::run();
    }
  }
}

void ObserverbasedAdmittance::reset(
    const mc_control::ControllerResetData &reset_data) {
  mc_control::MCController::reset(reset_data);
}

CONTROLLER_CONSTRUCTOR("ObserverbasedAdmittance", ObserverbasedAdmittance)
