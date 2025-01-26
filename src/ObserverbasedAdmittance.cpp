#include "ObserverbasedAdmittance.h"

#include <SpaceVecAlg/EigenTypedef.h>

ObserverbasedAdmittance::ObserverbasedAdmittance(
    mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config)
    : mc_control::MCController(rm, dt),
      getestimatedContactWrench_(Eigen::Vector6d::Zero()),
      getestimatedExternalWrench_(Eigen::Vector6d::Zero()) {
  // Get values from DataStore
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

  // Endeffector Task
  efTask = std::make_shared<mc_tasks::EndEffectorTask>("l_wrist", robots(), 0,
                                                       10, 1000.0);
  solver().addTask(efTask);

  // contact surfaces
  addContact({robot().name(), "ground", "LeftFoot", "AllGround"});
  addContact({robot().name(), "ground", "RightFoot", "AllGround"});

  mc_rtc::log::success("ObserverbasedAdmittance init done ");
}

bool ObserverbasedAdmittance::run() {
  auto pt = efTask->get_ef_pose();
  efTask->set_ef_pose(
      sva::PTransformd{sva::RotY(-M_PI / 2), Eigen::Vector3d{0.5, -0.5, 1.2}});
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

  t_ += timeStep;

  return mc_control::MCController::run();
}

void ObserverbasedAdmittance::reset(
    const mc_control::ControllerResetData &reset_data) {
  efTask->reset();
  // addToGUI();
  mc_control::MCController::reset(reset_data);
}

void ObserverbasedAdmittance::addToGUI() {
  gui()->addPlot(
      "Contact Estimation", mc_rtc::gui::plot::X("t", [this]() { return t_; }),
      mc_rtc::gui::plot::Y(
          "Force X", [this]() { return getestimatedContactWrench_[0]; },
          mc_rtc::gui::Color::Red),
      mc_rtc::gui::plot::Y(
          "Force Y", [this]() { return getestimatedContactWrench_[1]; },
          mc_rtc::gui::Color::Green),
      mc_rtc::gui::plot::Y(
          "Force Z", [this]() { return getestimatedContactWrench_[2]; },
          mc_rtc::gui::Color::Blue));
}

CONTROLLER_CONSTRUCTOR("ObserverbasedAdmittance", ObserverbasedAdmittance)
