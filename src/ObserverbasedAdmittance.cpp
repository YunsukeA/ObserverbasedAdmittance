#include "ObserverbasedAdmittance.h"

#include <SpaceVecAlg/EigenTypedef.h>

#include <SpaceVecAlg/SpaceVecAlg>
#include <string>

ObserverbasedAdmittance::ObserverbasedAdmittance(
    mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config)
    : mc_control::MCController(rm, dt),
      estimatedExternalWrench_centroid(sva::ForceVecd::Zero()),
      estimatedContactWrench_0_surface(sva::ForceVecd::Zero()),
      estimatedContactWrench_1_surface(sva::ForceVecd::Zero()),
      estimatedContactWrench_2_surface(sva::ForceVecd::Zero()),
      estimatedContactWrench_3_surface(sva::ForceVecd::Zero()) {
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

  // contact surfaces
  addContact({robot().name(), "ground", "LeftFoot", "AllGround"});
  addContact({robot().name(), "ground", "RightFoot", "AllGround"});

  mc_rtc::log::success("ObserverbasedAdmittance init done ");
}

bool ObserverbasedAdmittance::run() {
  getestimatedContactWrench();
  getestimatedExternalWrench();
  getForcesensorWrench();

  transformEstimatedWrenchs();
  transformEstimatedWrenchs2();

  t_ += timeStep;
  // addToGUI();

  return mc_control::MCController::run();
}

void ObserverbasedAdmittance::reset(
    const mc_control::ControllerResetData &reset_data) {
  addToGUI();
  addTologger();
  mc_control::MCController::reset(reset_data);
}

void ObserverbasedAdmittance::getestimatedContactWrench() {
  if (exportExternalWrench_) {
    if (datastore().has(robot_ + "::estimatedExternalWrench")) {
      estimatedExternalWrench_centroid =
          datastore().get<sva::ForceVecd>(robot_ + "::estimatedExternalWrench");
    }
    // mc_rtc::log::info("[DATASTORE]Exporting estimatedExternalWrench: \n {}",
    //                   estimatedExternalWrench_);
  }

  return;
}

void ObserverbasedAdmittance::getestimatedExternalWrench() {
  if (exportContactWrench_) {
    for (int i = 0; i < MaxContacts; i++) {
      if (datastore().has(robot_ + "::estimatedContactWrench_" +
                          std::to_string(i))) {
        switch (i) {
          case 0:
            estimatedContactWrench_0_surface = datastore().get<sva::ForceVecd>(
                robot_ + "::estimatedContactWrench_" + std::to_string(i));
            // mc_rtc::log::info(
            //     "[DATASTORE]Exporting estimatedContactWrench_{}: \n {}", i,
            //     estimatedContactWrench_0);
            break;
          case 1:
            estimatedContactWrench_1_surface = datastore().get<sva::ForceVecd>(
                robot_ + "::estimatedContactWrench_" + std::to_string(i));
            // mc_rtc::log::info(
            //     "[DATASTORE]Exporting estimatedContactWrench_{}: \n {}", i,
            //     estimatedContactWrench_1);
            break;
          case 2:
            estimatedContactWrench_2_surface = datastore().get<sva::ForceVecd>(
                robot_ + "::estimatedContactWrench_" + std::to_string(i));
            // mc_rtc::log::info(
            //     "[DATASTORE]Exporting estimatedContactWrench_{}: \n {}", i,
            //     estimatedContactWrench_2);
            break;
          case 3:
            estimatedContactWrench_3_surface = datastore().get<sva::ForceVecd>(
                robot_ + "::estimatedContactWrench_" + std::to_string(i));
            // mc_rtc::log::info(
            //     "[DATASTORE]Exporting estimatedContactWrench_{}: \n {}", i,
            //     estimatedContactWrench_3);
            break;
        }
      }
    }
  }
  return;
}

void ObserverbasedAdmittance::getForcesensorWrench() {
  forcesensor_0 = robot().forceSensor("RightFootForceSensor").wrench();
  forcesensor_1 = robot().forceSensor("LeftFootForceSensor").wrench();
  forcesensor_2 = robot().forceSensor("RightHandForceSensor").wrench();
  forcesensor_3 = robot().forceSensor("LeftHandForceSensor").wrench();

  return;
}

sva::ForceVecd
ObserverbasedAdmittance::wrenchTransformation(  // surface frame to sensor frame
    const sva::ForceVecd wrench, const std::string surface,
    const std::string forceSensor) {
  sva::PTransformd X_0_surface = robot().frame(surface).position();
  std::cout << "Surface Frame Position: \n" << X_0_surface << "\n" << std::endl;

  sva::PTransformd X_0_ft = robot().forceSensor(forceSensor).X_0_f(robot());
  std::cout << "Force Sensor Frame Position: \n" << X_0_ft << "\n" << std::endl;

  sva::PTransformd X_surface_ft = X_0_ft * X_0_surface.inv();
  std::cout << "Pose: \n" << X_surface_ft << "\n" << std::endl;

  sva::ForceVecd wrench_out = X_surface_ft.dualMul(wrench);
  std::cout << "Transformed Wrench: \n"
            << wrench_out.force().transpose() << " "
            << wrench_out.couple().transpose() << "\n"
            << std::endl;

  return wrench_out;
}

sva::ForceVecd
ObserverbasedAdmittance::wrenchTransformation2(  // sensor frame to surface
                                                 // frame
    const sva::ForceVecd wrench, const std::string surface,
    const std::string forceSensor) {
  sva::PTransformd X_0_surface = robot().frame(surface).position();
  // std::cout << "Surface Frame Position: \n" << X_0_surface << "\n" <<
  // std::endl;

  sva::PTransformd X_0_ft = robot().forceSensor(forceSensor).X_0_f(robot());
  // std::cout << "Force Sensor Frame Position: \n" << X_0_ft << "\n" <<
  // std::endl;

  sva::PTransformd X_ft_surface = X_0_surface * X_0_ft.inv();
  // std::cout << "Pose: \n" << X_ft_surface << "\n" << std::endl;

  sva::ForceVecd wrench_out = X_ft_surface.dualMul(wrench);
  // std::cout << "Transformed Wrench: \n"
  //           << wrench_out.force().transpose() << " "
  //           << wrench_out.couple().transpose() << "\n"
  //           << std::endl;

  return wrench_out;
}

void ObserverbasedAdmittance::transformEstimatedWrenchs() {
  std::cout << "Transforming estimated contact wrenches..." << std::endl;
  estimatedContactWrench_0_sensor = wrenchTransformation(
      estimatedContactWrench_0_surface, "LeftFoot", "LeftFootForceSensor");
  std::cout << "Estimated Contact Wrench 0 (LeftFoot): "
            << estimatedContactWrench_0_sensor.force().transpose() << " "
            << estimatedContactWrench_0_sensor.couple().transpose()
            << std::endl;

  estimatedContactWrench_1_sensor = wrenchTransformation(
      estimatedContactWrench_1_surface, "RightFoot", "RightFootForceSensor");
  std::cout << "Estimated Contact Wrench 1 (RightFoot): "
            << estimatedContactWrench_1_sensor.force().transpose() << " "
            << estimatedContactWrench_1_sensor.couple().transpose()
            << std::endl;
  std::cout << " " << std::endl;

  estimatedContactWrench_2_sensor = wrenchTransformation(
      estimatedContactWrench_2_surface, "LeftGripper", "LeftHandForceSensor");
  std::cout << "Estimated Contact Wrench 2 (LeftGripper): "
            << estimatedContactWrench_2_sensor.force().transpose() << " "
            << estimatedContactWrench_2_sensor.couple().transpose()
            << std::endl;
  std::cout << " " << std::endl;

  estimatedContactWrench_3_sensor = wrenchTransformation(
      estimatedContactWrench_3_surface, "RightGripper", "RightHandForceSensor");
  std::cout << "Estimated Contact Wrench 3 (RightGripper): "
            << estimatedContactWrench_3_sensor.force().transpose() << " "
            << estimatedContactWrench_3_sensor.couple().transpose()
            << std::endl;
  std::cout << " " << std::endl;
}

void ObserverbasedAdmittance::transformEstimatedWrenchs2() {
  forcesensor_0_surface =
      wrenchTransformation2(forcesensor_0, "RightFoot", "RightFootForceSensor");

  forcesensor_1_surface =
      wrenchTransformation2(forcesensor_1, "LeftFoot", "LeftFootForceSensor");

  forcesensor_3_surface = wrenchTransformation2(forcesensor_3, "LeftGripper",
                                                "LeftHandForceSensor");

  forcesensor_2_surface = wrenchTransformation2(forcesensor_2, "RightGripper",
                                                "RightHandForceSensor");
}

/////////////////////////////////////////////////
////////////////logging part/////////////////////
/////////////////////////////////////////////////

void ObserverbasedAdmittance::addToGUI() {
  gui()->addPlot(
      "Contact_0 Estimation",
      mc_rtc::gui::plot::X("t", [this]() { return t_; }),
      mc_rtc::gui::plot::Y(
          "Force X",
          [this]() { return estimatedContactWrench_0_surface.force().x(); },
          mc_rtc::gui::Color::Red),
      mc_rtc::gui::plot::Y(
          "Force Y",
          [this]() { return estimatedContactWrench_0_surface.force().y(); },
          mc_rtc::gui::Color::Green),
      mc_rtc::gui::plot::Y(
          "Force Z",
          [this]() { return estimatedContactWrench_0_surface.force().z(); },
          mc_rtc::gui::Color::Blue));
}

void ObserverbasedAdmittance::addTologger() {
  std::string category = "ObserverbasedAdmittance_";
  std::string subcategory_est = "estimatedContactWrench_";
  std::string subcategory_force = "forcesensor_";

  logger().addLogEntry(category + "estimatedExternalWrench" + "_raw",
                       [this]() { return estimatedExternalWrench_centroid; });

  logger().addLogEntry(category + subcategory_est + "transformed_0",
                       [this]() { return estimatedContactWrench_0_sensor; });
  logger().addLogEntry(category + subcategory_est + "transformed_1",
                       [this]() { return estimatedContactWrench_1_sensor; });
  logger().addLogEntry(category + subcategory_est + "transformed_2",
                       [this]() { return estimatedContactWrench_2_sensor; });
  logger().addLogEntry(category + subcategory_est + "transformed_3",
                       [this]() { return estimatedContactWrench_3_sensor; });

  logger().addLogEntry(category + subcategory_est + "raw_0",
                       [this]() { return estimatedContactWrench_0_surface; });
  logger().addLogEntry(category + subcategory_est + "raw_1",
                       [this]() { return estimatedContactWrench_1_surface; });
  logger().addLogEntry(category + subcategory_est + "raw_2",
                       [this]() { return estimatedContactWrench_2_surface; });
  logger().addLogEntry(category + subcategory_est + "raw_3",
                       [this]() { return estimatedContactWrench_3_surface; });

  logger().addLogEntry(category + subcategory_force + "transformed_0",
                       [this]() { return forcesensor_0_surface; });
  logger().addLogEntry(category + subcategory_force + "transformed_1",
                       [this]() { return forcesensor_1_surface; });
  logger().addLogEntry(category + subcategory_force + "transformed_2",
                       [this]() { return forcesensor_2_surface; });
  logger().addLogEntry(category + subcategory_force + "transformed_3",
                       [this]() { return forcesensor_3_surface; });
}

CONTROLLER_CONSTRUCTOR("ObserverbasedAdmittance", ObserverbasedAdmittance)