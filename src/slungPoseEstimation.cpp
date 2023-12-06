/**

 */

#include <kalmanif/kalmanif.h>

#include "utils_estimation/rand.h"
#include "utils_estimation/plots.h"
#include "utils_estimation/utils.h"

#include <manif/SE3.h>

#include <iostream>

#include <vector>

using namespace kalmanif;
using namespace manif;

using State = SE3d;
using StateCovariance = Covariance<State>;
using SystemModel = LieSystemModel<State>;
using Control = SystemModel::Control;
using MeasurementModel = Landmark3DMeasurementModel<State>; // TODO: CHANGE FROM LANDMARK???
using Landmark = MeasurementModel::Landmark;                // TODO: CHANGE FROM LANDMARK???
using Measurement = MeasurementModel::Measurement;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Array6d = Eigen::Array<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

// Filters
using EKF = ExtendedKalmanFilter<State>;
using UKFM = UnscentedKalmanFilterManifolds<State>;

// Smoothers
using ERTS = RauchTungStriebelSmoother<EKF>;
using URTSM = RauchTungStriebelSmoother<UKFM>;

int main(){ //int argc, char* argv[]) {
  std::cout << "WORKING" <<"\n";

  //KALMANIF_DEMO_PROCESS_INPUT(argc, argv);
  //KALMANIF_DEMO_PRETTY_PRINT();

  std::cout << "AFTER PRINT" <<"\n";

  // START CONFIGURATION

  constexpr double eot = 15;                 //350 s
  constexpr double dt = 0.01;                 // s
  double sqrtdt = std::sqrt(dt);

  constexpr double var_gyro = 1e-4;           // (rad/s)^2
  constexpr double var_odometry = 9e-6;       // (m/s)^2

  constexpr int gps_freq = 10;                // Hz
  constexpr int landmark_freq = 50;           // Hz
  (void)landmark_freq;
  (void)gps_freq;

  State X_simulation = State::Identity(),
        X_unfiltered = State::Identity(); // propagation only, for comparison purposes

  // Define a control vector and its noise and covariance
  Control      u_simu, u_est, u_unfilt;

  Vector6d     u_nom, u_noisy, u_noise;
  Array6d      u_sigmas;
  Matrix6d     U;

  u_nom    << 0.1, 0.0, 0.05, 0.0, 0, 0.05;
  u_sigmas.head<3>().setConstant(std::sqrt(var_odometry));
  u_sigmas.tail<3>().setConstant(std::sqrt(var_gyro));
  U        = (u_sigmas * u_sigmas * 1./dt).matrix().asDiagonal();

  // Define the beacon's measurements
  Eigen::Vector3d y, y_noise;
  Eigen::Array3d  y_sigmas;
  Eigen::Matrix3d R;

  y_sigmas << 0.01, 0.01, 0.01;
  R        = (y_sigmas * y_sigmas).matrix().asDiagonal();

  std::vector<MeasurementModel> measurement_models = {
    MeasurementModel(Landmark(2.0,  0.0,  0.0), R),
    MeasurementModel(Landmark(3.0, -1.0, -1.0), R),
    MeasurementModel(Landmark(2.0, -1.0,  1.0), R),
    MeasurementModel(Landmark(2.0,  1.0,  1.0), R),
    MeasurementModel(Landmark(2.0,  1.0, -1.0), R)
  };

  std::vector<Measurement> measurements(measurement_models.size());

  // Define the gps measurements
  Eigen::Vector3d y_gps, y_gps_noise;
  Eigen::Array3d  y_gps_sigmas;
  Eigen::Matrix3d R_gps;

  y_gps_sigmas << std::sqrt(6e-3), std::sqrt(6e-3), std::sqrt(6e-3);
  R_gps        = (y_sigmas * y_sigmas).matrix().asDiagonal();

  SystemModel system_model;
  system_model.setCovariance(U);

  StateCovariance state_cov_init = StateCovariance::Identity();
  state_cov_init.bottomRightCorner<3,3>() *= MANIF_PI_4;

  Vector6d n = randn<Array6d>();
  Vector6d X_init_noise = state_cov_init.cwiseSqrt() * n;
  State X_init = X_simulation + State::Tangent(X_init_noise);

  EKF ekf;
  ekf.setState(X_init);
  ekf.setCovariance(state_cov_init);

  UKFM ukfm(X_init, state_cov_init);

  ERTS erts(X_init, state_cov_init);

  URTSM urtsm(X_init, state_cov_init);

  // Store some data for plots
  DemoDataCollector<State> collector;
  collector.reserve(
    eot/dt, "UNFI", "EKF", "UKFM", "ERTS", "URTSM"
  );

  // Make T steps. Measure up to K landmarks each time.
  for (double t = 0; t < eot; t += dt) {
    // I. Simulation

    /// simulate noise
    u_noise = randn<Array6d>(u_sigmas / sqrtdt); // control noise
    u_noisy = u_nom + u_noise;                   // noisy control

    u_simu   = u_nom   * dt;
    u_est    = u_noisy * dt;
    u_unfilt = u_noisy * dt;

    /// first we move - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    X_simulation = system_model(X_simulation, u_simu);

    /// then we measure all landmarks - - - - - - - - - - - - - - - - - - - -
    for (std::size_t i = 0; i < measurement_models.size(); ++i)  {
      auto measurement_model = measurement_models[i];

      y = measurement_model(X_simulation);      // landmark measurement, before adding noise

      /// simulate noise
      y_noise = randn(y_sigmas);                // measurement noise

      y = y + y_noise;                          // landmark measurement, noisy
      measurements[i] = y;                      // store for the estimator just below
    }

    //// II. Estimation

    /// First we move

    ekf.propagate(system_model, u_est);

    ukfm.propagate(system_model, u_est);

    erts.propagate(system_model, u_est);

    urtsm.propagate(system_model, u_est);

    X_unfiltered = system_model(X_unfiltered, u_unfilt);

    /// Then we correct using the measurements of each lmk

    // if (int(t*100) % int(100./landmark_freq) == 0) {
      for (std::size_t i = 0; i < measurement_models.size(); ++i)  {
        // landmark
        auto measurement_model = measurement_models[i];

        // measurement
        y = measurements[i];

        // filter update
        ekf.update(measurement_model, y);

        ukfm.update(measurement_model, y);

        erts.update(measurement_model, y);

        urtsm.update(measurement_model, y);
      }
    // }

    // if (int(t*100) % int(100./gps_freq) == 0) {

      // gps measurement model
      // auto gps_measurement_model = DummyGPSMeasurementModel<State>(R_gps);

      // y_gps = gps_measurement_model(X_simulation);                  // gps measurement, before adding noise

      // /// simulate noise
      // y_gps_noise = randn(y_gps_sigmas);                            // measurement noise
      // y_gps = y_gps + y_gps_noise;                                  // gps measurement, noisy

      // // filter update
      // ekf.update(gps_measurement_model, y_gps);

      // ukfm.update(gps_measurement_model, y_gps);

      // erts.update(gps_measurement_model, y_gps);

      // urtsm.update(gps_measurement_model, y_gps);
    // }

    //// III. Results

    collector.collect(X_simulation, t);

    collector.collect("UNFI", X_unfiltered, StateCovariance::Zero(), t);

    collector.collect("EKF",  ekf.getState(), ekf.getCovariance(), t);
    collector.collect("UKFM", ukfm.getState(), ukfm.getCovariance(), t);
  }

  // END OF TEMPORAL LOOP. DONE.

  // Batch backward pass - smoothing
  {
    erts.smooth();
    const auto& Xs_erts = erts.getStates();
    const auto& Ps_erts = erts.getCovariances();

    urtsm.smooth();
    const auto& Xs_urtsm = urtsm.getStates();
    const auto& Ps_urtsm = urtsm.getCovariances();

    // Collect smoothers' data
    double t=0;
    for (std::size_t i=0; i<Xs_erts.size(); ++i, t+=dt) {
      collector.collect("ERTS", Xs_erts[i], Ps_erts[i], t);
      collector.collect("URTSM", Xs_urtsm[i], Ps_urtsm[i], t);
    }
  }

  std::cout << "END" <<"\n";

  // print the trajectory
  // if (!quiet) {
  //   KALMANIF_DEMO_PRINT_TRAJECTORY(collector);
  // }

  // Generate some metrics and print them
  // DemoDataProcessor<State>().process(collector).print();

  // // Actually plots only if PLOT_EXAMPLES=ON
  // DemoTrajPlotter<State>::plot(collector, filename, plot_trajectory);
  // DemoDataPlotter<State>::plot(collector, filename, plot_error);

  return EXIT_SUCCESS;
}