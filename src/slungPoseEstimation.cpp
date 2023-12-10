/**

 */

#include <kalmanif/kalmanif.h>

#include "utils_estimation/rand.h"
#include "utils_estimation/plots.h"
#include "utils_estimation/utils.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "slung_pose_estimation/datatypes.h"
#include "slung_pose_estimation/utils.h"

#include <manif/SE3.h>
#include <vector>

#include <iostream>
#include <fstream>
#include <sstream>

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
//using ERTS = RauchTungStriebelSmoother<EKF>;
//using URTSM = RauchTungStriebelSmoother<UKFM>;

int main(int argc, char* argv[]) {
  std::cout << "POSE ESTIMATION STARTED" <<"\n";

  KALMANIF_DEMO_PROCESS_INPUT(argc, argv);
  KALMANIF_DEMO_PRETTY_PRINT();

  // START CONFIGURATION
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("slung_pose_estimation");
  std::string data_file_path = package_share_directory + "/data/data_vnav_0.1.txt";
  std::string save_file_path = package_share_directory + "/data/estimation_out.txt";

  constexpr double eot = 15;                 //350 s
  constexpr double dt = 0.1;                 // 0.01;                 // s
  double sqrtdt = std::sqrt(dt);

  constexpr double var_gyro = 1e-4;           // (rad/s)^2
  constexpr double var_odometry = 9e-6;       // (m/s)^2

  constexpr int gps_freq = 10;                // Hz
  constexpr int landmark_freq = 50;           // Hz
  (void)landmark_freq;
  (void)gps_freq;

  State X_simulation = State::Identity(),
        X_unfiltered = State::Identity(); // propagation only, for comparison purposes

  // Load characteristics
  float mass_load = 0.225; // kg
  Eigen::Matrix3d j_load; 
  j_load << 2.1, 0, 0,
            0, 1.87, 0,
            0, 0, 3.97;
  
  // Vectors fom the COM of the load to the ith cable's attachment point
  std::vector<Eigen::Vector3d> r_cables;
  r_cables.push_back(Eigen::Vector3d(-0.42, -0.27, 0));
  r_cables.push_back(Eigen::Vector3d(0.48, -0.27, 0));
  r_cables.push_back(Eigen::Vector3d(-0.06, 0.55, 0));

  // Define a control vector and its noise and covariance
  Control u_simu,
  u_est, u_unfilt;

  Vector6d     u_nom, u_noisy, u_noise;
  Array6d      u_sigmas;
  Matrix6d     U;

  u_nom = Vector6d::Zero(); // TODO: Update these
  u_sigmas.head<3>().setConstant(std::sqrt(var_odometry));
  u_sigmas.tail<3>().setConstant(std::sqrt(var_gyro));
  U        = (u_sigmas * u_sigmas * 1./dt).matrix().asDiagonal();

  // Define the beacon's measurements
  Eigen::Vector3d y, y_noise;
  Eigen::Array3d  y_sigmas;
  Eigen::Matrix3d R;

  y_sigmas << 0.01, 0.01, 0.01;
  R        = (y_sigmas * y_sigmas).matrix().asDiagonal();

  std::vector<MeasurementModel> measurement_models = { // Initialize the drone positions (to be updated from real data) TODO: How add knowledge of drone orientations? PnP can estimate load orientation and have a model for this...
    MeasurementModel(Landmark(0.0,  0.0,  0.0), R),
    MeasurementModel(Landmark(0.0,  0.0,  0.0), R),
    MeasurementModel(Landmark(0.0,  0.0,  0.0), R)
  };

  std::vector<Measurement> measurements(measurement_models.size());

  // Define the gps measurements //TODO: Could incorportate these into slung load measurements to combine GPS measurements on load with visual measurements from drones
  // Eigen::Vector3d y_gps, y_gps_noise;
  // Eigen::Array3d  y_gps_sigmas;
  // Eigen::Matrix3d R_gps;

  // y_gps_sigmas << std::sqrt(6e-3), std::sqrt(6e-3), std::sqrt(6e-3);
  // R_gps        = (y_sigmas * y_sigmas).matrix().asDiagonal();

  SystemModel system_model;
  system_model.setCovariance(U);

  StateCovariance state_cov_init = StateCovariance::Identity();
  state_cov_init.bottomRightCorner<3,3>() *= MANIF_PI_4;

  Vector6d n = randn<Array6d>();
  Vector6d X_init_noise = state_cov_init.cwiseSqrt() * n;
  State X_init = X_simulation + State::Tangent(X_init_noise); // TODO: Update these???? Says starts at identity but my data doesn't necessarily...

  EKF ekf;
  ekf.setState(X_init);
  ekf.setCovariance(state_cov_init);

  UKFM ukfm(X_init, state_cov_init);
  //UKFM ukfm; 

  
  //ERTS erts(X_init, state_cov_init);

  //URTSM urtsm(X_init, state_cov_init);

  // Store some data for plots
  DemoDataCollector<State> collector;
  collector.reserve(
    eot/dt, "UNFI", "EKF", "UKFM" //, "ERTS", "URTSM"
  );

  // END CONFIGURATION


  // BEGIN TEMPORAL LOOP
  // Read testing data from file
  float t;
  StateData load_data;
  std::vector<StateData> tension_data;
  std::vector<StateData> drones_data;

  std::ifstream file(data_file_path);
  std::string line;

  if (!file.is_open()) {
      std::cerr << "Error opening file." << std::endl;
      return false;
  }

  // Make T steps. Measure up to K landmarks each time.
  //for (double t = 0; t < eot; t += dt) {
  int cnt_steps = 0;
  while (std::getline(file, line))
  {   
    // Reset data vectors
    tension_data.clear();
    drones_data.clear();
    
    //GET DATA
    std::istringstream iss(line);
    std::string segment;
    
    // Read time
    std::getline(iss, segment, ' ');
    t = std::stof(segment);

    // Read load state
    // x
    std::getline(iss, segment, ' ');
    load_data.x = utils::convert_vec_floats_to_eigen(utils::splitAndConvert(segment, ','));

    // x_dot. NORMALLY WOULD USE BWKD DIFFERENCE OF THETA TO GET. Using currently to decrease effect of numerical errors.
    std::getline(iss, segment, ' ');
    load_data.x_dot = utils::convert_vec_floats_to_eigen(utils::splitAndConvert(segment, ','));

    //x_ddot - Calculated below. COULD USE THIS DIRECTLY TO SIMULATE AN IMU MOUNTED ON THE LOAD
    std::getline(iss, segment, ' '); 
    
    // theta
    std::getline(iss, segment, ' ');
    load_data.theta = utils::convert_vec_floats_to_eigen(utils::splitAndConvert(segment, ','));

    // theta_dot. NORMALLY WOULD USE BWKD DIFFERENCE OF THETA TO GET. Using currently to decrease effect of numerical errors.
    std::getline(iss, segment, ' ');
    load_data.theta_dot = utils::convert_vec_floats_to_eigen(utils::splitAndConvert(segment, ','));

    // theta_ddot - Calculated below. COULD USE THIS DIRECTLY TO SIMULATE AN IMU MOUNTED ON THE LOAD
    std::getline(iss, segment, ' ');

    std::cout << "t: " << t << std::endl;
    //std::cout << "load_x : " << load_data.x[0] << " " << load_data.x[1] << " " << load_data.x[2] << std::endl;
    // std::cout << "load_theta : " << load_data.theta[0] << " " << load_data.theta[1] << " " << load_data.theta[2] << std::endl;
    //std::cout << "load_x_dot : " << load_data.x_dot[0] << " " << load_data.x_dot[1] << " " << load_data.x_dot[2] << std::endl;
    //std::cout << "load_theta_dot : " << load_data.theta_dot[0] << " " << load_data.theta_dot[1] << " " << load_data.theta_dot[2] << std::endl;

    // Read tension vectors
    for (int i = 0; i < 3; i++)
    {
        std::getline(iss, segment, ' ');
        StateData tension;
        tension.x = utils::convert_vec_floats_to_eigen(utils::splitAndConvert(segment, ','));
        tension_data.push_back(tension);

        //std::cout << "tension: " << tension_data[i].x[0] << " " << tension_data[i].x[1] << " " << tension_data[i].x[2] << std::endl;
    }

    // Read drone states
    for (int i = 0; i < 3; i++)
    {   
        // x
        std::getline(iss, segment, ' ');
        StateData drone;
        drone.x = utils::convert_vec_floats_to_eigen(utils::splitAndConvert(segment, ','));
        
        // x_dot and x_ddot
        std::getline(iss, segment, ' ');
        std::getline(iss, segment, ' ');

        // theta
        std::getline(iss, segment, ' ');
        drone.theta = utils::convert_vec_floats_to_eigen(utils::splitAndConvert(segment, ','));

        // theta_dot and theta_ddot
        std::getline(iss, segment, ' ');
        std::getline(iss, segment, ' ');

        // Store drone data
        drones_data.push_back(drone);
    }

    // Set initial state to data state if desired
    // if (cnt_steps == 0) {
    //   X_init = State::Identity();
    //   X_init.translation() = load_data.x;
    //   X_init.so3() = utils::convert_rvec_to_rotmat(load_data.theta);

    //   ekf.setState(X_init);
    //   ekf.setCovariance(state_cov_init);
    //   ukfm(X_init, state_cov_init);
    // } 
  

    // I. Simulation
    // Convert tension data to control inputs using accel equations integrated over time/ *dt
    // TODO: Use estimated rotation matrix R_L and angular velocity omega_L rather than actual data
    Eigen::Vector3d ge_3 = Eigen::Vector3d(0, 0, 9.81); 
    Eigen::Matrix3d R_L = utils::convert_rvec_to_rotmat(load_data.theta);
    Eigen::Vector3d Omega_L = load_data.theta_dot;

    // Linear acceleration of load
    Eigen::Vector3d x_ddot = -ge_3;
    Eigen::Vector3d ri_cross_tiqi_sum = Eigen::Vector3d(0, 0, 0);

    for(std::size_t i = 0; i < r_cables.size(); i++)
    {
      x_ddot -= (1/mass_load) * R_L*tension_data[i].x;
      ri_cross_tiqi_sum += r_cables[i].cross(-tension_data[i].x);
    }

    // Angular acceleration of load
    Eigen::Vector3d theta_ddot = j_load.inverse()*(ri_cross_tiqi_sum - Omega_L.cross(j_load*Omega_L));
     
    // Control inputs are the changes in state x as (x_next = x_prev + u)
    // Use kinematic equations with constant acceleration to calculate, then discretize
    Vector6d state_ddot, state_dot;


    state_ddot << x_ddot, theta_ddot;
    state_dot << load_data.x_dot, load_data.theta_dot; // Would normally get with numerical integration of state_ddot or bkwd finite difference

    u_nom = state_dot + 0.5 * state_ddot * dt; //state_dot * dt + 0.5 * state_ddot * std::pow(dt, 2.0); //u_nom    << 0.1, 0.0, 0.05, 0.0, 0, 0.05;

    /// simulate noise
    u_noise = randn<Array6d>(u_sigmas / sqrtdt); // control noise
    u_noisy = u_nom + u_noise;                   // noisy control

    u_simu   = u_nom   * dt;
    u_est    = u_noisy * dt;
    u_unfilt = u_noisy * dt;


    /// first we move - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    X_simulation = system_model(X_simulation, u_simu);
  
    /// then we measure all drones  - - - - - - - - - - - - - - - - - - - -
    for (std::size_t i = 0; i < measurement_models.size(); ++i)  {
      //std::cout << "Drone " << i << " position: " << drones_data[i].x[0] << " " << drones_data[i].x[1] << " " << drones_data[i].x[2] << "\n";
      
      // Update drone position based on latest data
      Landmark drone_pos = Landmark(drones_data[i].x[0], drones_data[i].x[1], drones_data[i].x[2]);
      measurement_models[i].setLandmark(drone_pos);

      // Make measurement
      auto measurement_model = measurement_models[i];

      y = measurement_model(X_simulation);      // landmark measurement, before adding noise. TODO: This gives drone rel load. OK? Would need to flip PnP estimate

      /// simulate noise
      y_noise = randn(y_sigmas);                // measurement noise

      y = y + y_noise;                          // landmark measurement, noisy
      measurements[i] = y;                      // store for the estimator just below
    }

    //// II. Estimation

    /// First we move

    ekf.propagate(system_model, u_est);

    ukfm.propagate(system_model, u_est);

    //erts.propagate(system_model, u_est);

    //urtsm.propagate(system_model, u_est);

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

      //erts.update(measurement_model, y);

      //urtsm.update(measurement_model, y);
    }
    // }

    // if (int(t*100) % int(100./gps_freq) == 0) {

      // gps measurement model
      //auto gps_measurement_model = DummyGPSMeasurementModel<State>(R_gps); 

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
    if(cnt_steps >= 20){ //20 Filter out the first results to allow for convergence - nicer for plotting
      collector.collect(X_simulation, t);

      collector.collect("UNFI", X_unfiltered, StateCovariance::Zero(), t);

      collector.collect("EKF",  ekf.getState(), ekf.getCovariance(), t);
      collector.collect("UKFM", ukfm.getState(), ukfm.getCovariance(), t);
    }
    
    cnt_steps++;
  }

  // END OF TEMPORAL LOOP. DONE.

  // Batch backward pass - smoothing
  // {
  //   erts.smooth();
  //   const auto& Xs_erts = erts.getStates();
  //   const auto& Ps_erts = erts.getCovariances();

  //   urtsm.smooth();
  //   const auto& Xs_urtsm = urtsm.getStates();
  //   const auto& Ps_urtsm = urtsm.getCovariances();

  //   // Collect smoothers' data
  //   double t=0;
  //   for (std::size_t i=0; i<Xs_erts.size(); ++i, t+=dt) {
  //     collector.collect("ERTS", Xs_erts[i], Ps_erts[i], t);
  //     collector.collect("URTSM", Xs_urtsm[i], Ps_urtsm[i], t);
  //   }
  // }

  // print the trajectory
  if (!quiet) {
    //KALMANIF_DEMO_PRINT_TRAJECTORY(collector);
    //KALMANIF_DEMO_SAVE_TRAJECTORY(collector, save_file_path, dt) 
  }

  // Generate some metrics and print them
  DemoDataProcessor<State>().process(collector).print();

  // // Actually plots only if PLOT_EXAMPLES=ON
  //std::string filename = "plot";
  DemoTrajPlotter<State>::plot(collector, filename, plot_trajectory); // Put in filename if want to save to file
  DemoDataPlotter<State>::plot(collector, filename, plot_error);



  return EXIT_SUCCESS;
}