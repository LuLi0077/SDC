#include "MPC.h"
#include <math.h>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

//**********************************************************************************************
// Describe the model in detail, includes the state, actuators and update equations:
//
// The state includes:
// * x, y: the position of the vehicle in the map coordinate
// * psi: the orientation of the vehicle
// * v: the velocity of the vehicle
// * cte: cross track error - the error between the center of the road and the vehicle's position
// * epsi: orientation error - current orientation error and the change in error caused by the vehicle's movement
//
// Actuators include:
// * delta: steering angle
// * a: throttle
//
// For simplicity, a kinematic model is used instead of dynamic model. 
// The update equations for the model are:
    // fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
    // fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
    // fg[2 + psi_start + i] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
    // fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
    // fg[2 + cte_start + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
    // fg[2 + epsi_start + i] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
//**********************************************************************************************
//
// Discuss the reasoning behind the chosen N and dt values:
//
// Tested N within [8, 25] and dt within [0.05, 0.1]. N determines the number of variables
// optimized and dt is the frequency between actuations. If N is too small, we don't have 
// enough data points; too large, it gets too far into the future where predictions are 
// less certain. Similarly, if dt is too large, we can't approximate a continuous reference 
// trajectory; too small, the changes are minimal and updates are unnecessary. Here, I 
// choosed the minimum N and maximum dt possible for car to stay on track.
//**********************************************************************************************
//
// Provides details on how MPC handles a 100 millisecond latency:
//
// Account for latency is implemented in `main.cpp` (as below) simply by projecting current
// state values one latency timestep ahead:    
    // const double latency = .1 ;
    // px = v * latency;
    // psi = -v * steer_angle * latency / 2.67;

    // double cte = polyeval(coeffs, px);
    // double epsi = -atan(coeffs[1] + 2 * px * coeffs[2] + 3 * px * px * coeffs[3]);

    // state << px, 0, psi, v, cte, epsi;
    // auto move =  mpc.Solve(state, coeffs);
//**********************************************************************************************


using CppAD::AD;

// Set the timestep length and duration
size_t N = 8;
double dt = 0.08;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// Both the reference cross track and orientation errors are 0.
// The reference velocity is set to 65 mph.
double ref_cte = 0;
double ref_epsi = 0;
double ref_v = 65;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:

  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {

    // fg a vector of constraints, x is a vector of constraints.
    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // The part of the cost based on the reference state.
    for (int i = 0; i < N; i++) {
      fg[0] += CppAD::pow(vars[cte_start + i] - ref_cte, 2);
      fg[0] += CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
      fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (int i = 0; i < N - 1; i++) {
      fg[0] += CppAD::pow(vars[delta_start + i], 2);
      fg[0] += CppAD::pow(vars[a_start + i], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int i = 0; i < N - 2; i++) {
      // tuning the steering value in the cost function results in a smoother steering transitions
      fg[0] += 500*CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
      fg[0] += CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
      //fg[0] += CppAD::pow(vars[cte_start + i + 1] - vars[cte_start+i], 2);
    }

    //
    // Setup Constraints
    //

    // We add 1 to each of the starting indices due to cost being located at index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (int i = 0; i < N - 1; i++) {
      // The state at time t.
      AD<double> x0 = vars[x_start + i];
      AD<double> y0 = vars[y_start + i];
      AD<double> psi0 = vars[psi_start + i];
      AD<double> v0 = vars[v_start + i];
      AD<double> cte0 = vars[cte_start + i];
      AD<double> epsi0 = vars[epsi_start + i];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + i];
      AD<double> a0 = vars[a_start + i];
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0;
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2.0 * coeffs[2] * x0 + 3.0 * coeffs[3] * x0 * x0);

      // The state at time t+1 .
      AD<double> x1 = vars[x_start + i + 1];
      AD<double> y1 = vars[y_start + i + 1];
      AD<double> psi1 = vars[psi_start + i + 1];
      AD<double> v1 = vars[v_start + i + 1];
      AD<double> cte1 = vars[cte_start + i + 1];
      AD<double> epsi1 = vars[epsi_start + i + 1];

      // The equations for the model:
      fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[2 + psi_start + i] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
      fg[2 + cte_start + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[2 + epsi_start + i] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  }
};


//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Set the number of model variables (includes both states and inputs).
    // the position of the vehicle in the map coordinate
  double x = state[0];
  double y = state[1];
    // the orientation of the vehicle
  double psi = state[2];
    // the velocity of the vehicle
  double v = state[3];
    // (cross track error) the error between the center of the road and the vehicle's position
  double cte = state[4];
    // (orientation error) current orientation error and the change in error caused by the vehicle's movement
  double epsi = state[5];

  // Set the number of variables
  size_t n_vars = N * 6 + (N - 1) * 2;

  // Set the number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }

  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  // Set lower and upper limits for variables to the max negative and positive values.
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25 degrees (values in radians).
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -0.5;
    vars_upperbound[i] = 0.5;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // options for IPOPT solver
  std::string options;

  options += "Integer print_level  0\n";
  // Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. 
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // Currently the solver has a maximum time limit of 0.5 seconds.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  //std::cout << "Cost " << cost << std::endl;

  vector<double> out;
  out.push_back(N);
  out.push_back(solution.x[delta_start]);
  out.push_back(solution.x[a_start]);
  for(int i = x_start; i < y_start; i++){
    out.push_back(solution.x[i]);
  }
  for(int i = y_start; i < psi_start; i++){
    out.push_back(solution.x[i]);
  }

  return out;
}
