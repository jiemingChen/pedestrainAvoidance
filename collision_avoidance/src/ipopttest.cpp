//
// Created by jieming on 09.07.20.
//

#include "ipopttest.h"

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen/Core"

using CppAD::AD;
using Eigen::VectorXd;
using namespace std;
/**
 * TODO: Set the timestep length and duration
 */
int N = 12;
const double dt = 0.5;
const double Lf = 0.56;

int x_start = 0;
int y_start = x_start + N;
int psi_start = y_start + N;
int v_start = psi_start + N;
int delta_start = v_start +N-1;

int cons_collision_start = psi_start+N;
double ref_x = 10;
double ref_y = -8;
double ref_psi = atan2(ref_y, ref_x);

class FG_eval {
public:
    // Fitted polynomial coefficients
    pair<vector<vector<float>>,vector<vector<float>>> obs;
    FG_eval(pair<vector<vector<float>>,vector<vector<float>>> obs) {this->obs = obs;}

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    void operator()(ADvector& fg, const ADvector& vars) {

         fg[0] = 0.0;
         vector<float> P = {1, 1, 0};
         vector<float> Q = {1.5, 1.5, 0};
         vector<float> R = {0.1, 0.1, 0.8, 0.8};
         // The part of the cost based on the reference state.
         for(auto t=0; t<N-1; t++){
             fg[0]+= P[0]* CppAD::pow(vars[x_start+t]-ref_x, 2);
             fg[0]+= P[1]* CppAD::pow(vars[y_start+t]-ref_y, 2);
             fg[0]+= P[2]* CppAD::pow(vars[psi_start+t]-ref_psi, 2);
         }
         fg[0] += Q[0]*CppAD::pow(vars[x_start+N-1]-ref_x, 2) + Q[1]* CppAD::pow(vars[y_start+N-1]-ref_y, 2)
                 + Q[2]*CppAD::pow(vars[psi_start+N-1]-ref_psi, 2);
         // Minimize the use of actuators. Costs for Steering Angle and Acceleration
         for (unsigned int t = 0; t < N - 1; t++) {
            fg[0] += R[0] * CppAD::pow(vars[v_start + t], 2);
            fg[0] += R[1] * CppAD::pow(vars[delta_start + t], 2);
         }
         // Minimize the value gap between sequential actuations.
         for (unsigned int t = 0; t < N - 2; t++) {
            fg[0] += R[2] * CppAD::pow(vars[v_start + t + 1] - vars[v_start + t], 2);
            fg[0] += R[3] * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
         }
         // avoidance cost
#if 0 //cost method
         for(auto t=0; t<N-1; t++){
             for(auto j=0; j<obs.first.size(); j++){
                fg[0]+= 200*vars[v_start+t]/(CppAD::pow(vars[x_start+t]-obs.first[j][0], 2)+CppAD::pow(vars[y_start+t]-obs.first[j][1], 2)+0.0000001);
             }
        }
#endif
         // Initial  constraints
         fg[1+x_start] = vars[x_start];
         fg[1+y_start] = vars[y_start];
         fg[1+psi_start] = vars[psi_start];

         // kinematic constraints
         for(auto t=1; t<N; t++){
             AD<double> x1 = vars[x_start+t];
             AD<double> y1 = vars[y_start+t];
             AD<double> psi1 = vars[psi_start+t];

             AD<double> x0 = vars[x_start+t-1];
             AD<double> y0 = vars[y_start+t-1];
             AD<double> psi0 = vars[psi_start+t-1];

             AD<double> v0 = vars[v_start+t-1];
             AD<double> delta0 = vars[delta_start+t-1];

             fg[1+x_start+t] = x1 - (x0 + v0*CppAD::cos(psi0)*dt);
             fg[1+y_start+t] = y1 - (y0 + v0*CppAD::sin(psi0)*dt);
             fg[1+psi_start+t] = psi1 - (psi0 + v0/Lf*CppAD::tan(delta0)*dt);
         }

         //collision constraints
#if 1
        for(auto t=0; t<N; t++) {
            for(auto j=0; j<obs.first.size(); j++){
                fg[1+cons_collision_start+t*obs.first.size() + j] = CppAD::pow(vars[x_start+t]-obs.first[j][0], 2)+CppAD::pow(vars[y_start+t]-obs.first[j][1], 2);
//                cout << "woqu "<< 1+cons_collision_start+t*obs.first.size() + j <<  endl;
            }
        }
#endif
    }
};

//
// MPC class definition implementation.
//
ipopttest::ipopttest() {}
ipopttest::~ipopttest() {}

std::vector<float> ipopttest::Solve(const vector<float> &state, const pair<vector<vector<float>>,vector<vector<float>>> &coeffs) {
    bool ok = true;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    size_t n_vars = N*3+(N-1)*2;
    size_t n_constraints = (N-1)*3+3;

    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; ++i) {
        vars[i] = 0;
    }

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    for(auto i=x_start; i<y_start; i++){
        vars_lowerbound[i] = -300;
        vars_upperbound[i] = 300;
    }
    for(auto i=y_start; i<psi_start; i++){
        vars_lowerbound[i] = -300;
        vars_upperbound[i] = 300;
    }
    for(auto i=psi_start; i<v_start; i++){
        vars_lowerbound[i] = -6.28;
        vars_upperbound[i] = 6.28;
    }
    for(auto i=v_start; i<delta_start; i++){
        vars_lowerbound[i] = 0.6;
        vars_upperbound[i] = 1.0;
    }
    for(auto i=delta_start; i<n_vars; i++){
        vars_lowerbound[i] = -3.14159/4;
        vars_upperbound[i] = 3.14159/4;
    }

    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; ++i) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    constraints_lowerbound[x_start] = state[0];
    constraints_lowerbound[y_start] = state[1];
    constraints_lowerbound[psi_start] = state[2];
    constraints_upperbound[x_start] = state[0];
    constraints_upperbound[y_start] = state[1];
    constraints_upperbound[psi_start] = state[2];
    // object that computes objective and constraints
    FG_eval fg_eval(coeffs);

    std::string options;
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    //   of sparse routines, this makes the computation MUCH FASTER. If you can
    //   uncomment 1 of these and see if it makes a difference or not but if you
    //   uncomment both the computation time should go up in orders of magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
            options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
            constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    if(!ok){
        cout << solution.status <<"   solve problem";
        exit(0);
    }
    // Cost
    auto cost = solution.obj_value;
    std::cout << "Cost " << cost << std::endl;

    vector<float> values_done(2+N*3);
    values_done[0] = solution.x[v_start];
    values_done[1] = solution.x[delta_start];
    for(auto i=0; i<N; i++){
        values_done[2+i*3] = solution.x[x_start+i];
        values_done[3+i*3] = solution.x[y_start+i];
        values_done[4+i*3] = solution.x[psi_start+i];
    }
    return values_done;
}




std::vector<float> ipopttest::Solve_constr(const vector<float> &state, const pair<vector<vector<float>>,vector<vector<float>>> &coeffs) {
    bool ok = true;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    size_t n_vars = N*3+(N-1)*2;
    size_t n_constraints = (N-1)*3+3 + N*coeffs.first.size();

    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; ++i) {
        vars[i] = 0;
    }

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    for(auto i=x_start; i<y_start; i++){
        vars_lowerbound[i] = -300;
        vars_upperbound[i] = 300;
    }
    for(auto i=y_start; i<psi_start; i++){
        vars_lowerbound[i] = -300;
        vars_upperbound[i] = 300;
    }
    for(auto i=psi_start; i<v_start; i++){
        vars_lowerbound[i] = -6.28;
        vars_upperbound[i] = 6.28;
    }
    for(auto i=v_start; i<delta_start; i++){
        vars_lowerbound[i] = 0.6;
        vars_upperbound[i] = 1.0;
    }
    for(auto i=delta_start; i<n_vars; i++){
        vars_lowerbound[i] = -3.14159/4;
        vars_upperbound[i] = 3.14159/4;
    }

    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < cons_collision_start; ++i) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    constraints_lowerbound[x_start] = state[0];
    constraints_lowerbound[y_start] = state[1];
    constraints_lowerbound[psi_start] = state[2];
    constraints_upperbound[x_start] = state[0];
    constraints_upperbound[y_start] = state[1];
    constraints_upperbound[psi_start] = state[2];

    for(auto i=cons_collision_start; i<n_constraints; i++){
        constraints_lowerbound[i] = 0.8*0.8;
        constraints_upperbound[i] = INT_MAX;
    }

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs);

    std::string options;
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    //   of sparse routines, this makes the computation MUCH FASTER. If you can
    //   uncomment 1 of these and see if it makes a difference or not but if you
    //   uncomment both the computation time should go up in orders of magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
            options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
            constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    if(!ok){
        cout << solution.status <<"   solve problem";
        exit(0);
    }
    // Cost
    auto cost = solution.obj_value;
    std::cout << "Cost " << cost << std::endl;

    vector<float> values_done(2+N*3);
    values_done[0] = solution.x[v_start];
    values_done[1] = solution.x[delta_start];
    for(auto i=0; i<N; i++){
        values_done[2+i*3] = solution.x[x_start+i];
        values_done[3+i*3] = solution.x[y_start+i];
        values_done[4+i*3] = solution.x[psi_start+i];
    }
    return values_done;
}