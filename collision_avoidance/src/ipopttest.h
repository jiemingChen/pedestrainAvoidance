//
// Created by jieming on 09.07.20.
//

#ifndef COLLISION_AVOIDANCE_IPOPTTEST_H
#define COLLISION_AVOIDANCE_IPOPTTEST_H

#include <vector>
#include "Eigen/Core"
using namespace std;

class ipopttest {
public:
    ipopttest();

    virtual ~ipopttest();

    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuations.
    std::vector<float> Solve(const vector<float> &state, const pair<vector<vector<float>>,vector<vector<float>>>& coeffs);

    std::vector<float> Solve_constr(const vector<float> &state, const pair<vector<vector<float>>,vector<vector<float>>> &coeffs);
};



#endif //COLLISION_AVOIDANCE_IPOPTTEST_H
