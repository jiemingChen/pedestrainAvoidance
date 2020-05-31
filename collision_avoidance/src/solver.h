//
// Created by jieming on 28.04.20.
//

#ifndef COLLISION_AVOIDANCE_SOLVER_H
#define COLLISION_AVOIDANCE_SOLVER_H

#include  "collision_avoidance/header.h"
// Magic tricks to have CPLEX behave well:
#ifndef IL_STD
#define IL_STD
#endif
#include <cstring>
#include <ilcplex/ilocplex.h>
ILOSTLBEGIN
// End magic tricks

class Solver {
private:

    int N_;
    vector<float> Q;
    vector<float> R;
    vector<float> P;
    std::chrono::high_resolution_clock::time_point previous_timestamp_;

    IloEnv env_;

    IloArray<IloNumVarArray> x_;
    IloArray<IloNumVarArray> u_;
    IloNumVarArray slack_;
    float safe_dist_;
    float dt_;

public:
    Solver();
    ~Solver();
    vector<float> solve(std::pair<vector<double>, vector<float>> job_point, vector<double>target_point, vector<float>current_state, std::pair<std::vector<std::vector<float>>, std::vector<std::vector<float>>> );
    // consider obstacles speed
    vector<float> solve2(std::pair<vector<double>, vector<float>> job_point, vector<double>target_point, vector<float>current_state, std::pair<std::vector<std::vector<float>>,std::vector<std::vector<float>>> );

    vector<double> nominalSystem(const vector<float>&, const vector<float>& );
    bool checkCollisionInVision(const std::pair<std::vector<std::vector<float>>,std::vector<std::vector<float>>>&, const IloCplex&, int n) const;


    IloExpr bicycleModel(std::pair<vector<double>, vector<float>>, IloArray<IloNumVarArray>&,  IloArray<IloNumVarArray>&, const int, const int, float);
    IloExpr tricycleModel(std::pair<vector<double>, vector<float>>, IloArray<IloNumVarArray>&,  IloArray<IloNumVarArray>&, const int, const int, float);

};
#endif //COLLISION_AVOIDANCE_SOLVER_H
