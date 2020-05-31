//
// Created by jieming on 28.05.20.
//

#ifndef COLLISION_AVOIDANCE_CIAO_H
#define COLLISION_AVOIDANCE_CIAO_H

#include "solver.h"
#include "Config.h"
using std::pair;
class CIAO {
private:
    IloEnv env_;

    IloArray<IloNumVarArray> x_;
    IloArray<IloNumVarArray> u_;
    vector<vector<float>> c_;
    IloNumVarArray slack_;
    IloNumVarArray eta_;

    int N_;
    float safe_dist_;
    float dt_;
    vector<float> Q_;
    vector<float> R_;
    vector<float> P_;

public:
    CIAO();
    vector<float> ciaoIteration(vector<vector<float>> inital_guess, pair<vector<double>, vector<float>> job_point, vector<double>target_point, vector<float>current_state, pair<vector<vector<float>>, vector<vector<float>>>);
    vector<vector<float>> maximizeFB(const vector<vector<float>>&);
    vector<float> solveNLP(const vector<vector<float>>&, std::pair<vector<double>, vector<float>>, vector<double>, vector<float>, pair<vector<vector<float>>, vector<vector<float>>>);
    void mpc();
    int closestObs(const vector<vector<float>>&, int idx) const;
    void initializeCenters(const vector<vector<float>>&);

    IloExpr bicycleModel(std::pair<vector<double>, vector<float>> job_point, IloArray<IloNumVarArray>& x,  IloArray<IloNumVarArray>& u, const int i, const int j, float dt);
};


#endif //COLLISION_AVOIDANCE_CIAO_H
