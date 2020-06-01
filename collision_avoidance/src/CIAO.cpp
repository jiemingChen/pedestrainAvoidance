//
// Created by jieming on 28.05.20.
//
#include "CIAO.h"

CIAO::CIAO():N_(10), x_(env_, 3), u_(env_, 2), slack_(env_, 10){
    Q_={1, 1, 0};
    P_={1, 1, 0};
    R_={0.09, 0.8};
    safe_dist_ = 1.5;
    dt_  = 0.5;
    // initialize decision variable
    stringstream name;
    for(auto i=0; i<3; i++){
        x_[i] = IloNumVarArray(env_, N_+1);
        for(auto j=0; j<N_+1; j++){
            name << "x_" << i << "_" << j;
            if(i==0 || i==1){
                x_[i][j] = IloNumVar(env_, -IloInfinity, IloInfinity, IloNumVar::Float, name.str().c_str());
            }
            else{
                x_[i][j] = IloNumVar(env_, -M_PI*2, M_PI*2, IloNumVar::Float, name.str().c_str());
            }
            name.str("");
        }
    }
    for(auto i=0; i<2; i++){
        u_[i] = IloNumVarArray(env_, N_);
        for(auto j=0; j<N_; j++){
            name << "u_" << i << "_" << j;
            if(i==0){
                u_[i][j] = IloNumVar(env_, 0.8, 1.0, IloNumVar::Float, name.str().c_str());
            }
            else{
                u_[i][j] = IloNumVar(env_, -M_PI/4, M_PI/4, IloNumVar::Float, name.str().c_str());
            }
            name.str("");
        }
    }
    for(auto i=0; i<N_; i++){
        name << "slack_" << i;
        slack_[i] = IloNumVar(env_, 0, IloInfinity, IloNumVar::Float, name.str().c_str());
        name.str("");
    }
}

vector<float> CIAO::ciaoIteration(vector<vector<float>> inital_guess, pair<vector<double>, vector<float>> job_point, vector<double>target_point, vector<float>current_state, pair<vector<vector<float>>, vector<vector<float>>> obs){
    initializeCenters(inital_guess);
    auto circles = maximizeFB(obs.first);
    auto rst = solveNLP(circles, job_point, target_point, current_state, obs);
    return rst;
}

vector<float> CIAO::solveNLP(const vector<vector<float>>& circles, std::pair<vector<double>, vector<float>> job_point, vector<double>target_point, vector<float>current_state, pair<vector<vector<float>>, vector<vector<float>>> obs){
    IloRangeArray dynamic_constraints(env_, 3*N_);
    IloRangeArray x0_constraints(env_, 3);
    IloRangeArray ob_constraints(env_, N_);

    IloExpr expr(env_);
    IloModel model(env_);
    vector<float> rst;
    stringstream name;
    // constraints
    for(auto i=0; i<3; i++){
        expr = x_[i][0] - current_state[i];
        name << "x0_constraints_" << i;
        x0_constraints[i] = IloRange(env_, 0, expr, 0, name.str().c_str());
        name.str("");
        expr.clear();
    }
    model.add(x0_constraints);
    for(auto i=0; i<3; ++i){
        for(auto j=0; j<N_; ++j){
            auto model_expr = bicycleModel(job_point, x_, u_, i, j, dt_);
            expr = x_[i][j+1] - model_expr;
            name << "dynamicConstr_" << i << "_" << j;
            dynamic_constraints[i*N_+j] = IloRange(env_, 0, expr, 0, name.str().c_str());
            name.str("");
            expr.clear();
        }
    }
    model.add(dynamic_constraints);

    for(auto i = 0; i < 3; ++i) {
        for (auto j = 0; j < N_; ++j){
            if(j==N_-1){
                expr +=  (x_[i][j]-target_point[i])*P_[i]*(x_[i][j]-target_point[i]);
            }
            else{
                expr +=  (x_[i][j]-target_point[i])*Q_[i]*(x_[i][j]-target_point[i]);
            }
        }
    }
    for(auto i = 0; i < 2; ++i) {
        for (auto j = 1; j < N_; ++j){
            expr += (u_[i][j]-u_[i][j-1])*R_[i]*(u_[i][j]-u_[i][j-1]);
        }
    }
    for(auto i=0; i<N_; i++){
        expr += 5000*slack_[i];
    }
    for(auto i=1; i<N_; i++){
        for(auto j=0; j<obs.first.size(); j++){
            expr += 100* 1 / ((x_[0][i] - obs.first[j][0])*(x_[0][i] - obs.first[j][0]) + (x_[1][i] - obs.first[j][1])*(x_[1][i] - obs.first[j][1]) );
        }
    }

    IloObjective obj(env_, expr, IloObjective::Minimize);
    model.add(obj);
    expr.clear();

    for(auto i=0; i<circles.size(); i++){
        expr = (x_[0][i]-circles[i][0])*(x_[0][i]-circles[i][0])+(x_[1][i]-circles[i][1])*(x_[1][i]-circles[i][1])-slack_[i];
        name << "obConstr_" << i;
//        if(circles[i][2]-safe_dist_-dt_/2*0.8 <0){
//            cout<<"not large enough"<<endl;
//            throw("no large enough circ");
//        }
        float upper_limit = pow(circles[i][2]-safe_dist_-dt_/2*0.8, 2); //speed
        ob_constraints[i] = IloRange(env_, -IloInfinity, expr, upper_limit, name.str().c_str());;
        expr.clear();
        name.str("");
    }
    model.add(ob_constraints);
    IloCplex cplex(model);
    cplex.setOut(env_.getNullStream());

    bool solved = false;
    try {
        solved = cplex.solve();
    }
    catch(const IloException& e) {
        std::cerr << "\n\nCPLEX Raised an exception:\n";
        std::cerr << e << "\n";
        env_.end();
        throw;
    }
    if(solved) {
        for(auto i=0; i<2; i++){
            rst.push_back(cplex.getValue(u_[i][0]));
        }
        for(auto j=0; j<N_; j++){
            rst.push_back(cplex.getValue(x_[0][j]));
            rst.push_back(cplex.getValue(x_[1][j]));
            rst.push_back(cplex.getValue(x_[2][j]));
        }
    }
    else {
        std::cerr << "\n\nCplex error!\n";
        std::cerr << "\tStatus: " << cplex.getStatus() << "\n";
        std::cerr << "\tSolver status: " << cplex.getCplexStatus() << "\n";
        throw("can not find a path, no initial solution");
    }
    return rst;
}

int CIAO::closestObs(const vector<vector<float>>& obs, int idx) const {
    float min_dist = 20000;
    int  min_id = 0;
    for(auto i=0; i<obs.size(); i++){
        auto dist = pow(obs[i][0]-c_[0][idx], 2) + pow(obs[i][1]-c_[1][idx], 2);
        if(dist<min_dist){
            min_dist = dist;
            min_id = i;
        }
    }
    return min_id;
}
vector<vector<float>> CIAO::maximizeFB(const vector<vector<float>>& obs){

    vector<vector<float>> gamma_vec;
    vector<int> ids;

    vector<vector<float>> circles;
    vector<float> circle(3,0);
    float dist=0;
    std::stringstream name;
    if(obs.size()>0) {
        for(auto i=0; i<N_; i++) {
            auto ob_id = closestObs(obs, i);
            auto theta = atan2(c_[1][i] - obs[ob_id][1], c_[0][i] - obs[ob_id][0]);
            vector<float> gamma = {cos(theta), sin(theta)};
            gamma_vec.push_back(gamma);
            ids.push_back(ob_id);
            float eta_max = 8;
            float x_range_upper = c_[0][i] + eta_max*gamma[0];
            float y_range_upper = c_[1][i] + eta_max*gamma[1];
            float x_range_upper_old = x_range_upper;
            float y_range_upper_old = y_range_upper;
            float x_range_lower = c_[0][i];
            float y_range_lower = c_[1][i];
            while(true){
                float new_x = (x_range_upper+x_range_lower) /2;
                float new_y = (y_range_upper+y_range_lower) /2;

                float radius =  pow(new_x - obs[ob_id][0], 2) + pow(new_y - obs[ob_id][1], 2);
                bool flag = true;
                for(auto j=0; j<obs.size(); j++){
                    if(j == ob_id){
                        continue;
                    }
                    float dist = pow(new_x - obs[j][0], 2) + pow(new_y - obs[j][1], 2);
                    if(dist < radius){
                        x_range_upper = new_x;
                        y_range_upper = new_y;
                        x_range_upper_old = x_range_upper;
                        y_range_upper_old = y_range_upper;
                        flag = false;
                        break;
                    }
                }
                if(flag){
                    x_range_lower = new_x;
                    y_range_lower = new_y;
                    x_range_upper = x_range_upper_old;
                    y_range_upper = y_range_upper_old;
                }
                if(abs(x_range_upper-x_range_lower)<0.1 && abs(y_range_upper-y_range_lower)<0.1){
                    break;
                }
            }
            c_[0][i] = (x_range_upper+x_range_lower)/2;
            c_[1][i] = (y_range_upper+y_range_lower)/2;
        }
        for (auto i = 0; i < N_; i++) {
            circle[0] = c_[0][i];
            circle[1] = c_[1][i];
            dist = pow(circle[0] - obs[ids[i]][0], 2) + pow(circle[1] - obs[ids[i]][1], 2);
            circle[2] = pow(dist, 0.5);

            circles.push_back(circle);
        }
    }
    else{
        for (auto i = 0; i < N_; i++) {
            circle[0] = c_[0][i];
            circle[1] = c_[1][i];
            circle[2] = 20;
            circles.push_back(circle);
        }
    }
#if 1
    std::ofstream out_file;
    out_file.open("/home/jieming/circle.txt");
    for(auto i=0; i<N_; i++){
        out_file << c_[0][i] << " " << c_[1][i] <<" "<< ids[i]<<endl;
    }
    for(auto i=0; i<obs.size(); i++){
        out_file << obs[i][0] << " " << obs[i][1] <<endl;
    }
    out_file.close();
#endif
    return circles;

}
void CIAO::initializeCenters(const vector<vector<float>>& initial){
    vector<float> tmp_center;
    c_.clear();
    for(auto i=0; i<2; i++){
        for(auto j=0; j<N_; j++){
            tmp_center.push_back(initial[j][i]);
        }
        c_.push_back(tmp_center);
        tmp_center.clear();
    }
}


IloExpr CIAO::bicycleModel(std::pair<vector<double>, vector<float>> job_point,
                             IloArray<IloNumVarArray>& x,  IloArray<IloNumVarArray>& u, const int i, const int j, float dt){
    double theta0 = job_point.first[2];
    float v0 = job_point.second[0];
    float alpha0 = job_point.second[1];
    static float L=0.75*3/4-0.055;

    Eigen::Matrix3f Ad;
    Ad << 1, 0, -v0*cos(alpha0)*sin(theta0)*dt,
            0, 1,  v0*cos(alpha0)*cos(theta0)*dt,
            0, 0,  1;

    Eigen::MatrixXf Bd(3,2);
    Bd << cos(alpha0)*cos(theta0)*dt,  -v0*cos(theta0)*sin(alpha0)*dt,
            cos(alpha0)*sin(theta0)*dt,  -v0*sin(alpha0)*sin(theta0)*dt,
            1/L*sin(alpha0)*dt,           v0/L*cos(alpha0)*dt;

    Eigen::Vector3f Ed;
    Ed << (v0*cos(alpha0)*sin(theta0)*theta0+v0*cos(theta0)*sin(alpha0)*alpha0)*dt,
            (-v0*cos(alpha0)*cos(theta0)*theta0+v0*sin(alpha0)*sin(theta0)*alpha0)*dt,
            -v0/L*cos(alpha0)*alpha0*dt;

    IloExpr expr(env_);
    expr = Ad(i,0)*x[0][j] + Ad(i,1)*x[1][j] + Ad(i,2)*x[2][j] + Bd(i,0)*u[0][j] + Bd(i,1)*u[1][j] + Ed(i);
    return expr;
}
