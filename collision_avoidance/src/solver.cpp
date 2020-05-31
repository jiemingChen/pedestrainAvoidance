//
// Created by jieming on 28.04.20.
//

#include "solver.h"
#include <cmath>
#include <limits>
#include <boost/concept_check.hpp>
#include "Config.h"

Solver::Solver():N_(Config::get<float>("N_")), x_(env_, 3), u_(env_, 2), slack_(env_, N_){
    Q={1, 1, 0};
    P={1, 1, 0};
    R={0.09, 0.8};
    safe_dist_ = Config::get<float>("safe_dist");
    dt_  = Config::get<float>("dt");
    previous_timestamp_ =  std::chrono::high_resolution_clock::now();
    // initialize decision variable
    stringstream name;
    for(auto i=0; i<3; i++){
        x_[i] = IloNumVarArray(env_, N_+1);
        for(auto j=0; j<N_+1; j++){
            name << "x_" << i << "_" << j;
            if(i==0 || i==1){
                x_[i][j] = IloNumVar(env_, -300, 300, IloNumVar::Float, name.str().c_str());
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
                u_[i][j] = IloNumVar(env_, 0, 0.5, IloNumVar::Float, name.str().c_str());
            }
            else{
                u_[i][j] = IloNumVar(env_, -M_PI/4, M_PI/4, IloNumVar::Float, name.str().c_str());
            }
            name.str("");
        }
    }
//    for(auto i=0; i<N_; i++){
//        name << "slack_"<<i;
//        slack_[i] = IloNumVar(env_, 0, 100000, IloNumVar::Float, name.str().c_str());
//        name.str("");
//    }

}
#if 0
vector<float> Solver::solve(std::pair<vector<double>, vector<float>> job_point, vector<double>target_point, vector<float>current_state, std::pair<std::vector<std::vector<float>>,std::vector<std::vector<float>>> obstacles){
// mpc without obstacle
    IloRangeArray dynamic_constraints(env_, 3*N_);
    IloRangeArray x0_constraints(env_, 3);
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
                expr +=  (x_[i][j]-target_point[i])*P[i]*(x_[i][j]-target_point[i]);
            }
            else{
                expr +=  (x_[i][j]-target_point[i])*Q[i]*(x_[i][j]-target_point[i]);
            }
        }
    }
    for(auto i = 0; i < 2; ++i) {
        for (auto j = 1; j < N_; ++j){
            expr += (u_[i][j]-u_[i][j-1])*R[i]*(u_[i][j]-u_[i][j-1]);
        }
    }
    IloObjective obj(env_, expr, IloObjective::Minimize);
    model.add(obj);
    IloCplex cplex(model);
    cplex.setOut(env_.getNullStream());
//    cplex.exportModel("/home/jieming/model.lp");
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
        cout<< "initialization success"<<endl;
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

// mpc with obstacles
    if(!obstacles.first.empty()){
        auto obstacles_size = obstacles.first.size();
        IloRangeArray collision_constraints(env_, N_*obstacles_size);
        int rou = 50000;
        double xIter[3][N_];
        double lastcost;

        for(auto j=0; j<N_;j++){
            xIter[0][j] =  cplex.getValue(x_[0][j]);
            xIter[1][j] =  cplex.getValue(x_[1][j]);
            xIter[2][j] =  cplex.getValue(x_[2][j]);
        }
        lastcost = cplex.getObjValue();
        model.remove(obj);
        IloNumVarArray slack(env_, N_*obstacles_size);
        for(auto i=0; i<N_*obstacles_size; i++){
            name << "slack_"<<i;
            slack[i] = IloNumVar(env_, 0, IloInfinity, IloNumVar::Float, name.str().c_str());
            name.str("");
        }

        for (auto i = 0; i < N_*obstacles_size; ++i){
            expr += rou*slack[i];
        }
        obj.setExpr(expr);
        expr.clear();
        model.add(obj);

//         for loop
        int cnt=0;
        while(1){
            for(auto j=0; j<N_; j++){
                for(auto k=0; k<obstacles_size; k++){
                    expr = (xIter[0][j]-obstacles.first[k][0])*(x_[0][j]-obstacles.first[k][0])+(xIter[1][j]-obstacles.first[k][1])*(x_[1][j]-obstacles.first[k][1]) + slack[j*obstacles_size+k] - \
                                (pow(safe_dist_, 2)+pow(xIter[0][j]-obstacles.first[k][0], 2)+pow(xIter[1][j]-obstacles.first[k][1], 2))/2;
                    name << "collisionConstr_" << "_" << j << "_"<< k;
                    collision_constraints[j*obstacles_size+k] = IloRange(env_, 0, expr, IloInfinity, name.str().c_str());
                    name.str("");
                    expr.clear();
                }
            }
            model.add(collision_constraints);
//            IloCplex cplex_collision(model);

            solved = false;
            try {
                solved = cplex.solve();
//                cplex.exportModel("/home/jieming/model3.lp");

            }
            catch(const IloException& e) {
                std::cerr << "\n\nCPLEX Raised an exception:\n";
                std::cerr << e << "\n";
                env_.end();
                throw("error");
            }

            if(solved) {
//                std::cout << "\tObjective value: " << cplex.getObjValue() << "\n";

            }
            else {
                std::cerr << "\n\nCplex error!\n";
                std::cerr << "\tStatus: " << cplex.getStatus() << "\n";
                std::cerr << "\tSolver status: " << cplex.getCplexStatus() << "\n";
                throw("can not find a path, no initial solution");
            }

            cnt ++;
            if(abs(lastcost-cplex.getObjValue())<1) {
                vector<float> tmp;
                for(auto i=0; i<2; i++){
                    tmp.push_back(cplex.getValue(u_[i][0]));
                }
                for(auto j=0; j<N_; j++){
                    tmp.push_back(cplex.getValue(x_[0][j]));
                    tmp.push_back(cplex.getValue(x_[1][j]));
                    tmp.push_back(cplex.getValue(x_[2][j]));
                }
                rst = tmp;
//                cout<<"with obstacles!!!!!!" << cplex.getValue(u_[0][0]) << "   "<<cplex.getValue( u_[1][0])<<endl;

                break;
            }
            else if(cnt>25){
                cout<<"too many iterations, goes to next point"<<endl;
                vector<float> tmp;
                for(auto i=0; i<2; i++){
                    tmp.push_back(cplex.getValue(u_[i][0]));
                }
                for(auto j=0; j<N_; j++){
                    tmp.push_back(cplex.getValue(x_[0][j]));
                    tmp.push_back(cplex.getValue(x_[1][j]));
                    tmp.push_back(cplex.getValue(x_[2][j]));
                }
                rst = tmp;
                cout<<"with obstacles!!!!!!" << cplex.getValue(u_[0][0]) << "   "<<cplex.getValue( u_[1][0])<<endl;

                break;
            }
            else{
                for(auto j=0; j<N_;j++){
                    xIter[0][j] =  cplex.getValue(x_[0][j]);
                    xIter[1][j] =  cplex.getValue(x_[1][j]);
                    xIter[2][j] =  cplex.getValue(x_[2][j]);
                }
                lastcost = cplex.getObjValue();

                model.remove(collision_constraints);

            }
        }
    }
    //post process
    expr.end();
    return rst;
}
#endif

vector<float> Solver::solve2(std::pair<vector<double>, vector<float>> job_point, vector<double>target_point, vector<float>current_state, std::pair<std::vector<std::vector<float>>,std::vector<std::vector<float>>>  obstacles){
// mpc without obstacle
    IloRangeArray dynamic_constraints(env_, 3*N_);
    IloRangeArray x0_constraints(env_, 3);
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

    auto current = std::chrono::high_resolution_clock::now();
    std::chrono::milliseconds delta_time = std::chrono::duration_cast<std::chrono::milliseconds>(current - previous_timestamp_);
    previous_timestamp_ = current;
//    float dt_people = (float)(delta_time.count()) /1000;
    float dt_people = dt_;

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
                expr +=  (x_[i][j]-target_point[i])*P[i]*(x_[i][j]-target_point[i]);
            }
            else{
                expr +=  (x_[i][j]-target_point[i])*Q[i]*(x_[i][j]-target_point[i]);
            }
        }
    }
    for(auto i = 0; i < 2; ++i) {
        for (auto j = 1; j < N_; ++j){
            expr += (u_[i][j]-u_[i][j-1])*R[i]*(u_[i][j]-u_[i][j-1]);
        }
    }
    IloObjective obj(env_, expr, IloObjective::Minimize);
    model.add(obj);
    IloCplex cplex(model);
    cplex.setOut(env_.getNullStream());
//    cplex.exportModel("/home/jieming/model.lp");
    bool solved = false;
    try {
        solved = cplex.solve();
//        cplex.exportModel("/home/jieming/model1.lp");

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

// mpc with obstacles
//    auto obstacles_saw = checkCollisionInVision(obstacles, cplex);
    auto obstacles_saw = obstacles;
//    while(!obstacles_saw.first.empty()){
    vector<float> tried;

    if(!obstacles_saw.first.empty()){
        auto obstacles_size = obstacles_saw.first.size();
        IloRangeArray collision_constraints(env_, N_ * obstacles_size);
        int rou = 2000;
        double xIter[3][N_];
        double lastcost;
        for (auto j = 0; j < N_; j++) {
            xIter[0][j] = cplex.getValue(x_[0][j]);
            xIter[1][j] = cplex.getValue(x_[1][j]);
            xIter[2][j] = cplex.getValue(x_[2][j]);
        }
        lastcost = cplex.getObjValue();
        model.remove(obj);
        IloNumVarArray slack(env_, N_ * obstacles_size);
        for (auto i = 0; i < N_ * obstacles_size; i++) {
            name << "slack_" << i;
            slack[i] = IloNumVar(env_, 0, 50000, IloNumVar::Float, name.str().c_str());
            name.str("");
        }

        for (auto i = 0; i < N_ * obstacles_size; ++i) {
            expr += rou * slack[i];
        }
        obj.setExpr(expr);
        expr.clear();
        model.add(obj);

            //         for loop
        int cnt = 0;
        while (1) {
            for (auto j = 0; j < N_; j++) {
                for (auto k = 0; k < obstacles_size; k++) {
                        expr = (xIter[0][j] - obstacles_saw.first[k][0] - obstacles_saw.second[k][0] * dt_people * j)*(x_[0][j] - obstacles_saw.first[k][0] - obstacles_saw.second[k][0] * dt_people * j) +
                               (xIter[1][j] - obstacles_saw.first[k][1] - obstacles_saw.second[k][1] * dt_people * j)*(x_[1][j] - obstacles_saw.first[k][1] - obstacles_saw.second[k][1] * dt_people * j) +
                               slack[j * obstacles_size + k] - (pow(safe_dist_, 2) +
                               pow(xIter[0][j]-obstacles_saw.first[k][0]-obstacles_saw.second[k][0]*dt_people*j, 2) +
                                     pow(xIter[1][j] - obstacles_saw.first[k][1] - obstacles_saw.second[k][1] * dt_people * j, 2)) / 2;




                        name << "collisionConstr_" << "_" << j << "_" << k;
                        collision_constraints[j * obstacles_size + k] = IloRange(env_, 0, expr, IloInfinity,
                                                                                 name.str().c_str());
                        name.str("");
                        expr.clear();
                    }
                }
                model.add(collision_constraints);
                solved = false;
                try {
                    solved = cplex.solve();
    //                                    cplex.exportModel("/home/jieming/model4.lp");
                    for(auto j=0; j<N_; j++){
                        tried.push_back(cplex.getValue(x_[0][j]));
                        tried.push_back(cplex.getValue(x_[1][j]));
                        tried.push_back(cplex.getValue(x_[2][j]));
                    }
                }
                catch (const IloException &e) {
                    std::cerr << "\n\nCPLEX Raised an exception:\n";
                    std::cerr << e << "\n";
                    env_.end();
                    throw ("error");
                }

                if (solved) {
                    //                std::cout << "\tObjective value: " << cplex.getObjValue() << "\n";
                } else {
                    std::cerr << "\n\nCplex error!\n";
                    std::cerr << "\tStatus: " << cplex.getStatus() << "\n";
                    std::cerr << "\tSolver status: " << cplex.getCplexStatus() << "\n";
                    throw ("can not find a path, no initial solution");
                }

                cnt++;
                if ( (abs(lastcost - cplex.getObjValue()) < 1 && lastcost > cplex.getObjValue()) || (cnt>25)){
                    vector<float> tmp;
                    for (auto i = 0; i < 2; i++) {
                        tmp.push_back(cplex.getValue(u_[i][0]));
                    }
                    for (auto j = 0; j < N_; j++) {
                        tmp.push_back(cplex.getValue(x_[0][j]));
                        tmp.push_back(cplex.getValue(x_[1][j]));
                        tmp.push_back(cplex.getValue(x_[2][j]));
                    }
                    rst = tmp;
                    if(cnt>25){
                        cout << "too many iterations, goes to next point" << endl;
                    }
                    break;
                }
                else {
                    for (auto j = 0; j < N_; j++) {
                        xIter[0][j] = cplex.getValue(x_[0][j]);
                        xIter[1][j] = cplex.getValue(x_[1][j]);
                        xIter[2][j] = cplex.getValue(x_[2][j]);
                    }
                    lastcost = cplex.getObjValue();

                    model.remove(collision_constraints);
                }
            }

    }
    //post process
    expr.end();
    return rst;
//    return  tried;
}

vector<double> Solver::nominalSystem(const vector<float>& input, const vector<float>& current_pos){
    float delta[3]={0,0,0};
    float L=0.75*3/4-0.055;

    vector<double> next_state(3,0);
    /* \brief
    v(t)*cos(a(t))*cos(theta(t))
    v(t)*cos(a(t))*sin(theta(t))
    v(t)*sin(a(t))/d
     */
    delta[0] = input[0] * cos(input[1]) * cos(current_pos[2]);
    delta[1] = input[0] * cos(input[1]) * sin(current_pos[2]);
    delta[2] = input[0] * sin(input[1]) / L;
    next_state[0] = current_pos[0] +  delta[0]*dt_;
    next_state[1] = current_pos[1] +  delta[1]*dt_;
    next_state[2] = current_pos[2] +  delta[2]*dt_;

    std::cout<<input[5]<<"````````````````````"<<endl;
    std::cout<<input[6]<<"````````````````````"<<endl;
    std::cout<<input[7]<<"````````````````````"<<endl;
    std::cout<<next_state[0]<<"````````````````````"<<endl;
    std::cout<<next_state[1]<<"````````````````````"<<endl;
    std::cout<<next_state[2]<<"````````````````````"<<endl;

    std::cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<std::endl;
    std::cout<<delta[0]*dt_<<"````````````````````"<<endl;
    std::cout<<delta[1]*dt_<<"````````````````````"<<endl;
    std::cout<<delta[2]*dt_<<"````````````````````"<<endl;
    std::cout<<input[0]<<"````````````````````"<<endl;
    std::cout<<input[1]<<"````````````````````"<<endl;

    return next_state;
}


IloExpr Solver::bicycleModel(std::pair<vector<double>, vector<float>> job_point,
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


IloExpr Solver::tricycleModel(std::pair<vector<double>, vector<float>> job_point,
                      IloArray<IloNumVarArray>& x,  IloArray<IloNumVarArray>& u, const int i, const int j, float dt){
    float phi0 = job_point.first[2];
    float v0 = job_point.second[0];
    float psi0 = job_point.second[1];
    static float L=0.75*3/4-0.055;

    Eigen::Matrix3f Ad;
    Ad << 1, 0, -v0*sin(phi0)*dt,
            0, 1,  v0*cos(phi0)*dt,
            0, 0,  1;

    Eigen::MatrixXf Bd(3,2);
    Bd << cos(phi0)*dt, 0,
          sin(phi0)*dt, 0,
          1/L*tan(psi0)*dt, v0/(L*cos(psi0)*cos(psi0))*dt;

    Eigen::Vector3f Ed;
    Ed <<  v0*sin(phi0)*phi0*dt,
            -v0*cos(phi0)*phi0*dt,
            -v0/(L*cos(psi0)*cos(psi0))*psi0*dt;

    IloExpr expr(env_);
    expr = Ad(i,0)*x[0][j] + Ad(i,1)*x[1][j] + Ad(i,2)*x[2][j] + Bd(i,0)*u[0][j] + Bd(i,1)*u[1][j] + Ed(i);
    return expr;


}


bool Solver::checkCollisionInVision(const std::pair<std::vector<std::vector<float>>,std::vector<std::vector<float>>>& obstacles, const IloCplex& cplex, int n) const{
    bool sign=true;
    for(auto k=0; k<obstacles.first.size(); k++){
        for(auto j=0; j<N_; j++){
            float distance = pow(cplex.getValue(x_[0][j]) - obstacles.first[k][0], 2) + pow(cplex.getValue(x_[1][j]) - obstacles.first[k][1], 2);
            if(distance <pow(safe_dist_, 2)){
                sign = false;
                break;
            }
        }

    }
    return  sign;
}



Solver::~Solver(){
    env_.end();
}

//
//void Solver::print_solution(const IloCplex& cplex, const IloArray<IloNumVarArray>& x) const {
//    const auto n = g.size();
//    assert(x.getSize() == n);
//
//    std::cout << "\n\nTour: ";
//
//    const auto starting_vertex = 0u;
//    auto current_vertex = starting_vertex;
//
//    do {
//        std::cout << current_vertex << " ";
//        for(auto i = 0u; i < n; ++i) {
//            if(cplex.getValue(x[current_vertex][i]) > .5) {
//                current_vertex = i;
//                break;
//            }
//        }
//    } while(current_vertex != starting_vertex);
//
//    std::cout << "\n";
//}



//    IloNumVarArray x(env);
//    IloRangeArray con(*env);//    env_ = shared_ptr<IloEnv>(new IloEnv);
////    model_ = shared_ptr<IloModel>(new IloModel(*env_));
