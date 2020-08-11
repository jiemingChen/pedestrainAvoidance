//
// Created by jieming on 28.04.20.
//

#include "solver.h"
#include <cmath>
#include <limits>
#include <boost/concept_check.hpp>
#include "Config.h"

Solver::Solver():N_(22), x_(env_, 3), u_(env_, 2), slack_(env_, N_){
    Q={1, 1, 0};
    P={1, 1, 0};
    R={0.09, 0.8};
//    safe_dist_ = Config::get<float>("safe_dist");
    safe_dist_ = 1;
    safe_dist_wall_ = 0.7;
//    dt_  = Config::get<float>("dt");
//    dt_  = 0.6*2;
    dt_  = 0.4;
//    previous_timestamp_ =  std::chrono::high_resolution_clock::now();
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
                u_[i][j] = IloNumVar(env_, 0.12, 0.2, IloNumVar::Float, name.str().c_str());
            }
            else{
                u_[i][j] = IloNumVar(env_, -0.7, 0.7, IloNumVar::Float, name.str().c_str());
            }
            name.str("");
        }
    }

}

vector<float> Solver::solve2(pair<vector<double>, vector<float>> job_point, vector<double>target_point, vector<float>current_state, pair<vector<vector<float>>,vector<vector<float>>> obstacles){
// mpc without obstacle
    IloRangeArray dynamic_constraints(env_, 3*N_);
    IloRangeArray x0_constraints(env_, 3);
    IloExpr expr(env_);
    IloModel model(env_);
    vector<float> rst;
    stringstream name;
//    IloCplex::Param::TimeLimit
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
    float dt_people = (float)(delta_time.count()) /1000;

    for(auto i=0; i<3; ++i){
        for(auto j=0; j<N_; ++j){
            auto model_expr = tricycleModel(job_point, x_, u_, i, j, dt_);
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
//    cplex.setParam(IloCplex::Param::Emphasis::Numerical, 1);
//    cplex.setParam(IloCplex::Param::Read::DataCheck	, 2);
    cplex.setParam(IloCplex::Param::Preprocessing::Dependency, 3);

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
        throw ("problem!!!!!!!!!!!!!!");
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
//        std::cerr << "\n\nCplex error  no obs!\n";
//        std::cerr << "\tStatus: " << cplex.getStatus() << "\n";
//        std::cerr << "\tSolver status: " << cplex.getCplexStatus() << "\n";
//                cplex.exportModel("/home/jieming/errormodel4.sav");
        if(cplex.getCplexStatus() == 6){
            std::cerr << "\tSolver status: " << cplex.getCplexStatus() << "\n";
            for(auto i=0; i<2; i++){
                rst.push_back(cplex.getValue(u_[i][0]));
            }
            for(auto j=0; j<N_; j++){
                rst.push_back(cplex.getValue(x_[0][j]));
                rst.push_back(cplex.getValue(x_[1][j]));
                rst.push_back(cplex.getValue(x_[2][j]));
            }
//            rst = last_rst;
        }
        else{
            std::cerr << "\n\nCplex error  no obs!\n";
            std::cerr << "\tStatus: " << cplex.getStatus() << "\n";
            std::cerr << "\tSolver status: " << cplex.getCplexStatus() << "\n";
            throw ("problem2222222!!!!!!!!!!!!!!");
        }
    }

// mpc with obstacles


    if(!obstacles.first.empty() ){
        auto obstacles_size = obstacles.first.size();
        auto people_size = obstacles.second.size();

        IloRangeArray collision_constraints(env_, N_ * obstacles_size);
        int rou = 50000;
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
            name << "slack" << i;
            slack[i] = IloNumVar(env_, 0, 50000, IloNumVar::Float, name.str().c_str());
            name.str("");
        }

        for (auto i = 0; i < N_ * obstacles_size; ++i) {
            expr += rou * slack[i];
        }
        obj.setExpr(expr);
        expr.clear();
        model.add(obj);
        //float dt_people = dt_;

            //         for loop
        int cnt = 0;
        while (1) {
            for (auto j = 0; j < N_; j++) {
                for (auto k = 0; k < obstacles_size; k++) {
//                    if(k<people_size)
//                        cout << obstacles.second[k][0] << " " << obstacles.second[k][1] << " " <<j*dt_ << endl;

                    if(k <people_size)
#if 0
                        expr = (xIter[0][j] - obstacles.first[k][0])*(x_[0][j] - obstacles.first[k][0]) +
                            (xIter[1][j] - obstacles.first[k][1])*(x_[1][j] - obstacles.first[k][1]) +
                            slack[j * obstacles_size + k] - (pow(safe_dist_, 2) + pow(xIter[0][j]-obstacles.first[k][0], 2) + pow(xIter[1][j] - obstacles.first[k][1], 2)) / 2;
#endif
#if 1
                        expr = (xIter[0][j] - obstacles.first[k][0]-obstacles.second[k][0]*j*dt_people)*(x_[0][j] - obstacles.first[k][0]-obstacles.second[k][0]*j*dt_people) +
                               (xIter[1][j] - obstacles.first[k][1]-obstacles.second[k][1]*j*dt_people)*(x_[1][j] - obstacles.first[k][1]-obstacles.second[k][1]*j*dt_people) +
                               slack[j * obstacles_size + k] - (pow(safe_dist_, 2) + pow(xIter[0][j]-obstacles.first[k][0]-obstacles.second[k][0]*j*dt_people, 2) + pow(xIter[1][j] - obstacles.first[k][1]-obstacles.second[k][1]*j*dt_people, 2)) / 2;
#endif
                    else
                        expr = (xIter[0][j] - obstacles.first[k][0])*(x_[0][j] - obstacles.first[k][0]) +
                               (xIter[1][j] - obstacles.first[k][1])*(x_[1][j] - obstacles.first[k][1]) +
                               slack[j * obstacles_size + k] - (pow(safe_dist_wall_, 2) + pow(xIter[0][j]-obstacles.first[k][0], 2) + pow(xIter[1][j] - obstacles.first[k][1], 2)) / 2;


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
            }
            catch (const IloException &e) {
                std::cerr << "\n\nCPLEX Raised an exception in collision :\n";
                std::cerr << e << "\n";
                goto emergency;
            }

            if (solved) {
            }
            else {
                std::cerr << "\n\nCplex error obs!\n";
                std::cerr << "\tStatus: " << cplex.getStatus() << "\n";
                std::cerr << "\tSolver status: " << cplex.getCplexStatus() << "\n";
                goto emergency;
            }

            cnt++;
            if ( (abs(lastcost - cplex.getObjValue()) < 1 && lastcost > cplex.getObjValue()) ){
                bool flag = false;
                float violate_cnt= 0;
                for(auto i=0; i<N_ * obstacles_size; i++){
//                    cout<<cplex.getValue(slack[i])<<" ";
                    if(cplex.getValue(slack[i]) > 0.1){
                        violate_cnt ++;
                        if(violate_cnt>8*obstacles_size){
                            cout <<"danger"<<endl;

                            flag = true;
                            break;
                        }
                    }
                }
//                cout<<"\n"<<endl;
                if(!flag){
                    cout <<"solved  " << cnt <<endl;
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
                    break;
                }
            }
            if(cnt>16){
emergency:
                float d = 10000;
                float temp_d=1000;
                int dir=0;
                int idx = 0;
                for(auto i=0; i<N_ * obstacles_size; i++){
                    if(cplex.getValue(slack[i])>0.1){
                        idx = int(i/obstacles_size);
                        break;
                    }
                }
                for(auto& ob: obstacles.first){
                    temp_d = pow(pow(cplex.getValue(x_[0][idx])-ob[0],2)+pow(cplex.getValue(x_[0][idx])-ob[1],2), 0.5);
                    if(temp_d<d){
                        d = temp_d;
                        dir = current_state[1]-ob[1]>0? 1:-1;
                    }
                }
                float u_steer = abs(safe_dist_-temp_d)*dir*0.4;
                if(u_steer>M_PI/4){
                    u_steer = M_PI/4;
                }
                else if(u_steer< -M_PI/4){
                    u_steer = -M_PI/4;
                }
                vector<float> tmp;
                tmp.push_back(0.1);
                tmp.push_back(u_steer);
                for (auto j = 0; j < N_; j++) {
                    tmp.push_back(cplex.getValue(x_[0][0]));
                    tmp.push_back(cplex.getValue(x_[1][0]));
                    tmp.push_back(cplex.getValue(x_[2][0]));
                }
                rst = tmp;
                cout<<"BUG strategy..." << u_steer <<endl;
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
        collision_constraints.end();
        slack.end();
    }
    //post process
    model.end();
    expr.end();
    dynamic_constraints.end();
    x0_constraints.end();
    cplex.end();
    last_rst = rst;
    return rst;
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
    static float L=0.52;

    Eigen::Matrix3f Ad;
    Ad << 1, 0, -v0*cos(alpha0)*sin(theta0)*dt,
          0, 1,  v0*cos(alpha0)*cos(theta0)*dt,
          0, 0,  1;

    Eigen::MatrixXf Bd(3,2);
    Bd << cos(alpha0)*cos(theta0)*dt,  -v0*cos(theta0)*sin(alpha0)*dt,
          cos(alpha0)*sin(theta0)*dt,  -v0*sin(alpha0)*sin(theta0)*dt,
          1.0/L*sin(alpha0)*dt,           v0/L*cos(alpha0)*dt;

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
    static float L=0.52;

    Eigen::Matrix3f Ad;
    Ad << 1, 0, -v0*sin(phi0)*dt,
            0, 1,  v0*cos(phi0)*dt,
            0, 0,  1;

    Eigen::MatrixXf Bd(3,2);
    Bd << cos(phi0)*dt, 0,
          sin(phi0)*dt, 0,
          1/L*tan(psi0)*dt, v0/(L*cos(psi0)*cos(psi0))*dt;
//          0,              v0/L*dt;
    Eigen::Vector3f Ed;
    Ed <<  v0*sin(phi0)*phi0*dt,
            -v0*cos(phi0)*phi0*dt,
            -v0/(L*cos(psi0)*cos(psi0))*psi0*dt;
//            0;
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


//                        expr = (xIter[0][j] - obstacles_saw.first[k][0] - obstacles_saw.second[k][0] * dt_people * j)*(x_[0][j] - obstacles_saw.first[k][0] - obstacles_saw.second[k][0] * dt_people * j) +
//                               (xIter[1][j] - obstacles_saw.first[k][1] - obstacles_saw.second[k][1] * dt_people * j)*(x_[1][j] - obstacles_saw.first[k][1] - obstacles_saw.second[k][1] * dt_people * j) +
//                               slack[j * obstacles_size + k] - (pow(safe_dist_, 2) +
//                               pow(xIter[0][j]-obstacles_saw.first[k][0]-obstacles_saw.second[k][0]*dt_people*j, 2) +
//                                     pow(xIter[1][j] - obstacles_saw.first[k][1] - obstacles_saw.second[k][1] * dt_people * j, 2)) / 2;


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
