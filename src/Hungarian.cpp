//
// Created by jieming on 06.02.20.
//
#include "Hungarian.h"

Hungarian::Hungarian(const vector<vector<double>>& e, const int& xsize, const int& ysize){
    edge = e;
    for(int i=0; i< ysize; i++){
        path.push_back(-1);
        onPath.push_back(false);
    }
}

Hungarian::~Hungarian(){
}

void Hungarian::clearOnPathSign(){
    for(int i=0; i<onPath.size(); i++){
        onPath[i] = false;
    }  //assign?
}

bool Hungarian::FindAugPath( int xi){
    for(int yj=0; yj<onPath.size(); yj++){
        if((edge[xi][yj]>0) && (!onPath[yj])){
            onPath[yj] = true;
            if((path[yj]==-1) || FindAugPath(path[yj])){
                path[yj] = xi;
                return true;
            }
        }
    }
}

void Hungarian::Hungary_match(){

    for(int xi=0; xi<edge.size(); xi++){
        FindAugPath(xi);/*feed into ith x*/
        clearOnPathSign(); /*clear sign on y*/
    }
}

void Hungarian::outputRes(){
    for (int i = 0 ; i<path.size(); i++) {
        cout<<i<<"....."<<path[i]<<endl;
    }
}

/*return index*/
std::tuple< vector<pair<int, int>>,vector<int>,vector<int> > KM::match(std::vector<std::vector<float>>& edges){
    /*1. generate square matrix and convert from maximum to minimum prrblem*/
    int originRow = edges.size();
    int originCol = edges[0].size(); //maybe have a bug
    auto squared = generateSquareMatrx(edges);
    int siz = squared.size();
    double xArray[siz*siz];
    vector<std::pair<int, int>> matches;
    vector<int> unmatchedP, unmatchedM;
    /*2. solve  optimal minimum problem*/

    const int nx   = pow(siz,2);
    double c[nx];
    for(int i=0; i<siz; i++){
        for(int j=0; j<siz; j++){
            c[j+i*siz] = squared[i][j];
        }
    }
    double  xupp[nx];
    char   ixupp[nx];
    std::fill(xupp, xupp+nx, 1);
    std::fill(ixupp,ixupp+nx, 1);

    double  xlow[nx];
    char   ixlow[nx];
    std::fill(xlow, xlow+nx, 0);
    std::fill(ixlow,ixlow+nx, 1);

    const int nnzQ = 0;
    int    irowQ[]={};
    int    jcolQ[]={};
    double    dQ[]={};

    const int my = siz*2;
    double b[my];
    std::fill(b, b+my, 1);

    const int nnzA = siz*siz*2;
    int irowA[nnzA];
    int jcolA[nnzA];
    double dA[nnzA];
    for(int i=0; i<siz; i++){
        for(int j=0; j<siz; j++){
            irowA[i*siz+j] = i;
            jcolA[i*siz+j] = i*siz+j;
            dA[i*siz+j] = 1.0;
        }
    }
    for(int i=0; i<siz; i++){
        for(int j=0; j<siz; j++){
            irowA[siz*siz+j+i*siz] = siz+i;
            jcolA[siz*siz+j+i*siz] = j*siz+i;
            dA[siz*siz+j+i*siz] = 1;
        }
    }

    const int mz   = 0;
    double clow[]  = {};
    char  iclow[]  = {};
    double cupp[]  = {};
    char  icupp[]  = {};
    const int nnzC = 0;
    int   irowC[]  = {};
    int   jcolC[]  = {};
    double   dC[]  = {};

    QpGenSparseMa27 * qp
            = new QpGenSparseMa27( nx, my, mz, nnzQ, nnzA, nnzC );

    QpGenData      * prob = (QpGenData * ) qp->copyDataFromSparseTriple(
            c,      irowQ,  nnzQ,   jcolQ,  dQ,
            xlow,   ixlow,  xupp,   ixupp,
            irowA,  nnzA,   jcolA,  dA,     b,
            irowC,  nnzC,   jcolC,  dC,
            clow,   iclow,  cupp,   icupp );

    QpGenVars      * vars
            = (QpGenVars *) qp->makeVariables( prob );
    QpGenResiduals * resid
            = (QpGenResiduals *) qp->makeResiduals( prob );

    GondzioSolver  * s     = new GondzioSolver( qp, prob );
//    if( !quiet ) s->monitorSelf();

    int ierr = s->solve(prob,vars, resid);
    if( ierr == 0 ) {
        vars->x->copyIntoArray(xArray);
        for(int i=0; i<siz*siz; i++){
            if(xArray[i]>0.5){
                int row = i/siz;
                int col = i%siz;
                if(row>=originRow){
                    /*unmatched predictor*/
                    unmatchedP.push_back(col);
                }
                else if(col>=originCol){
                    unmatchedM.push_back(row);
                }
                else{
                    if(edges[row][col] > 0.3){
                        matches.emplace_back(row, col);
                    }
                    else{
                        unmatchedP.push_back(col);
                        unmatchedM.push_back(row);
                    }
                }
            }
        }
    }
    else{
        cout << "Could not solve the problem.\n";
        for(int i=0; i<originCol; i++)
            unmatchedP.push_back(i);
        for(int i=0; i<originRow; i++)
            unmatchedM.push_back(i);
    }
    return std::make_tuple(matches, unmatchedM, unmatchedP);
}

std::vector<std::vector<float>>  KM::generateSquareMatrx(std::vector<std::vector<float>>& edge){
    int rowSize = edge.size();
    int colSize;
    if(edge.size()>0)
        colSize= edge[0].size();
    else
        throw("!!! strange mei you measured");
    std::vector<std::vector<float>> squared;
    squared = edge;
    if(rowSize>colSize){
        for(int i=0; i<rowSize; i++){
            for(int j=0; j<rowSize-colSize; j++){
                squared[i].push_back(0.0);
            }
        }
    }
    else{
        for (int j = 0; j < colSize-rowSize; j++) {
            squared.push_back(vector<float>(colSize));
        }
    }
    return squared;
}







/*
 *     //for test, chage edges
    originRow = 4;
    originCol = 3;
    vector<vector<float>> edg;
    vector<float>tmp;
    tmp.push_back(-0.9);
    tmp.push_back(-0.9);
    tmp.push_back(-1);
    tmp.push_back(-1);
    tmp.push_back(-1);
    edg.push_back(tmp);
    tmp.clear();
    tmp.push_back(-0.8);
    tmp.push_back(-0.7);
    tmp.push_back(-0.1);
    tmp.push_back(-1);
    tmp.push_back(-1);
    edg.push_back(tmp);
    tmp.clear();
    tmp.push_back(-0.3);
    tmp.push_back(-0.8);
    tmp.push_back(-1.0);
    tmp.push_back(-1);
    tmp.push_back(-1);
    edg.push_back(tmp);
    tmp.clear();
    tmp.push_back(-0.6);
    tmp.push_back(-0.2);
    tmp.push_back(-0.5);
    tmp.push_back(-0.1);
    tmp.push_back(-0.2);
    edg.push_back(tmp);
    tmp.clear();


            cout.precision(4);
        cout << "Solution: \n";
        vars->x->writefToStream( cout, "x[%{index}] = %{value}" );

    */




/*
void Hungarian::selfSort(){
    std::sort(previous_frame_.begin(), previous_frame_.end(), myOrder);
    std::sort(current_frame_.begin(), current_frame_.end(), myOrder);
}

bool Hungarian::myOrder (Eigen::Vector2d i, Eigen::Vector2d j){
    return (i(1,0)<j(1,0));
}

double Hungarian::calcDist(const Eigen::Vector2d& pFrame, const Eigen::Vector2d& cFrame) {
    auto deltaX = pFrame(0, 0) - cFrame(0, 0);
    auto deltaY = pFrame(1, 0) - cFrame(1, 0);
    return  sqrt(pow(deltaX,2)+pow(deltaY,2));
}*/