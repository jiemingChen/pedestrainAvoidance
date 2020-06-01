//
// Created by jieming on 06.02.20.
//

#ifndef RGBD_DETECT_HUNGARIAN_H
#define RGBD_DETECT_HUNGARIAN_H

#include "YoloRecognizer.h"
#include "QpGenData.h"
#include "QpGenVars.h"
#include "QpGenResiduals.h"
#include "GondzioSolver.h"
#include "QpGenSparseMa27.h"
#include <climits>

using  std::vector;
using std::set;

class Hungarian {
    public:
        vector<vector<double>> edge;
        vector<bool> onPath;
        vector<int> path;
        int maxMatch;

        Hungarian(const vector<vector<double>>&, const int& xsize, const int& ysize);
        virtual ~Hungarian();
        void clearOnPathSign();
        bool FindAugPath( int xi);

        void Hungary_match();
        void outputRes();
};

class KM{
    public:
        KM(){};
        virtual ~KM(){};
        std::tuple<vector<pair<int, int>>,vector<int>,vector<int> > match(std::vector<std::vector<int>>&);
        static vector<vector<int>>  generateSquareMatrx(std::vector<std::vector<int>>&);
        vector<pair<int,int>> km(std::vector<std::vector<int>> edges, int siz);
    };

#endif //RGBD_DETECT_HUNGARIAN_H

