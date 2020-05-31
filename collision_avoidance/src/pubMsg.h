//
// Created by jieming on 25.05.20.
//

#ifndef COLLISION_AVOIDANCE_PUBMSG_H
#define COLLISION_AVOIDANCE_PUBMSG_H

//  marker id  obstacles0-20, mc horizion, -------  terminal point 1000
#include "collision_avoidance/header.h"
using std::vector;

void pubTerminal( const ros::Publisher& vis_pub,  vector<double> terminal);

void pubObstacles( const ros::Publisher&  marker_array_pub,  vector<vector<float>> obstacles);

void pubTraj(const ros::Publisher& vis_pub, int idx, vector<float>state);

void pubTraj2(const ros::Publisher& marker_array_pub,  vector<float>&state);

void pubCommand(const ros::Publisher& cmd_pub, const vector<float>& data);

#endif //COLLISION_AVOIDANCE_PUBMSG_H
