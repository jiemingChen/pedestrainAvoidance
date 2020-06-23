//
// Created by jieming on 25.05.20.
//

#include "pubMsg.h"
#include "collision_avoidance/header.h"
#include "Config.h"
using std::vector;
//  marker id  obstacles0-20, mc horizion, -------  terminal point 1000
void pubTerminal( const ros::Publisher& vis_pub,  vector<double> terminal){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time();
    marker.ns = "/";
    marker.id = 7000;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.pose.position.x = terminal[0];
    marker.pose.position.y = terminal[1];
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.4;
    marker.scale.z = 0.3;
    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    vis_pub.publish( marker );
}

void pubObstacles( const ros::Publisher&  marker_array_pub,  vector<vector<float>> obstacles){
    visualization_msgs::Marker marker;

    marker.action = visualization_msgs::Marker::DELETEALL;
    visualization_msgs::MarkerArray array_del;
    array_del.markers.push_back(marker);
    marker_array_pub.publish(array_del);

    visualization_msgs::MarkerArray array;
    array.markers.clear();
    for(auto i=0; i<obstacles.size(); i++){
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time();
        marker.ns = "/";
        marker.id = obstacles[i][2];
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = obstacles[i][0];
        marker.pose.position.y = obstacles[i][1];
        marker.pose.position.z = 0;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 0.5;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        array.markers.push_back(marker);
    }
    for(auto i=0; i<obstacles.size(); i++){
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time();
        marker.ns = "/";
        marker.id = obstacles[i][2]+obstacles.size();
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = obstacles[i][0];
        marker.pose.position.y = obstacles[i][1];
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
//        marker.scale.x = Config::get<float>("safe_dist")*2;
//        marker.scale.y = Config::get<float>("safe_dist")*2;
        marker.scale.x = 1.5*2;
        marker.scale.y = 1.5*2;
        marker.scale.z = 0.1;
        marker.color.a = 0.3;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        array.markers.push_back(marker);
    }
    marker_array_pub.publish(array);
}

void pubTraj(const ros::Publisher& vis_pub, int idx, vector<float>state){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time();
    marker.ns = "/";
    marker.id = idx;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = state[0];
    marker.pose.position.y = state[1];
    marker.pose.position.z = 0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    vis_pub.publish( marker );
}

void pubTraj2(const ros::Publisher& marker_array_pub,  vector<float>&state){
    visualization_msgs::MarkerArray array;
    array.markers.clear();
    visualization_msgs::Marker  marker;

    Eigen::AngleAxisf rollAngle(0, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(0, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(0, Eigen::Vector3f::UnitZ());
    Eigen::Quaternionf q;
//    auto n = Config::get<float>("N_");

    for(auto i=0; i<(state.size()-2)/3; i++){
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time();
        marker.ns = "/";
        marker.id = 5000+i;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = state[2+i*3];
        marker.pose.position.y = state[3+i*3];
        marker.pose.position.z = 0;

//        yawAngle.angle() = state[4+i*3];
        yawAngle.angle() = state[2+i*3];
        q = yawAngle * pitchAngle * rollAngle;
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
        marker.scale.x = 0.4;
        marker.scale.y = 0.4;
        marker.scale.z = 0.4;
        marker.color.a = 0.5;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        array.markers.push_back(marker);
    }
    marker_array_pub.publish(array);
}

void pubCommand(const ros::Publisher& cmd_pub, const vector<float>& data){
    geometry_msgs::Twist cmd;
//    std::cout<<data[0]<<"!!!!!!!!!!!!!!!"<<std::endl;

    cmd.linear.x = data[0];
    cmd.linear.y = 0;
    cmd.linear.z = 0;
    cmd.angular.x = 0;
    cmd.angular.y = 0;
    cmd.angular.z = -data[1];
    cmd_pub.publish(cmd);
}