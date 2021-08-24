#pragma once

#include <cmath>
#include <cstddef>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <chrono>
#include <random>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ctime>
#include <sys/time.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <boost/thread.hpp>
#include <boost/asio.hpp>

#include <omp.h>

// #define SHOW_VELOCITY
// #define SHOW_SENSING
// #define SHOW_OBSTACLES
#define SAVE_FIGURES
#define VISUALIZATION
// #define LOGGING


#define USE_NOISE
// #define V_CONTROL

class Vector2{
public:
	double x;
	double y;
};

class Robot{
public:
	Vector2 position;
	Vector2 velocity;
	double id;
	double type;
    unsigned int bound;
    bool anchor;
    bool bounded;
    double mass;
    std::vector<std::vector<unsigned int>> binding;
    std::vector<unsigned int> orbitals;
};


class ClusterMetric
{
public:
    ClusterMetric(double robots, double groups, double threshold);
    double robots, groups, threshold;
    int compute(std::vector<Robot> states);
};

class cmp {
    Robot r;
public:
    cmp(Robot r_) : r(r_) {}
    bool operator()(Robot a, Robot b) {
        // double f1 = 1.0 + fabs((r.bound-r.binding.size()) + (a.bound-a.binding.size()));
        // double f1 = 1.0 + fabs((r.bound) - (a.bound));
        double f1 = 1.0;
        double dist1 = this->euclidean(r.position, a.position)/f1;
        // double f2 = 1.0 + fabs((r.bound-r.binding.size()) + (b.bound-b.binding.size()));
        // double f2 = 1.0 + fabs((r.bound) - (b.bound));
        double f2 = 1.0;
        double dist2 = this->euclidean(r.position, b.position)/f2;
        return dist1 < dist2;
    }
    double euclidean(Vector2 a, Vector2 b){
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    return sqrt(dx * dx + dy * dy) + 1.0e-9;
}
};

class cmp2 {
    Robot r;
public:
    cmp2(Robot r_) : r(r_) {}
    bool operator()(const Robot& a) {
        return a.id == r.id;
    }
};

class Controller
{
public:
    // WiseRobot Constructor
    Controller(ros::NodeHandle *nodehandle);

    int robots;
    int groups;
    double sensing;
    double worldsize;
    double safezone;
    double mass;
    double vmax;
    double dt;
    double noise;

    int metric_v;
    float max_iteration;
    
    std::string worldfile;
    std::string logginfile;
    std::ofstream logfile;

    int seed;
    bool gui;
    bool logging;
    std::string swarmconf;

    std::vector<Robot> states;
    std::vector<Vector2> obstacles;
    boost::mutex mutex;


	void update(long iterations);
    bool draw(int step);


private:
    ros::NodeHandle nh_; // ROS Node Handle

    int countCollisions();
    double meanDist();
    // bool cmp(Robot& a, Robot& b, Robot& c);
    double consensusVel();

    double kineticEnergy(double v, double m);
    double coulombBuckinghamPotential(double r, double eplson, double eplson0, double r0, double alpha, double q1, double q2);
    double fof_Us(Robot r_i, Vector2 v);
    double fof_Ust(Robot r_i, Vector2 v, std::vector<Robot> states_t);
    double euclidean(Vector2 a, Vector2 b);
    std::vector<Vector2> getObstaclesPoints(double sensing, Vector2 p);
    bool getIntersection(double sensing, Vector2 circle, Vector2 p1, Vector2 p2, Vector2& o1, Vector2& o2);
    std::vector<std::vector<Robot>> getAllRobotsNeighborns(std::vector<Robot> agents);
    Vector2 saturation(Vector2 v, double norm);
    Vector2 metropolisHastings(Robot r_i, std::vector<Robot> states_t);
    void updateBinding(Robot& r_i, std::vector<Robot> states_t);
};


