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

#include "grf_swarm.h"

class Metric
{
public:
    Metric(double robots, double groups, double threshold);
    double robots, groups, threshold;
    double norm(Vector2 v);
    double norm(double x1, double y1, double x2, double y2);
    double consensus_metric(std::vector<Robot> states);
    int miss_bounding_metric(std::vector<Robot> states);
    int number_of_molecules_metric(std::vector<Robot> states);
    int cluster_metric(std::vector<Robot> states);
};