#include "grf_swarm.h"

Controller::Controller(ros::NodeHandle *nodehandle) : nh_(*nodehandle)
{ // constructor
    /* Get params */
    int flockSize;
    this->seed = 1;
    this->gui = false;
    ros::param::get("~nrobots", this->robots);
    ros::param::get("~ngroups", this->groups);
    ros::param::get("~seed", this->seed);
    ros::param::get("~gui", this->gui);
    flockSize = this->robots / this->groups;

    this->sensing = 0.5;
    this->safezone = 0.3;
    ros::param::get("~sensing", this->sensing);

    ros::param::get("~safezone", this->safezone);

    this->worldsize = 5.0;
    ros::param::get("~worldsize", this->worldsize);

    this->max_iteration = 20000;
    ros::param::get("~iterations", this->max_iteration);
    // std::cout << max_iteration << std::endl;

    ros::param::get("~swarmconf", this->swarmconf);
    // std::cout << swarmconf << std::endl;

    this->logging = false;
    ros::param::get("~log", this->logging);
    if (this->logging)
    {
        this->logginfile = "min_swarm_r" + std::to_string(this->robots) + "_g_" + std::to_string(this->groups) + "_s_" + std::to_string(this->sensing) + "_w_" + std::to_string(this->worldsize) + "_run_" + std::to_string(this->seed) + ".log";
    }
    std::cout << this->logginfile << std::endl;

    this->mass = 0.3;
    this->vmax = 1.0;
    this->dt = 0.02;

    ROS_INFO("%d %d %f", this->robots, this->groups, this->sensing);
    if (this->logging)
    {
        this->logfile.open(this->logginfile);
        ROS_INFO("\33[92mLog: %s\33[0m", logginfile.c_str());
    }
    std::istringstream ss(this->swarmconf);

    int NUM = 18;
    int count = 0;
    for (int i = 0; i < this->robots; ++i)
    {
        Robot r;
        float x, y, vx, vy, type;
        ss >> x;
        ss >> y;
        ss >> vx;
        ss >> vy;
        ss >> type;
        // Get initial positions
        r.position.x = x;
        r.position.y = y;

        // Get initial velocities
        r.velocity.x = vx;
        r.velocity.y = vy;

        r.anchor = false;
        r.type = type;
        r.id = i;
        // r.bound = (unsigned int)(r.type + 1);
        r.bounded = false;

        //         if (r.type == 0)
        // {
        //     r.bound = 2;
        //     r.orbitals.push_back(2); // Number of H
        //     r.orbitals.push_back(1); // Number of C
        //     r.orbitals.push_back(0); // Number of S
        //     r.mass = 0.03;           //0.3;
        // }
        // else
        // {
        //     r.bound = 2;
        //     r.orbitals.push_back(2); // Number of H
        //     r.orbitals.push_back(0); // Number of C
        //     r.orbitals.push_back(0); // Number of S
        //     r.mass = 0.04;           //0.4;
        // }

        switch ((int)r.type)
        {
        case 0: /* H */
            // r.bound = 1;
            // r.orbitals.push_back(1); // Number of H
            // r.orbitals.push_back(1); // Number of N
            // r.orbitals.push_back(1); // Number of C
            // r.mass = 0.1;            // Mass H
            // r.radius = 53;           // Radius of H

            r.bound = 2;             /* O */
            r.orbitals.push_back(2); // Number of O
            r.orbitals.push_back(1); // Number of C
            r.mass = 1.6;            // Mass O
            r.radius = 60;           // Radius of O
            break;

        case 1:
            // r.bound = 3;             /* N */
            // r.orbitals.push_back(3); // Number of H
            // r.orbitals.push_back(0); // Number of N
            // r.orbitals.push_back(1); // Number of C
            // r.mass = 1.4;            // Mass O
            // r.radius = 65;           // Radius of O

            // r.bound = 2;             /* O */
            // r.orbitals.push_back(2); // Number of H
            // r.orbitals.push_back(0); // Number of N
            // r.mass = 1.6;            // Mass O
            // r.radius = 60;           // Radius of O

            r.bound = 4;             /* C */
            r.orbitals.push_back(2); // Number of H
            r.orbitals.push_back(0); // Number of N
            // r.orbitals.push_back(2); // Number of C
            r.mass = 1.2;            // Mass O
            r.radius = 70;           // Radius of O
            break;

            // case 2:
            //     r.bound = 4;             /* C */
            //     r.orbitals.push_back(2); // Number of H
            //     r.orbitals.push_back(1); // Number of N
            //     r.orbitals.push_back(2); // Number of C
            //     r.mass = 1.2;            // Mass O
            //     r.radius = 70;           // Radius of O
            //     // r.bound = 2;             /* O */
            //     // r.orbitals.push_back(2); // Number of H
            //     // r.orbitals.push_back(0); // Number of N
            //     // r.mass = 1.6;            // Mass O
            //     // r.radius = 60;           // Radius of O
            //     break;

            // case 2:
            //     r.bound = 1;             /* C */
            //     r.orbitals.push_back(0); // Number of H
            //     r.orbitals.push_back(1); // Number of N
            //     r.orbitals.push_back(0); // Number of C
            //     r.mass = 0.5;            //0.4;
            //     r.position.x = (6.0 * count / (NUM-1) - 6.0 / 2.0);
            //     r.position.y = fabs(14.0 * count / (NUM-1) - 14.0 / 2.0) - 3;
            //     r.anchor = true;
            //     r.velocity.x = 0;
            //     r.velocity.y = 0;
            //     count++;
            //     break;

            // default:
            //     break;
        }

        for (int k = 0; k < r.orbitals.size(); k++)
        {
            std::vector<unsigned int> t_;
            r.binding.push_back(t_);
        }
        this->states.push_back(r);
    }

    if (this->gui)
    {
        cv::namedWindow("GRF-Pattern-Formation", cv::WINDOW_AUTOSIZE);
    }
}

bool Controller::draw(int step)
{
// Create board
#define W_X 1000
#define W_Y 1000

    cv::Mat board(W_Y, W_X, CV_8UC3, cv::Scalar(255, 255, 255));
    // cv::Mat board = cv::imread("/home/rezeck/catkin_ws/src/2022_icra_grf_pattern_formation/world/background.png", cv::IMREAD_COLOR);
    cv::Scalar color;
    cv::rectangle(board, cv::Point(50, 50), cv::Point(W_X - 50, W_Y - 50), cv::Scalar(80, 80, 80), 4, 8);

    // cv::putText(board, std::to_string(step), cv::Point(board.cols / 2, 35), cv::FONT_HERSHEY_DUPLEX,
    //             0.6, CV_RGB(0, 0, 0), 1);
    cv::putText(board, "Steps: " + std::to_string(step), cv::Point(50, 35), cv::FONT_HERSHEY_TRIPLEX,
                0.6, CV_RGB(80, 80, 80), 1);
    // cv::putText(board, "Steps: " + std::to_string(this->metric_v), cv::Point(50, 35), cv::FONT_HERSHEY_DUPLEX,
    // 0.6, CV_RGB(0, 0, 0), 1);

    // float c = 300.0 / 5.0;
    float c = (W_Y - 100) / 10.0;

#ifdef SHOW_VELOCITY
    for (int i = 0; i < this->robots; i++)
    {
        Vector2 vel;
        vel = this->saturation(this->states[i].velocity, 0.3);
        vel.x = W_X / 2.0 + c * (this->states[i].position.x + vel.x);
        vel.y = W_Y / 2.0 - c * (this->states[i].position.y + vel.y);
        cv::arrowedLine(board, cv::Point(W_X / 2.0 + c * this->states[i].position.x, W_Y / 2.0 - c * this->states[i].position.y), cv::Point(vel.x, vel.y), cv::Scalar(220, 220, 220), 2, 8);
    }
#endif

#ifdef SHOW_SENSING
    for (int i = 0; i < this->robots; i++)
    {
        cv::circle(board, cv::Point(W_X / 2.0 + c * this->states[i].position.x, W_Y / 2.0 - c * this->states[i].position.y), c * this->sensing, cv::Scalar(240, 240, 240), 1, 8);
    }
#endif

    for (int i = 0; i < this->robots; i++)
    {
        switch ((int)this->states[i].type)
        { // BGR
        case 0:
            color = cv::Scalar(0, 0, 128);
            break; // maroon
        case 1:
            color = cv::Scalar(128, 0, 0);
            break; // dark slate gray
        case 2:
            // color = cv::Scalar(138, 43, 226);
            color = cv::Scalar(128, 0, 0);
            break; // blue violet
        case 3:
            color = cv::Scalar(199, 21, 133);
            break; // medium violet red
        case 4:
            color = cv::Scalar(144, 238, 144);
            break; // light green
        case 5:
            color = cv::Scalar(255, 215, 0);
            break; // gold
        case 6:
            color = cv::Scalar(218, 165, 32);
            break; // golden rod
        case 7:
            color = cv::Scalar(189, 183, 107);
            break; // dark khaki
        case 8:
            color = cv::Scalar(128, 128, 0);
            break; // olive
        case 9:
            color = cv::Scalar(154, 205, 50);
            break; // yellow green
        case 10:
            color = cv::Scalar(107, 142, 35);
            break; // olive drab
        case 11:
            color = cv::Scalar(127, 255, 0);
            break; // chart reuse
        case 12:
            color = cv::Scalar(0, 100, 0);
            break; // dark green
        case 13:
            color = cv::Scalar(255, 140, 0);
            break; // dark orange
        case 14:
            color = cv::Scalar(46, 139, 87);
            break; // sea green
        case 15:
            color = cv::Scalar(102, 205, 170);
            break; // medium aqua marine
        case 16:
            color = cv::Scalar(220, 20, 60);
            break; // crimson
        case 17:
            color = cv::Scalar(0, 139, 139);
            break; // dark cyan
        case 18:
            color = cv::Scalar(0, 255, 255);
            break; // cyan
        case 19:
            color = cv::Scalar(70, 130, 180);
            break; // steel blue
        case 20:
            color = cv::Scalar(100, 149, 237);
            break; // corn flower blue
        case 21:
            color = cv::Scalar(30, 144, 255);
            break; // dodger blue
        case 22:
            color = cv::Scalar(0, 0, 128);
            break; // navy
        case 23:
            color = cv::Scalar(240, 128, 128);
            break; // light coral
        case 24:
            color = cv::Scalar(75, 0, 130);
            break; // indigo
        case 25:
            color = cv::Scalar(139, 0, 139);
            break; // dark magenta
        case 26:
            color = cv::Scalar(238, 130, 238);
            break; // violet
        case 27:
            color = cv::Scalar(255, 160, 122);
            break; // light salmon
        case 28:
            color = cv::Scalar(255, 105, 180);
            break; // hot pink
        case 29:
            color = cv::Scalar(112, 128, 144);
            break; // slate gray
        default:
            color = cv::Scalar(0, 0, 0);
            break; // black
        }
        std::swap(color[0], color[2]);
        // cv::circle(board, cv::Point(W_X / 2.0 + c * this->states[i].position.x, W_Y / 2.0 - c * this->states[i].position.y), c * 0.14, color, -1, 8);
        // if (this->states[i].type == 2)
        // {
        // cv::circle(board, cv::Point(W_X / 2.0 + c * this->states[i].position.x, W_Y / 2.0 - c * this->states[i].position.y), c * 0.01 * 3, color, -1, 8);
        // continue;
        // }

        cv::circle(board, cv::Point(W_X / 2.0 + c * this->states[i].position.x, W_Y / 2.0 - c * this->states[i].position.y), c * this->states[i].radius * 0.0021, color, -1, 8);
        // cv::circle(board, cv::Point(350 + c * this->states[i].position.x, 350 - c * this->states[i].position.y), c * 0.07, color, -1, 8);
        // for (int k = 0; k < this->states[i].binding.size(); k++)
        // {
        //     for (int w = 0; w < this->states[i].binding[k].size(); w++)
        //     {
        //         cv::line(board, cv::Point(W_X / 2.0 + c * this->states[i].position.x, W_Y / 2.0 - c * this->states[i].position.y), cv::Point(W_X / 2.0 + c * this->states[this->states[i].binding[k][w]].position.x, W_Y / 2.0 - c * this->states[this->states[i].binding[k][w]].position.y), cv::Scalar(112, 128, 144), 1);
        //     }
        // }
#ifdef SHOW_ID
        cv::putText(board, std::to_string((int)this->states[i].id), cv::Point(W_X / 2.0 + c * this->states[i].position.x + 4, W_Y / 2.0 - c * this->states[i].position.y - 4), cv::FONT_HERSHEY_DUPLEX,
                    0.4, CV_RGB(0, 0, 0), 1);
#endif
    }

#ifdef SHOW_OBSTACLES
    for (int i = 0; i < this->obstacles.size(); i++)
    {
        cv::circle(board, cv::Point(W_X / 2.0 + c * this->obstacles[i].x, W_Y / 2.0 - c * this->obstacles[i].y), c * 0.05, cv::Scalar(0, 0, 255), -1, 8);
    }
    this->obstacles.clear();
#endif

    cv::imshow("GRF-Pattern-Formation", board);
#ifdef SAVE_FIGURES
    char filenanme[17];
    std::sprintf(filenanme, "image_%06d.png", step);
    cv::imwrite(filenanme, board);
#endif
    return (cv::waitKey(1) != 27);
}

double Controller::kineticEnergy(double v, double m)
{
    return 0.5 * v * m;
}

double Controller::coulombBuckinghamPotential(double r, double eplson, double eplson0, double r0, double alpha, double q1, double q2)
{
    // Compute the Coulomb-Buckingham Potential
    return eplson * ((6.0 / (alpha - 6.0)) * exp(alpha) * (1.0 - r / r0) - (alpha / (alpha - 6.0)) * std::pow(r0 / r, 6)) + (q1 * q2) / (4.0 * M_PI * eplson0 * r);
}

double Controller::fof_Us(Robot r_i, Vector2 v)
{
    // Simulated (kinematic model of the robot) the motion of the robot using the sampled velocity
    r_i.position.x = r_i.position.x + v.x * this->dt;
    r_i.position.y = r_i.position.y + v.y * this->dt;
    // Get potential for the sampled velocity
    double Us = 0.0f;
    // for each obstacles point in the world (same when using a laser)
    std::vector<Vector2> obstacles = this->getObstaclesPoints(this->safezone, r_i.position);
    // ROS_INFO("Number of obstacles: %d", obstacles.size());
    for (int i = 0; i < obstacles.size(); ++i)
    {
#ifdef SHOW_OBSTACLES
        this->mutex.lock();
        this->obstacles.push_back(obstacles[i]);
        this->mutex.unlock();
#endif
        // ROS_INFO("Number of obstacles: %f %f", obstacles[i].x, obstacles[i].y);
        // lets compute the distance to the obstacle (we only use the distance)
        double dist = this->euclidean(r_i.position, obstacles[i]);
        // if (dist <= (this->safezone*2.0)){
        Us += this->coulombBuckinghamPotential(dist / 2.0, 0.04, 0.04, 0.8, 1.0, 16.0, 1.0);
        // }
    }
    return Us;
}

double Controller::fof_Ust(Robot r_i, Vector2 v, std::vector<Robot> states_t)
{
    // Simulated (kinematic model of the robot) the motion of the robot using the sampled velocity
    r_i.position.x = r_i.position.x + v.x * this->dt;
    r_i.position.y = r_i.position.y + v.y * this->dt;
    // Get the sum of the relative velocity of all my neighbor and they mass
    Vector2 group_vrel;
    double group_mass = r_i.mass; //this->mass;
    // Get the pairwise potential for the sampled velocity
    double Ust = 0.0f;

    // #ifdef _OPENMP
    // #pragma omp parallel for
    // #endif

    // for each neighborn in current state
    // Neighborns should be already sorted by distance
    for (int i = 0; i < states_t.size(); ++i)
    {
        Vector2 n_p;
        n_p.x = states_t[i].position.x + states_t[i].velocity.x * this->dt;
        n_p.y = states_t[i].position.y + states_t[i].velocity.y * this->dt;
        double dist = this->euclidean(r_i.position, n_p);
        // double dist = this->euclidean(r_i.position, states_t[i].position);
        // Indicator function f: 1 -> same type, f: -1 -> otherwise
        double I = 2.0 * (int)(r_i.type == states_t[i].type) - 1.0;

        I = -0.002;
        // I = 0.2;
        dist = dist * 0.80;

        // if (states_t[i].type == r_i.type)
        // {
        //     if (r_i.binding[1].size() > 0 && states_t[i].binding[1].size() > 0)
        //         if (r_i.binding[1][0] == states_t[i].binding[1][0])
        //         {
        //             I = 0.8;
        //         }
        // }

        /* For each orbit (k) in the robot (r_i). */
        for (int k = 0; k < r_i.binding.size(); k++)
        {
            /* For each robot (y) in the orbit (k). */
            for (int y = 0; y < r_i.binding[k].size(); y++)
            {
                if (states_t[i].id == r_i.binding[k][y])
                {
                    if (states_t[i].anchor)
                    {
                        I = 16;
                        // dist = dist * 1.5;
                    }
                    else if (states_t[i].bounded == r_i.bounded)
                    {
                        I = 4;
                        // dist = dist * 1.4;
                    }
                    else
                    {
                        I = 3;
                        // dist = dist * 1.3;
                    }
                }
            }
        }

        // if (r_i.type == 1 && states_t[i].type == 2)
        // {
        //     dist = dist * 1.90;
        // }

        if ((I > 0) && (dist > this->sensing))
        {
            if (states_t[i].anchor)
            {
                return 100000;
            }
            else
            {
                return 10000;
            }
        }

        Ust += this->coulombBuckinghamPotential(dist * 0.8, 0.04, 0.04, 0.8, 1.0, 16.0 * I, -1.0);

        // Get the sum of the relative velocity of all my neighbor and they mass
        if (I > 0 && (dist < this->sensing) && (dist > this->safezone))
        {
            group_vrel.x += (states_t[i].velocity.x - v.x);
            group_vrel.y += (states_t[i].velocity.y - v.y);
            group_mass += states_t[i].mass;
        }
    }
    // Now compute the kinetic Energy using relative velocity
    group_vrel = this->saturation(group_vrel, 1.0);
    double group_speed = (group_vrel.x * group_vrel.x + group_vrel.y * group_vrel.y) + 1.0e-9;
    double my_speed = (v.x * v.x + v.y * v.y);
    // double group_speed = sqrt(group_vrel.x*group_vrel.x + group_vrel.y*group_vrel.y) + 1.0e-9;
    // double my_speed = sqrt(v.x*v.x + v.y*v.y);

    Ust += this->kineticEnergy(group_speed, group_mass) + this->kineticEnergy(group_mass, this->vmax - my_speed);
    // Ust += this->kineticEnergy(group_speed, group_mass) - this->kineticEnergy(group_mass, my_speed);
    return Ust;
}

bool Controller::getIntersection(double r, Vector2 circle, Vector2 p1, Vector2 p2, Vector2 &o1, Vector2 &o2)
{
    // Convert p1 and p2 to be relative to circle; circle -> (0,0)
    p1.x -= circle.x;
    p1.y -= circle.y;

    p2.x -= circle.x;
    p2.y -= circle.y;

    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double dr = sqrt(dx * dx + dy * dy);
    double D = p1.x * p2.y - p2.x * p1.y;
    double disc = (r * r) * (dr * dr) - (D * D);
    double sgn_dy = (dy >= 0) - (dy < 0);

    if (disc > 0.0)
    {
        o1.x = (D * dy + sgn_dy * dx * sqrt(disc)) / (dr * dr);
        o2.x = (D * dy - sgn_dy * dx * sqrt(disc)) / (dr * dr);
        o1.y = (-D * dx + fabs(dy) * sqrt(disc)) / (dr * dr);
        o2.y = (-D * dx - fabs(dy) * sqrt(disc)) / (dr * dr);

        o1.x += circle.x;
        o2.x += circle.x;
        o1.y += circle.y;
        o2.y += circle.y;
        return true;
    }
    return false;
}

std::vector<std::vector<Robot>> Controller::getAllRobotsNeighborns(std::vector<Robot> agents)
{
    std::vector<std::vector<Robot>> neighbors;
    // #pragma omp parallel for
    for (int i = 0; i < this->robots; ++i)
    {
        std::vector<Robot> ri;
        neighbors.push_back(ri);
    }

    // #pragma omp parallel for
    for (int i = 0; i < (this->robots - 1); ++i)
    {
        for (int j = i + 1; j < this->robots; ++j)
        {
            double dist = this->euclidean(agents[i].position, agents[j].position);
            if (dist <= this->sensing)
            {
                neighbors[j].push_back(agents[i]);
                neighbors[i].push_back(agents[j]);
            }
        }
    }

#define SORTING_NEIGHBORNS
#ifdef SORTING_NEIGHBORNS
    for (int i = 0; i < this->robots; ++i)
    {
        // Sort using comparator function
        std::sort(neighbors[i].begin(), neighbors[i].end(), cmp(agents[i]));
    }
#endif

    return neighbors;
}

std::vector<Vector2> Controller::getObstaclesPoints(double sensing, Vector2 p)
{
    /* World
             p3 o------o p4
                |      |                            
                |      |
             p1 o------o p2
    */
    Vector2 p1;
    p1.x = -this->worldsize;
    p1.y = -this->worldsize;
    Vector2 p2;
    p2.x = this->worldsize;
    p2.y = -this->worldsize;
    Vector2 p3;
    p3.x = -this->worldsize;
    p3.y = this->worldsize;
    Vector2 p4;
    p4.x = this->worldsize;
    p4.y = this->worldsize;

    std::vector<Vector2> obstacles;

    Vector2 out1, out2;
    double res = 0.1;
    if (this->getIntersection(sensing, p, p1, p2, out1, out2))
    {
        obstacles.push_back(out1);
        obstacles.push_back(out2);
        // printf("12: (%f %f) (%f %f)\n", out1.x, out1.y, out2.x, out2.y);
        for (double i = std::min(out1.x, out2.x) + res; i < std::max(out1.x, out2.x); i += res)
        {
            Vector2 p;
            p.x = i;
            p.y = out1.y;
            obstacles.push_back(p);
        }
    }
    if (this->getIntersection(sensing, p, p1, p3, out1, out2))
    {
        obstacles.push_back(out1);
        obstacles.push_back(out2);
        // printf("13: (%f %f) (%f %f)\n", out1.x, out1.y, out2.x, out2.y);
        for (double i = std::min(out1.y, out2.y) + res; i < std::max(out1.y, out2.y); i += res)
        {
            Vector2 p;
            p.x = out1.x;
            p.y = i;
            obstacles.push_back(p);
        }
    }
    if (this->getIntersection(sensing, p, p2, p4, out1, out2))
    {
        obstacles.push_back(out1);
        obstacles.push_back(out2);
        // printf("24: (%f %f) (%f %f)\n", out1.x, out1.y, out2.x, out2.y);
        for (double i = std::min(out1.y, out2.y) + res; i < std::max(out1.y, out2.y); i += res)
        {
            Vector2 p;
            p.x = out1.x;
            p.y = i;
            obstacles.push_back(p);
        }
    }
    if (this->getIntersection(sensing, p, p3, p4, out1, out2))
    {
        obstacles.push_back(out1);
        obstacles.push_back(out2);
        // printf("34: (%f %f) (%f %f)\n", out1.x, out1.y, out2.x, out2.y);
        for (double i = (std::min(out1.x, out2.x) + res); i < std::max(out1.x, out2.x); i += res)
        {
            Vector2 p;
            p.x = i;
            p.y = out1.y;
            obstacles.push_back(p);
        }
    }
    return obstacles;
}

/* Compute the 2D Euclidean Distance between two points */
double Controller::euclidean(Vector2 a, Vector2 b)
{
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    return sqrt(dx * dx + dy * dy) + 1.0e-9;
}

/* Saturate the velocity of the robots to the value norm */
Vector2 Controller::saturation(Vector2 v, double norm)
{
    Vector2 vnorm;
    double r = fabs(v.y / v.x);
    if ((r >= 1.0) && (fabs(v.y) > norm))
    {
        vnorm.y = (v.y * norm) / fabs(v.y);
        vnorm.x = (v.x * norm) / (r * fabs(v.x));
    }
    else if ((r < 1.0) && (fabs(v.x) > norm))
    {
        vnorm.x = (v.x * norm) / fabs(v.x);
        vnorm.y = (v.y * norm * r) / (fabs(v.y));
    }
    else
    {
        vnorm.x = v.x;
        vnorm.y = v.y;
    }
    return vnorm;
}

Vector2 Controller::metropolisHastings(Robot r_i, std::vector<Robot> states_t)
{
    std::vector<Vector2> mcmc_chain;
    mcmc_chain.push_back(r_i.velocity);

    std::vector<double> chain_potential;
    chain_potential.push_back(this->fof_Us(r_i, r_i.velocity) +
                              this->fof_Ust(r_i, r_i.velocity, states_t));

    // Random Seed for normal distribution
    unsigned seed_x = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator_x(seed_x);
    unsigned seed_y = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator_y(seed_y);

    /* Test using prefered velocty on normal dist*/
    // Vector2 prefVelocity;
    // // prefVelocity = this->saturation(r_i.position, 1.0);
    // for (int i = 0; i < states_t.size(); ++i){
    //     double dist = this->euclidean(r_i.position, states_t[i].position);
    //     // Indicator function f: 1 -> same type, f: -1 -> otherwise
    //     double I = 2.0 * (int)(r_i.type == states_t[i].type) - 1.0;
    //     if ((I > 0) && (dist > 0.5*this->sensing)){
    //         prefVelocity.x += states_t[i].velocity.x;
    //         prefVelocity.y += states_t[i].velocity.y;
    //     }
    // }
    // prefVelocity.x = -prefVelocity.x*1.0 + 0.8*r_i.velocity.x + 0*(-r_i.position.x);
    // prefVelocity.y = -prefVelocity.y*1.0 + 0.8*r_i.velocity.y + 0*(-r_i.position.y);
    // prefVelocity = this->saturation(prefVelocity, this->vmax);

    for (int i = 0; i < 100; i++)
    {
        // Get the last potential simulated
        Vector2 last_v;
        last_v.x = mcmc_chain.back().x;
        last_v.y = mcmc_chain.back().y;
        double last_U = chain_potential.back();
        // Get a sample of velocity considering normal distribution
        // std::normal_distribution<double> norm_vx(r_i.type - r_i.position.x, 0.2);
        // std::normal_distribution<double> norm_vy(r_i.type - r_i.position.y, 0.2);
        std::normal_distribution<double> norm_vx(r_i.velocity.x, 0.2);
        std::normal_distribution<double> norm_vy(r_i.velocity.y, 0.2);
        Vector2 sampled_vel;
        sampled_vel.x = norm_vx(generator_x);
        sampled_vel.y = norm_vy(generator_y);
        sampled_vel = this->saturation(sampled_vel, this->vmax);
        // Compute the potential for the sampled velocity
        double U = this->fof_Us(r_i, sampled_vel) + this->fof_Ust(r_i, sampled_vel, states_t);
        // Accept the sampled velocity over the gibbs distribuition sampling
        double dE = U - last_U;
        double grf = exp(-dE);
        double r = ((double)rand() / ((double)(RAND_MAX) + (double)(1)));
        if ((dE < 0.0) || (r < grf))
        {
            mcmc_chain.push_back(sampled_vel);
            chain_potential.push_back(U);
        }
        else
        {
            mcmc_chain.push_back(last_v);
            chain_potential.push_back(last_U);
        }
    }
    // Discard first half of MCMC chain and thin out the rest.
    Vector2 new_velocity;
    new_velocity.x = 0.0;
    new_velocity.y = 0.0;
    double n = 0.0;
    int burnin = (int)(0.6 * mcmc_chain.size());
    for (int i = burnin; i < mcmc_chain.size(); i++)
    {
        new_velocity.x += mcmc_chain[i].x;
        new_velocity.y += mcmc_chain[i].y;
        n += 1.0;
    }
    new_velocity.x = new_velocity.x / n;
    new_velocity.y = new_velocity.y / n;
    /* Move the robot to next position assuming the new velocity */
    new_velocity = this->saturation(new_velocity, this->vmax);

    if (!r_i.anchor)
    {
        r_i.velocity = new_velocity;
        r_i.position.x += new_velocity.x * this->dt;
        r_i.position.y += new_velocity.y * this->dt;
    }
    this->states[(int)r_i.id] = r_i;

    return new_velocity;
}

bool special_compare(const Robot &a, const Robot &b)
{
    return a.id == b.id;
}

void Controller::updateBinding(Robot &r_i, std::vector<Robot> states_t)
{
    /* Create a empty orbital structure - It will be the new r_i orbital structure. */
    std::vector<std::vector<int>> bound;
    for (int orb = 0; orb < r_i.orbitals.size(); orb++)
    {
        std::vector<int> t_;
        bound.push_back(t_);
    }

    double orbitalDist = this->safezone;
    /* For each neighborn (n_j) of the robot (r_i): */
    /* Neighborns list (states_t) are already sorted by distance. */
    for (int i = 0; i < states_t.size(); i++)
    {
        Robot n_j = states_t[i];
        /* Euclean distance from neighborn i */
        double dist = this->euclidean(r_i.position, n_j.position);
#ifdef DEBUG_BINDING
        std::cout << "- Evaluating robot # " << n_j.id << ": " << (int)!(dist > orbitalDist);
#endif

        /* If the neighborn (r_j) is out the orbital distance: stop searching. */
        if (dist > orbitalDist)
        {
            break;
        }

        /* Check if (n_j) has a room for the robot r_i */
        bool isthereanyroom = false;
        unsigned int nbindingsofar = 0;
        /* For each orbital in the of the neighborn (n_j) */
        for (int k = (n_j.binding.size() - 1); k >= 0; k--)
        {
            /* Is k the r_i robot orbital? */
            if (k == r_i.type)
            {
                // is there any room for me?
                isthereanyroom = ((n_j.binding[k].size() + nbindingsofar) < n_j.orbitals[k]);
                // if (r_i.type == 1 && !isthereanyroom)
                // {
                //     isthereanyroom = ((n_j.binding[k + 1].size() + nbindingsofar) < n_j.orbitals[k + 1]);
                // }
                break;
            }
            nbindingsofar += n_j.binding[k].size();
        }
#ifdef DEBUG_BINDING
        std::cout << " |  " << (int)isthereanyroom;
#endif

        /* Check if (n_j) has r_i in the binding list. There are already connected? */
        bool areweconnected = false;
        /* For each orbital in the of the neighborn (n_j) */
        for (int k = (n_j.binding.size() - 1); k >= 0; k--)
        {
            if (areweconnected)
                break;
            /* For each robot (y) is the orbital k */
            for (int y = 0; y < n_j.binding[k].size(); y++)
            {
                /* Is it me? */
                if ((n_j.binding[k][y]) == r_i.id)
                {
                    areweconnected = true;
                    break;
                }
            }
        }
#ifdef DEBUG_BINDING
        std::cout << " |  " << (int)areweconnected;
#endif

        /* Check if connecting n_j with r_i create inner cycle. */
        bool cycledetected = false;
        /* For each orbital (k) in the neighborn (n_j) */
        for (int k = (n_j.binding.size() - 1); k >= 0; k--)
        {
            if (cycledetected)
                break;
            /* For each robot (y) is the orbital (k) */
            for (int y = 0; y < n_j.binding[k].size(); y++)
            {
                /* For each orbital (m) in the robot (r_i) */
                for (int m = (bound.size() - 1); m >= 0; m--)
                {
                    /* For each robot (x) in the orbital (m) */
                    for (int w = 0; w < bound[m].size(); w++)
                    {
                        if (n_j.binding[k][y] == bound[m][w])
                        {
                            cycledetected = true;
                        }
                    }
                }
            }
        }
#ifdef DEBUG_BINDING
        std::cout << " |  " << (int)cycledetected << std::endl;
#endif

        // bool bridging = true && (n_j.anchors.size() > 0) && (r_i.anchors.size() > 0);
        // for (int s = 0; s < n_j.anchors.size(); s++)
        // {
        //     for (int t = 0; t < r_i.anchors.size(); t++)
        //     {
        //         if (n_j.anchors[s] == r_i.anchors[t])
        //         {
        //             bridging = false;
        //         }
        //     }
        // }

        if ((areweconnected || isthereanyroom) && !cycledetected)
        {
            /* Is orbital (2) filled? */
            // bool isorbitalfilled = (n_j.binding[2].size() > 0);
            // if (isorbitalfilled && (r_i.type == 1))
            // {
            //     /* is the robot (r_i) there? */
            //     bool amIthere = false;
            //     /* For each robot (y) is the orbital k */
            //     for (int y = 0; y < n_j.binding[2].size(); y++)
            //     {
            //         if (n_j.binding[2][y] == r_i.id)
            //         {
            //             amIthere = true;
            //             break;
            //         }
            //     }
            //     if (amIthere)
            //     {
            //         bound[1].push_back(n_j.id);
            //     }
            //     else
            //     {
            //         bound[2].push_back(n_j.id);
            //     }
            // }
            // else
            // {
            bound[n_j.type].push_back(n_j.id);
            // }
            // bound[n_j.type].push_back(n_j.id);
        }
    }

#ifdef DEBUG_BINDING
    for (int k = 0; k < r_i.binding.size(); k++)
    {
        std::cout << "  -> [" << k << "]: ";
        for (int y = 0; y < bound[k].size(); y++)
        {
            std::cout << bound[k][y] << "  ";
        }
        std::cout << std::endl;
    }
#endif

    //Update binding list
    unsigned int num_binding = 0;
    /* For each orbital (k) in robot (r_i). */
    // r_i.anchors.clear();
    for (int k = (r_i.binding.size() - 1); k >= 0; k--)
    {
        /* Clear history of orbital */
        r_i.binding[k].clear();

        /* Dropping element next orbital */
        // if ((k == 2) && (bound[k].size() > r_i.orbitals[k]) && (bound[k - 1].size() == 0))
        // {
        //     for (int u = r_i.orbitals[k]; u < bound[k].size(); u++)
        //     {
        //         bound[1].push_back( bound[k][u]);
        //         // bound[1].insert(bound[1].begin(), bound[k][u]);
        //     }
        // }

        /* For each robot (k) to be add to robot (r_i) orbital (i). */
        for (int i = 0; i < bound[k].size(); i++)
        {
            if ((i < r_i.orbitals[k]) && (num_binding < r_i.bound))
            {
                // Robot n_i;
                // for (int r = 0; r < states_t.size(); r++)
                // {
                //     if (states_t[r].id == bound[k][i])
                //         n_i = states_t[r];
                // }

                // if ((n_i.type == 2) && (r_i.type == 1))
                // {
                //     r_i.anchors.push_back(bound[k][i]);
                // }

                // if ((n_i.type == 1) && (n_i.anchors.size() > 0)  && (r_i.type == 1)){
                //      for (int a = 0; a < n_i.anchors.size(); a++){
                //          r_i.anchors.push_back(n_i.anchors[a]);
                //      }
                // }

                r_i.binding[k].push_back(bound[k][i]);
                num_binding++;
            }
        }
    }

    // sort( r_i.anchors.begin(), r_i.anchors.end() );
    // r_i.anchors.erase( unique( r_i.anchors.begin(), r_i.anchors.end() ), r_i.anchors.end() );

    if (num_binding == r_i.bound)
    {
        r_i.bounded = true;
    }
    else
    {
        r_i.bounded = false;
    }
}

void Controller::update(long iterations)
{
    std::vector<Robot> states_t;
    states_t = this->states;
    std::vector<std::vector<Robot>> neighbors = this->getAllRobotsNeighborns(this->states);
#ifdef DEBUG_BINDING
    std::cout << "============================= " << std::endl;
#else
#pragma omp parallel for
#endif
    for (int i = 0; i < this->robots; ++i)
    {
#ifdef DEBUG_BINDING
        std::cout << "== Robot # " << states_t[i].id << " | " << states_t[i].type << " ==" << std::endl;
#endif
        this->updateBinding(states_t[i], neighbors[i]);
#ifdef DEBUG_BINDING
        for (int k = 0; k < states_t[i].binding.size(); k++)
        {
            std::cout << "-> [" << k << "]: ";
            for (int y = 0; y < states_t[i].binding[k].size(); y++)
            {
                std::cout << states_t[i].binding[k][y] << "  ";
            }
            std::cout << std::endl;
        }
#endif
    }

    // #pragma omp parallel for ordered schedule(dynamic)

#pragma omp parallel for
    for (int i = 0; i < this->robots; ++i)
    {
        this->metropolisHastings(states_t[i], neighbors[i]);
    }
}

#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 100

void printProgress(double percentage, double timer)
{
    int val = (int)(percentage * 100);
    int lpad = (int)(percentage * PBWIDTH);
    int rpad = PBWIDTH - lpad;
    printf("\33[92m\r%3d%% [%.*s%*s] %f ms\33[0m", val, lpad, PBSTR, rpad, "", timer);
    fflush(stdout);
}

/* Main loop: instantiate gibbs controller */
int main(int argc, char **argv)
{
    srand(time(0));
    // ROS setups:
    ros::init(argc, argv, "grf_controller", ros::init_options::AnonymousName); // node name

    ros::NodeHandle nh("~"); // create a node handle; need to pass this to the class constructor

    ROS_INFO("[Main] Instantiating an object of type Controller");
    Controller control(&nh);

    long iterations = 0;
    bool cvok = true;
    int porcente = 0;
    long max_it = control.max_iteration;
    do
    {
        auto start = std::chrono::high_resolution_clock::now();
        control.update(iterations);
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        if (control.gui)
        {
            cvok = control.draw(iterations);
        }
        if (!(iterations % (int)(max_it * 0.01)))
        {
            printProgress(porcente / 100.0, (double)duration.count() / 1000.0);
            porcente += 1;
        }
        iterations += 1;
#ifdef BABYSTEP
        cvok = (cv::waitKey(0) != 27);
#endif
    } while (ros::ok() && (iterations < max_it) && cvok);
    // if (control.gui)
    // {
    //     cv::waitKey(0);
    // }

    return 0;
}