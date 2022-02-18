#include "grf_ros_gazebo.h"

/*********************************/
/*                CONSTRUCTOR                 */
/*********************************/
Controller::Controller(ros::NodeHandle *nodehandle) : nh_(*nodehandle)
{ // constructor
    this->sensing = 0.50;
    ros::param::get("/sensing", this->sensing);
    this->safezone = 0.15;
    ros::param::get("/safezone", this->safezone);
    this->mass = 5.0;
    ros::param::get("/mass", this->mass);
    this->vmax = 0.30;
    ros::param::get("/vmax", this->vmax);
    this->vmax = 0.12;
    this->dt = 0.01;
    ros::param::get("/dt", this->dt);
    this->worldsize = 3.90;
    ros::param::get("/worldsize", this->worldsize);

    this->robots = 20;
    ros::param::get("/robots", this->robots);
    this->groups = 2;
    ros::param::get("/groups", this->groups);

#ifdef SHOW_OBSTACLES_RVIZ
    this->show_obstacles_rviz = nh_.advertise<visualization_msgs::Marker>("/show_obstacles_rviz", 1);
#endif
#ifdef SHOW_NEIGHBORNS_RVIZ
    this->show_neighborns_rviz = nh_.advertise<visualization_msgs::MarkerArray>("/show_neighborns_rviz", 1);
#endif
#ifdef SHOW_TARGET_VEL_RVIZ
    this->show_target_vel_rviz = nh_.advertise<visualization_msgs::MarkerArray>("/show_target_vel_rviz", 1);
#endif

    for (int i = 0; i < this->robots; i++)
    {
        /* Topics name */
        std::string robot_name = "/hero_" + boost::lexical_cast<std::string>(i);
        ROS_INFO("Starting robot: %s", robot_name.c_str());
        std::string cmd_topic = robot_name + "/cmd_vel";
        // std::string cmd_topic = robot_name + "/goal";
        std::string pose_topic = robot_name + "/odom";
        std::string color_topic = robot_name + "/hat_color";
        /* Topics */
        this->r_cmdvel_.push_back(nh_.advertise<geometry_msgs::Twist>(cmd_topic.c_str(), 1));
        // this->r_cmdvel_.push_back(nh_.advertise<geometry_msgs::PoseStamped>(cmd_topic.c_str(), 1));
        this->r_pose_.push_back(nh_.subscribe<nav_msgs::Odometry>(pose_topic.c_str(), 1, boost::bind(&Controller::r_pose_cb, this, _1, pose_topic, i)));
        this->r_cmdcolor_.push_back(nh_.advertise<std_msgs::ColorRGBA>(color_topic.c_str(), 1));

        /* Get robot type */
        // int type_ = i % 5 > 3;
        int type_ = i < 7;
        ros::param::get(robot_name + "/type", type_);
        /* Instatiate robot global states */
        geometry_msgs::Pose2D p;
        geometry_msgs::Twist t;
        this->global_velocities.push_back(t);
        this->global_poses.push_back(p);

        /* Get initial state of the robots */
        Robot r;
        /* Get initial velocities */
        r.velocity.x = this->vmax;
        r.velocity.y = this->vmax;
        r.type = type_;
        r.id = (double)i;
        r.anchor = false;
        r.bounded = false;

        switch ((int)r.type)
        {
        case 0: /* O */
            r.bound = 2;
            r.orbitals.push_back(2); // Number of O
            r.orbitals.push_back(1); // Number of C
            r.orbitals.push_back(0); // Number of C
            // r.orbitals.push_back(1); // Number of X
            r.mass = 1.4 * 16; // 0.3;
            r.charge = 6;
            break;

        case 1:
            r.bound = 2;             /* O */
            r.orbitals.push_back(2); // Number of O
            r.orbitals.push_back(0); // Number of C
            r.orbitals.push_back(1); // Number of C
            // r.orbitals.push_back(1); // Number of X
            r.mass = 1.4 * 12; // 0.4;
            r.charge = 8;
            break;

            // case 2:
            //     r.bound = 1;             /* C */
            //     r.orbitals.push_back(0); // Number of H
            //     r.orbitals.push_back(1); // Number of N
            //     r.orbitals.push_back(0); // Number of C
            //     r.mass = 0.5;            //0.4;
            //     // r.position.x = (6.0 * count / (NUM - 1) - 6.0 / 2.0);
            //     // r.position.y = fabs(14.0 * count / (NUM - 1) - 14.0 / 2.0) - 3;
            //     r.anchor = true;
            //     r.velocity.x = 0;
            //     r.velocity.y = 0;
            //     // count++;
            //     break;

            // default:
            //     break;
        }

        if (i < 2)
        { // this robots are anchors
            r.type = 2;
            r.mass = 200;
            r.charge = 40;
            r.anchor = true;
            r.bound = 1;
            r.orbitals.clear();
            r.orbitals.push_back(1); // Number of O
            r.orbitals.push_back(1); // Number of C
            r.orbitals.push_back(0); // Number of C
            r.velocity.x = 0;
            r.velocity.y = 0;
        }

        for (int k = 0; k < (int)r.orbitals.size(); k++)
        {
            std::vector<unsigned int> t_;
            r.binding.push_back(t_);
        }
        ROS_INFO("Type: %f", r.type);
        this->states.push_back(r);
        /* Set robot color by type */
        this->setRobotColor(r, (int)r.type);
    }

    // Obstacle body local state
    Body obstacle_;
    obstacle_.cm_position = Vector2(0.0, 0.0);
    obstacle_.is_obstacle = true;
    obstacle_.name = "arena";
    obstacle_.local_corners.push_back(Vector2(-this->worldsize * 0.5, -this->worldsize * 0.5));
    obstacle_.local_corners.push_back(Vector2(this->worldsize * 0.5, -this->worldsize * 0.5));
    obstacle_.local_corners.push_back(Vector2(this->worldsize * 0.5, this->worldsize * 0.5));
    obstacle_.local_corners.push_back(Vector2(-this->worldsize * 0.5, this->worldsize * 0.5));
    obstacle_.global_corners = obstacle_.local_corners;
    this->bodies_state.push_back(obstacle_);
}
/*********************************/
/*        END OF CONSTRUCTOR         */
/*********************************/

/*********************************/
/*        CALLBACK FUNCTIONS           */
/*********************************/
void Controller::r_pose_cb(const nav_msgs::OdometryConstPtr &msg, const std::string &topic, const int &id)
{
    // ROS_INFO("Robot %d getting poses callback", id);
    this->global_poses[id].x = msg->pose.pose.position.x;
    this->global_poses[id].y = msg->pose.pose.position.y;
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    this->global_poses[id].theta = yaw;
    this->global_velocities[id].linear.x = msg->twist.twist.linear.x;
    this->global_velocities[id].linear.y = msg->twist.twist.linear.y;
}
/*********************************/
/* END OF CALLBACK FUNCTIONS */
/*********************************/

/*********************************/
/*          GENERAL FUNCTION             */
/*********************************/
void Controller::setRobotColor(Robot robot, int colorId)
{
    std_msgs::ColorRGBA color = this->getColorByType(colorId);
    std_msgs::ColorRGBA color_;
    color_.a = color.a / 255.0;
    color_.r = color.r / 255.0;
    color_.g = color.g / 255.0;
    color_.b = color.b / 255.0;
    this->r_cmdcolor_[(int)robot.id].publish(color_);
}

std_msgs::ColorRGBA Controller::getColorByType(uint8_t type)
{
    std_msgs::ColorRGBA color;
    color.a = 255.0;
    switch (type)
    { // BGR
    case 0:
        color.r = 0;
        color.g = 0;
        color.b = 128;
        break; // maroon
    case 1:
        color.r = 128;
        color.g = 0;
        color.b = 0;
        break; // dark slate gray
    case 2:
        color.r = 50;
        color.g = 50;
        color.b = 50;
        break; // blue violet
    case 3:
        color.r = 199;
        color.g = 21;
        color.b = 133;
        break; // medium violet red
    case 4:
        // color.r = 144;
        // color.g = 238;
        // color.b = 144;
        color.r = 180;
        color.g = 180;
        color.b = 180;
        break; // light green
    case 5:
        color.r = 255;
        color.g = 215;
        color.b = 0;
        break; // gold
    case 6:
        color.r = 218;
        color.g = 165;
        color.b = 32;
        break; // golden rod
    case 7:
        color.r = 189;
        color.g = 183;
        color.b = 107;
        break; // dark khaki
    case 8:
        color.r = 128;
        color.g = 128;
        color.b = 0;
        break; // olive
    case 9:
        color.r = 154;
        color.g = 205;
        color.b = 50;
        break; // yellow green
    case 10:
        color.r = 107;
        color.g = 142;
        color.b = 35;
        break; // olive drab
    case 11:
        color.r = 127;
        color.g = 255;
        color.b = 0;
        break; // chart reuse
    case 12:
        color.r = 0;
        color.g = 100;
        color.b = 0;
        break; // dark green
    case 13:
        color.r = 255;
        color.g = 140;
        color.b = 0;
        break; // dark orange
    case 14:
        color.r = 46;
        color.g = 139;
        color.b = 87;
        break; // sea green
    case 15:
        color.r = 102;
        color.g = 205;
        color.b = 170;
        break; // medium aqua marine
    case 16:
        color.r = 220;
        color.g = 20;
        color.b = 60;
        break; // crimson
    case 17:
        color.r = 0;
        color.g = 139;
        color.b = 139;
        break; // dark cyan
    case 18:
        color.r = 0;
        color.g = 255;
        color.b = 255;
        break; // cyan
    case 19:
        color.r = 70;
        color.g = 130;
        color.b = 180;
        break; // steel blue
    case 20:
        color.r = 100;
        color.g = 149;
        color.b = 237;
        break; // corn flower blue
    case 21:
        color.r = 30;
        color.g = 144;
        color.b = 255;
        break; // dodger blue
    case 22:
        color.r = 0;
        color.g = 0;
        color.b = 128;
        break; // navy
    case 23:
        color.r = 240;
        color.g = 128;
        color.b = 128;
        break; // light coral
    case 24:
        color.r = 75;
        color.g = 0;
        color.b = 130;
        break; // indigo
    case 25:
        color.r = 139;
        color.g = 0;
        color.b = 139;
        break; // dark magenta
    case 26:
        color.r = 238;
        color.g = 130;
        color.b = 238;
        break; // violet
    case 27:
        color.r = 255;
        color.g = 160;
        color.b = 122;
        break; // light salmon
    case 28:
        color.r = 255;
        color.g = 105;
        color.b = 180;
        break; // hot pink
    case 29:
        color.r = 112;
        color.g = 128;
        color.b = 144;
        break; // slate gray
    default:
        color.r = 0;
        color.g = 128;
        color.b = 240;
        break; // black
    }
    return color;
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
    std::vector<Vector2> obstacles = this->getObstaclesPoints(this->sensing, r_i);
    // ROS_INFO("Number of obstacles: %d", obstacles.size());
    for (int i = 0; i < (int)obstacles.size(); ++i)
    {
        // lets compute the distance to the obstacle (we only use the distance)
        double dist = this->euclidean(r_i.position, obstacles[i]);
        if (dist <= 2.0 * this->safezone)
        {
            Us += this->coulombBuckinghamPotential(dist * 0.5, 0.04, 0.04, 0.8, 1.0, 16.0, 1.0);
        }
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
    double group_mass = r_i.mass; 
    // Get the pairwise potential for the sampled velocity
    double Ust = 0.0f;

    // #ifdef _OPENMP
    // #pragma omp parallel for
    // #endif

    // for each neighborn in current state
    // Neighborns should be already sorted by distance
    for (int i = 0; i < (int)states_t.size(); ++i)
    {
        Vector2 n_p;
        n_p.x = states_t[i].position.x + states_t[i].velocity.x * this->dt;
        n_p.y = states_t[i].position.y + states_t[i].velocity.y * this->dt;
        double dist = this->euclidean(r_i.position, n_p);
        // double dist = this->euclidean(r_i.position, states_t[i].position);
        // Indicator function f: 1 -> same type, f: -1 -> otherwise
        double I = 2.0 * (int)(r_i.type == states_t[i].type) - 1.0;

        I = -0.010;
        // I = 0.2;
        dist = dist * 1.44;

        // if (states_t[i].type == r_i.type)
        // {
        //     if (r_i.binding[1].size() > 0 && states_t[i].binding[1].size() > 0)
        //         if (r_i.binding[1][0] == states_t[i].binding[1][0])
        //         {
        //             I = 0.8;
        //         }
        // }

        /* For each orbit (k) in the robot (r_i). */
        for (int k = 0; k < (int)r_i.binding.size(); k++)
        {
            /* For each robot (y) in the orbit (k). */
            for (int y = 0; y < (int)r_i.binding[k].size(); y++)
            {
                if ((int)states_t[i].id == (int)r_i.binding[k][y])
                {
                    if (states_t[i].anchor)
                    {
                        I = r_i.charge * states_t[i].charge; // 8
                        dist = dist * 0.6;
                    }
                    else if (states_t[i].bounded == r_i.bounded)
                    {
                        I = r_i.charge * states_t[i].charge * 5.0/3.0; // 4
                         dist = dist * 0.70;
                    }
                    else
                    {
                        I = r_i.charge * states_t[i].charge; // 3
                         dist = dist * 0.70;
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
                return 1e8;
            }
            else
            {
                return 1e4;
            }
        }

        Ust += this->coulombBuckinghamPotential(dist * 0.8, 0.04, 0.04, 0.8, 1.0, I, -1.0);

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

Vector2 Controller::checkSegment(Vector2 v, Vector2 v0, Vector2 v1)
{
    double dv0_v1 = this->euclidean(v0, v1);
    double dv_v0 = this->euclidean(v, v0);
    double dv_v1 = this->euclidean(v, v1);

    if (abs(dv0_v1 - (dv_v0 + dv_v1)) <= 0.0001)
    {
        return v;
    }

    if (dv_v0 > dv_v1)
    {
        return v1;
    }
    else
    {
        return v0;
    }
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

        p1.x += circle.x;
        p1.y += circle.y;

        p2.x += circle.x;
        p2.y += circle.y;

        o1 = this->checkSegment(o1, p1, p2);
        o2 = this->checkSegment(o2, p1, p2);

        double dist_o1_p1 = this->euclidean(o1, p1);
        double dist_o1_p2 = this->euclidean(o1, p2);
        if (dist_o1_p2 < dist_o1_p1)
        {
            Vector2 aux = o1;
            o1 = o2;
            o2 = aux;
        }
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
                // check occlusions
                bool intersect = false;
                for (int k = 0; k < (int)this->bodies_state.size(); k++)
                {
                    if (this->doIntersectWithObstacle(agents[i].position, agents[j].position, this->bodies_state[k].global_corners))
                    {
                        intersect = true;
                        break;
                    }
                }
                if (intersect)
                {
                    continue;
                }
                neighbors[j].push_back(agents[i]);
                neighbors[i].push_back(agents[j]);
            }
        }
    }
    for (int i = 0; i < this->robots; ++i)
    {
        // Sort using comparator function
        std::sort(neighbors[i].begin(), neighbors[i].end(), cmp(agents[i]));
    }
    return neighbors;
}

bool Controller::onSegment(Vector2 p, Vector2 q, Vector2 r)
{
    if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
        q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
        return true;

    return false;
}

int Controller::orientation(Vector2 p, Vector2 q, Vector2 r)
{
    double val = (q.y - p.y) * (r.x - q.x) -
                 (q.x - p.x) * (r.y - q.y);
    if (abs(val) <= 0.0000001)
        return 0; // colinear

    return (val > 0) ? 1 : 2; // clock or counterclock wise
}

int Controller::doIntersectWithObstacle(Vector2 p1, Vector2 q1, std::vector<Vector2> obstacle)
{
    int size_ = (int)obstacle.size();
    int intersections = 0;
    for (uint8_t i = 0; i < size_; i++)
    {
        if (this->doIntersect(p1, q1, obstacle[i], obstacle[(i + 1) % size_]))
        {
            intersections++;
        }
    }
    return intersections;
}

bool Controller::getSegmentIntersection(Vector2 p1, Vector2 q1, Vector2 p2, Vector2 q2, Vector2 &out)
{
    /* Does the segments intersects ? */
    if (this->doIntersect(p1, q1, p2, q2))
    {
        // Line AB represented as a1x + b1y = c1
        double a1 = q1.y - p1.y;
        double b1 = p1.x - q1.x;
        double c1 = a1 * (p1.x) + b1 * (p1.y);

        // Line CD represented as a2x + b2y = c2
        double a2 = q2.y - p2.y;
        double b2 = p2.x - q2.x;
        double c2 = a2 * (p2.x) + b2 * (p2.y);

        double determinant = a1 * b2 - a2 * b1;

        if (abs(determinant) < DBL_EPSILON)
        {
            // The lines are parallel. This is simplified
            // by returning a pair of FLT_MAX
            return false;
        }
        else
        {
            out.x = (b2 * c1 - b1 * c2) / determinant;
            out.y = (a1 * c2 - a2 * c1) / determinant;
            return true;
        }
    }
    return false;
}

bool Controller::doIntersect(Vector2 p1, Vector2 q1, Vector2 p2, Vector2 q2)
{
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);
    // ROS_INFO("ori (%d, %d, %d, %d)", o1, o2, o3, o4);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1))
        return true;

    // p1, q1 and q2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1))
        return true;

    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2))
        return true;

    // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2))
        return true;

    return false; // Doesn't fall in any of the above cases
}

std::vector<Vector2> Controller::getObstaclesPoints(double sensing, Robot r)
{
    std::vector<Vector2> obstacles;

    // Get object points
    double laser_res = 0.2;
    double laser_range = sensing;
    for (double step = 0; step < (M_PI * 2.0); step += laser_res)
    {
        Vector2 point;
        point.x = r.position.x + laser_range * cos(step + r.theta);
        point.y = r.position.y + laser_range * sin(step + r.theta);

        Vector2 min_point(point.x, point.y);
        double min_dist = laser_range;
        for (uint8_t k = 0; k < (int)this->bodies_state.size(); k++)
        {
            /* Detect only obstacles */
            if (this->bodies_state[k].is_obstacle)
            {
                // ROS_INFO("Founded an object %s", this->bodies_state[k].name.c_str());
                /* For each side of polygon */
                for (uint8_t i = 0; i < (int)this->bodies_state[k].global_corners.size(); i++)
                {
                    Vector2 out;
                    if (this->getSegmentIntersection(r.position, point, this->bodies_state[k].global_corners[i], this->bodies_state[k].global_corners[(i + 1) % (int)this->bodies_state[k].global_corners.size()], out))
                    {
                        double dist = this->euclidean(r.position, out);
                        // ROS_INFO("Collision at %f with dist %f", step, dist);
                        if (dist < min_dist)
                        {
                            min_dist = dist;
                            min_point = out;
                        }
                    }
                }
            }
        }
        obstacles.push_back(min_point);
    }
#ifdef SHOW_OBSTACLES_RVIZ
    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::SPHERE_LIST;
    m.header.stamp = ros::Time::now();
    m.action = visualization_msgs::Marker::DELETEALL;
    this->show_obstacles_rviz.publish(m);
    m.points.clear();
    m.action = visualization_msgs::Marker::ADD;
    m.header.frame_id = "/hero_" + boost::lexical_cast<std::string>((int)r.id) + "/odom";
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 1.0;
    m.color.a = 1.0;
    m.scale.x = 0.02;
    m.scale.y = 0.02;
    m.scale.z = 0.02;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    for (uint8_t i = 0; i < (int)obstacles.size(); i++)
    {
        geometry_msgs::Point point;
        point.x = obstacles[i].x;
        point.y = obstacles[i].y;
        if (this->euclidean(r.position, obstacles[i]) < this->sensing)
        {
            m.points.push_back(point);
        }
    }
    if ((int)obstacles.size())
    {
        this->show_obstacles_rviz.publish(m);
    }
#endif

    return obstacles;
}

double Controller::euclidean(Vector2 a, Vector2 b)
{
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    return sqrt(dx * dx + dy * dy) + 1.0e-9;
}

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
    chain_potential.push_back(
        this->fof_Us(r_i, r_i.velocity) +
        this->fof_Ust(r_i, r_i.velocity, states_t));

    // Random Seed for normal distribution
    unsigned seed_x = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator_x(seed_x);
    unsigned seed_y = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator_y(seed_y);

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
        std::normal_distribution<double> norm_vx(r_i.velocity.x, 0.03);
        std::normal_distribution<double> norm_vy(r_i.velocity.y, 0.03);
        Vector2 sampled_vel;
        sampled_vel.x = norm_vx(generator_x);
        sampled_vel.y = norm_vy(generator_y);
        sampled_vel = this->saturation(sampled_vel, this->vmax);
        // Compute the potential for the sampled velocity
        double U = this->fof_Us(r_i, sampled_vel) +
                   this->fof_Ust(r_i, sampled_vel, states_t);
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
    int burnin = (int)(0.6 * (int)mcmc_chain.size());
    for (int i = burnin; i < (int)mcmc_chain.size(); i++)
    {
        new_velocity.x += mcmc_chain[i].x;
        new_velocity.y += mcmc_chain[i].y;
        n += 1.0;
    }
    new_velocity.x = new_velocity.x / n;
    new_velocity.y = new_velocity.y / n;
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

void Controller::updateBinding(Robot &r_i, std::vector<Robot> states_t)
{
    /* Create a empty orbital structure - It will be the new r_i orbital structure. */
    std::vector<std::vector<int>> bound;
    for (int orb = 0; orb < (int)r_i.orbitals.size(); orb++)
    {
        std::vector<int> t_;
        bound.push_back(t_);
    }

    double orbitalDist = this->sensing;
    /* For each neighborn (n_j) of the robot (r_i): */
    /* Neighborns list (states_t) are already sorted by distance. */
    for (int i = 0; i < (int)states_t.size(); i++)
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
        for (int k = ((int)n_j.binding.size() - 1); k >= 0; k--)
        {
            /* Is k the r_i robot orbital? */
            if (k == r_i.type)
            {
                // is there any room for me?
                isthereanyroom = ( (unsigned int)n_j.binding[k].size()  < n_j.orbitals[k]);
                isthereanyroom = isthereanyroom && ( ((unsigned int) n_j.binding[k].size() + nbindingsofar) < n_j.bound);
                // isthereanyroom = ( ((unsigned int) n_j.binding[k].size() + nbindingsofar) < n_j.bound);
                // if (r_i.type == 1 && !isthereanyroom)
                // {
                //     isthereanyroom = ((n_j.binding[k + 1].size() + nbindingsofar) < n_j.orbitals[k + 1]);
                // }
                break;
            }
            nbindingsofar += (unsigned int) n_j.binding[k].size();
        }
#ifdef DEBUG_BINDING
        std::cout << " |  " << (int)isthereanyroom;
#endif

        /* Check if (n_j) has r_i in the binding list. There are already connected? */
        bool areweconnected = false;
        /* For each orbital in the of the neighborn (n_j) */
        for (int k = ((int)n_j.binding.size() - 1); k >= 0; k--)
        {
            if (areweconnected)
                break;
            /* For each robot (y) is the orbital k */
            for (int y = 0; y < (int)n_j.binding[k].size(); y++)
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
        for (int k = ((int)n_j.binding.size() - 1); k >= 0; k--)
        {
            if (cycledetected)
                break;
            /* For each robot (y) is the orbital (k) */
            for (int y = 0; y < (int)n_j.binding[k].size(); y++)
            {
                /* For each orbital (m) in the robot (r_i) */
                for (int m = ((int)bound.size() - 1); m >= 0; m--)
                {
                    /* For each robot (x) in the orbital (m) */
                    for (int w = 0; w < (int)bound[m].size(); w++)
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

        if (((areweconnected || isthereanyroom) && !cycledetected))
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

    // Update binding list
    unsigned int num_binding = 0;
    /* For each orbital (k) in robot (r_i). */
    // r_i.anchors.clear();
    for (int k = ((int)r_i.binding.size() - 1); k >= 0; k--)
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
        for (int i = 0; i < (int)bound[k].size(); i++)
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
    r_i.bounded = (num_binding == r_i.bound);
}

void Controller::update(long iterations)
{
    for (int i = 0; i < this->robots; i++)
    {
        this->states[i].position.x = this->global_poses[i].x;
        this->states[i].position.y = this->global_poses[i].y;
        this->states[i].theta = this->global_poses[i].theta;
        if (this->states[i].anchor){
            this->states[i].velocity.x = 0;
            this->states[i].velocity.y = 0;
        }
    }
    std::vector<Robot> states_t;
    states_t = this->states;
    std::vector<std::vector<Robot>> neighbors = this->getAllRobotsNeighborns(this->states);

#ifdef USE_OPENMP_
#pragma omp parallel for
    for (int i = 0; i < this->robots; ++i)
    {
        this->updateBinding(states_t[i], neighbors[i]);
    }
#endif

#ifdef SHOW_NEIGHBORNS_RVIZ
    visualization_msgs::MarkerArray m_array;
    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::ARROW;
    m.header.stamp = ros::Time::now();
    m.action = visualization_msgs::Marker::DELETEALL;
    m_array.markers.push_back(m);
    this->show_neighborns_rviz.publish(m_array);
    m_array.markers.clear();
    m.action = visualization_msgs::Marker::ADD;
    m.color.r = 0.0;
    m.color.g = 0.5;
    m.color.b = 0.5;
    m.color.a = 0.8;
    m.scale.y = 0.01;
    m.scale.z = 0.01;

    for (int i = 0; i < this->robots; ++i)
    {
        /* Insert robot i */
        Robot r_i = states_t[i];
        m.header.frame_id = "/hero_" + boost::lexical_cast<std::string>(i) + "/odom";
        for (int k = 0; k < (int)r_i.binding.size(); k++)
        {
            /* For each robot (y) in the orbit (k). */
            for (int y = 0; y < (int)r_i.binding[k].size(); y++)
            {
                unsigned int id_bind = r_i.binding[k][y];
                double dist = this->euclidean(r_i.position, states_t[id_bind].position);

                m.id = i * 1000 + id_bind;
                m.pose.position.x = r_i.position.x;
                m.pose.position.y = r_i.position.y;
                tf::Quaternion q;
                q.setEuler(0, 0, atan2(states_t[id_bind].position.y - r_i.position.y, states_t[id_bind].position.x - r_i.position.x));
                m.pose.orientation.x = q.getX();
                m.pose.orientation.y = q.getY();
                m.pose.orientation.z = q.getZ();
                m.pose.orientation.w = q.getW();
                m.scale.x = dist;
                m_array.markers.push_back(m);
            }
        }
    }

    if ((int)m_array.markers.size() > 0)
    {
        this->show_neighborns_rviz.publish(m_array);
    }

#endif

// #pragma omp parallel for ordered schedule(dynamic)
#ifdef USE_OPENMP_
#pragma omp parallel for
    for (int i = 0; i < this->robots; ++i)
    {
        Vector2 new_velocity = this->metropolisHastings(states_t[i], neighbors[i]);
    }
#endif

#ifdef SHOW_TARGET_VEL_RVIZ
    // visualization_msgs::MarkerArray m_array;
    m_array.markers.clear();
    // visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::ARROW;
    m.header.stamp = ros::Time::now();
    m.action = visualization_msgs::Marker::DELETEALL;
    m_array.markers.push_back(m);
    this->show_target_vel_rviz.publish(m_array);
    m_array.markers.clear();
    m.action = visualization_msgs::Marker::ADD;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.color.a = 1.0;
    m.scale.y = 0.02;
    m.scale.z = 0.02;
#endif
    for (int i = 0; i < this->robots; ++i)
    {
        this->setRobotColor(this->states[i], (int)this->states[i].type);
        if (this->states[i].anchor)
        {
            continue;
        }
#ifdef SHOW_TARGET_VEL_RVIZ
        m.header.frame_id = "/hero_" + boost::lexical_cast<std::string>(i) + "/odom";
        m.id = i;
        m.pose.position.x = this->global_poses[i].x;
        m.pose.position.y = this->global_poses[i].y;
        tf::Quaternion q;
        q.setEuler(0, 0, atan2(this->states[i].velocity.y, this->states[i].velocity.x));
        m.pose.orientation.x = q.getX();
        m.pose.orientation.y = q.getY();
        m.pose.orientation.z = q.getZ();
        m.pose.orientation.w = q.getW();
        m.scale.x = sqrt(this->states[i].velocity.y * this->states[i].velocity.y + this->states[i].velocity.x * this->states[i].velocity.x);
        m_array.markers.push_back(m);
#endif

        // Holonomic to differential driver controller
        geometry_msgs::Twist v;
        v.linear.x = std::min(sqrt(this->states[i].velocity.y * this->states[i].velocity.y + this->states[i].velocity.x * this->states[i].velocity.x), this->vmax);
        double theta_ = atan2(this->states[i].velocity.y, this->states[i].velocity.x);
        double theta_diff = (theta_ - this->global_poses[i].theta);

        if (theta_diff > M_PI)
            theta_diff = -2.0 * M_PI + theta_diff;
        else if (theta_diff < -M_PI)
            theta_diff = 2.0 * M_PI + theta_diff;

        v.angular.z = theta_diff * 4.5;
        // if (fabs(theta_diff) > M_PI_2)
        // {
        //     if (theta_diff > 0)
        //         theta_diff = theta_diff - M_PI;
        //     else if (theta_diff < 0)
        //         theta_diff = theta_diff + M_PI;
        //     v.linear.x = -v.linear.x;
        //     v.angular.z = theta_diff;
        // }
        this->r_cmdvel_[i].publish(v);
    }
#ifdef SHOW_TARGET_VEL_RVIZ
    this->show_target_vel_rviz.publish(m_array);
#endif
}

/********************************/
/*               MAIN FUNCTION              */
/********************************/
int main(int argc, char **argv)
{
    srand(time(0));
    /* ROS setups */
    ros::init(argc, argv, "grf_pattern_controller", ros::init_options::AnonymousName); // node name
    ros::NodeHandle nh("~");                                                           // create a node handle; need to pass this to the class constructor

    ROS_INFO("[Main] Instantiating an object of type Controller");
    Controller control(&nh);

    ros::Rate rate(30);
    uint64_t iterations = 0;
    while (ros::ok())
    {
        auto t1 = std::chrono::high_resolution_clock::now();
        control.update(0);
        auto t2 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
        ROS_INFO_THROTTLE(1, "Loop-time at %f ms", (float)duration * 0.001);
        iterations++;
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}