#include "metrics.h"
#include <unordered_map>

Metric::Metric(double robots_, double groups_, double threshold_) : robots(robots_), groups(groups_), threshold(threshold_) {}

double Metric::norm(Vector2 v)
{
    return sqrt(v.x * v.x + v.y * v.y);
}

double Metric::norm(double x1, double y1, double x2, double y2)
{
    double dx = (x1 - x2);
    double dy = (y1 - y2);
    return sqrt(dx * dx + dy * dy);
}

double Metric::consensus_metric(std::vector<Robot> states)
{
    double consensus_metric = 0.0;

    for (int i = 0; i < states.size(); i++)
    {
        Robot r = states[i];

        // Compute summation of velocities of a group
        double vx_sum = r.velocity.x;
        double vy_sum = r.velocity.y;
        double v_count = 1;

        for (int j = 0; j < r.binding.size(); j++)
        {
            for (int k = 0; k < r.binding[j].size(); k++)
            {
                int index = r.binding[j][k];
                Robot neighbor = states[index];
                vx_sum += neighbor.velocity.x;
                vy_sum += neighbor.velocity.y;
                v_count += 1.0;
            }
        }

        // Compute average velocity of a group
        double vx_mean = vx_sum / v_count;
        double vy_mean = vy_sum / v_count;

        double v_diff = this->norm(r.velocity.x, r.velocity.y, vx_mean, vy_mean);
        v_count = 1;
        for (int j = 0; j < r.binding.size(); j++)
        {
            for (int k = 0; k < r.binding[j].size(); k++)
            {
                int index = r.binding[j][k];
                assert(index == states[index].id);
                Robot neighbor = states[index];
                v_diff += this->norm(neighbor.velocity.x, neighbor.velocity.y, vx_mean, vy_mean);
                v_count += 1.0;
            }
        }

        consensus_metric += v_diff / v_count;
    }
    return consensus_metric / (double)states.size();
}

int Metric::miss_bounding_metric(std::vector<Robot> states)
{
    int number_missed_bounding = 0;
    for (int i = 0; i < states.size(); i++)
    {
        Robot r = states[i];

        int number_of_bounds = 0;

        for (int j = 0; j < r.binding.size(); j++)
        {
            for (int k = 0; k < r.binding[j].size(); k++)
            {
                number_of_bounds++;
            }
        }
        number_missed_bounding += (r.bound - number_of_bounds);
    }

    return number_missed_bounding;
}

int Metric::number_of_molecules_metric(std::vector<Robot> states)
{
    std::vector<std::vector<Robot>> candidate_molecules;
    std::unordered_map<int, bool> visited;

    while (visited.size() < states.size())
    {
        std::vector<Robot> candidate_molecule;
        for (int i = 0; i < states.size(); i++)
        {
            if (visited.find(states[i].id) != visited.end())
            {
                continue;
            }
            candidate_molecule.push_back(states[i]);
            break;
        }

        bool founded = true;
        while (founded)
        {
            founded = false;

            for (int i = 0; i < candidate_molecule.size(); i++)
            {
                if (visited.find(candidate_molecule[i].id) != visited.end())
                {
                    continue;
                }
                visited[candidate_molecule[i].id] = true;
                for (int j = 0; j < candidate_molecule[i].binding.size(); j++)
                {
                    for (int k = 0; k < candidate_molecule[i].binding[j].size(); k++)
                    {
                        int id = candidate_molecule[i].binding[j][k];
                        if (visited.find(id) != visited.end())
                        {
                            continue;
                        }
                        candidate_molecule.push_back(states[id]);
                        founded = true;
                    }
                }
            }
        }
        candidate_molecules.push_back(candidate_molecule);
    }

    int number_of_molecules = 0;

    for (int i = 0; i < candidate_molecules.size(); i++)
    {
        bool is_stable = true;
        for (int j = 0; j < candidate_molecules[i].size(); j++)
        {
            if (!candidate_molecules[i][j].bounded)
            {
                is_stable = false;
                break;
            }
        }
        if (is_stable)
            number_of_molecules++;
    }

    return number_of_molecules;
}

int Metric::cluster_metric(std::vector<Robot> states)
{
    std::vector<std::vector<Robot>> clusters;

    while (states.size())
    {
        std::vector<Robot> cluster;
        cluster.push_back(states[0]);
        states.erase(states.begin());
        bool founded = true;
        while (founded)
        {
            founded = false;
            for (int i = 0; i < cluster.size(); i++)
            {
                for (int j = 0; j < states.size(); j++)
                {
                    if (cluster[i].type != states[j].type)
                    {
                        continue;
                    }
                    double dx = cluster[i].position.x - states[j].position.x;
                    double dy = cluster[i].position.y - states[j].position.y;
                    double dist = sqrt(dx * dx + dy * dy);
                    if (dist > this->threshold)
                    {
                        continue;
                    }
                    cluster.push_back(states[j]);
                    states.erase(states.begin() + j);
                    founded = true;
                    break;
                }
            }
        }
        clusters.push_back(cluster);
    }

    return (int)clusters.size();
}