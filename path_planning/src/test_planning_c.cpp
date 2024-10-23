#include <ros/ros.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <queue>
#include <vector>
#include <chrono>

struct Node
{
    octomap::point3d position;
    double g_cost, h_cost;
    Node *parent;

    Node(octomap::point3d pos, double g, double h, Node *p)
        : position(pos), g_cost(g), h_cost(h), parent(p) {}

    double f_cost() const { return g_cost + h_cost; }
};

struct CompareNode
{
    bool operator()(const Node *a, const Node *b)
    {
        return a->f_cost() > b->f_cost();
    }
};

struct ComparePoint3D
{
    bool operator()(const octomap::point3d &a, const octomap::point3d &b) const
    {
        if (a.x() != b.x())
            return a.x() < b.x();
        if (a.y() != b.y())
            return a.y() < b.y();
        return a.z() < b.z();
    }
};

bool isSafe(const octomap::OcTree *octree, const octomap::point3d &point, double min_distance)
{
    for (double x = -min_distance; x <= min_distance; x += octree->getResolution())
    {
        for (double y = -min_distance; y <= min_distance; y += octree->getResolution())
        {
            for (double z = -min_distance; z <= min_distance; z += octree->getResolution())
            {
                octomap::point3d neighbor(point.x() + x, point.y() + y, point.z() + z);
                if (octree->search(neighbor) && octree->isNodeOccupied(octree->search(neighbor)))
                {
                    return false; // The point is too close to an obstacle
                }
            }
        }
    }
    return true;
}

double mahattanDistance(const octomap::point3d p1, const octomap::point3d p2)
{
    double distance = std::abs(p1.x() - p2.x()) + std::abs(p1.y() - p2.y()) + std::abs(p1.z() - p2.z());
    return distance;
}

void publishPath(Node *goal_node)
{
    Node *current = goal_node;
    while (current != nullptr)
    {
        std::cout << "Point: ("
                  << current->position.x() << ", "
                  << current->position.y() << ", "
                  << current->position.z() << ")" << std::endl;

        current = current->parent;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_path_planner");
    std::string map_file = "/home/manh/exploration/src/uav_frontier_exploration_3d/data/mb_lv1.binvox.bt";

    std::shared_ptr<octomap::OcTree> octree_ptr;
    octree_ptr = std::make_shared<octomap::OcTree>(map_file);
    if (octree_ptr)
    {
        size_t num_nodes = octree_ptr->calcNumNodes();
        std::cout << "Số lượng node trong OctoMap: " << num_nodes << std::endl;
        ROS_INFO("Successfully loaded OctoMap from file.");
    }
    else
    {
        ROS_ERROR("Failed to load OctoMap from file.");
    }

    double start_x, start_y, start_z;
    double goal_x, goal_y, goal_z;

    double step = 1.0;
    double limit_findPath = step;

    std::cout << "Nhap Start: ";
    std::cin >> start_x >> start_y >> start_z;

    std::cout << "Nhap Goal: ";
    std::cin >> goal_x >> goal_y >> goal_z;
    octomap::point3d start(start_x, start_y, start_z);
    octomap::point3d goal(goal_x, goal_y, goal_z);
    if (isSafe(octree_ptr.get(), goal, 0.5))
    {
        std::cout << "Goal is safe" << std::endl;
    }
    else
    {
        std::vector<octomap::point3d> directions = {
            octomap::point3d(step, 0, 0), octomap::point3d(-step, 0, 0),
            octomap::point3d(0, step, 0), octomap::point3d(0, -step, 0),
            octomap::point3d(0, 0, step), octomap::point3d(0, 0, -step)};
        std::cout << "Goal is NOT safe" << std::endl;
        for (const auto &dir : directions)
        {
            octomap::point3d neighbor_pos = goal + dir;
            if (isSafe(octree_ptr.get(), neighbor_pos, 0.5))
            {
                goal = neighbor_pos;
                std::cout << "Found neighbor goal safe is: " << neighbor_pos.x() << " " << neighbor_pos.y() << " " << neighbor_pos.z() << std::endl;
                break;
            }
        }
    }

    // double distance = (goal - start).norm();
    double distance = mahattanDistance(start, goal);
    std::cout << "Distance Mahattan is: " << distance << std::endl;
    // if (distance > 36.0)
    // {
    //     step = 5.0;
    //     limit_findPath = step;
    // }
    // else if (distance > 24.0)
    // {
    //     step = 4.0;
    //     limit_findPath = step;
    // }
    // else if (distance > 16.0)
    // {
    //     step = 3.0;
    //     limit_findPath = step;
    // }
    // else if (distance > 8.0)
    // {
    //     step = 2.0;
    //     limit_findPath = step;
    // }
    std::cout
        << "Step estimate: " << step << std::endl;
    std::cout << "Nhap lai step: ";
    std::cin >> step;
    limit_findPath = step;
    std::cout << "Step after: " << step << std::endl;

    std::priority_queue<Node *, std::vector<Node *>, CompareNode> open_set;
    std::vector<octomap::point3d> directions = {
        octomap::point3d(step, 0, 0), octomap::point3d(-step, 0, 0),
        octomap::point3d(0, step, 0), octomap::point3d(0, -step, 0),
        octomap::point3d(0, 0, step), octomap::point3d(0, 0, -step)};

    open_set.push(new Node(start, 0.0, (goal - start).norm(), nullptr));
    std::map<octomap::point3d, bool, ComparePoint3D> closed_set;

    while (!open_set.empty())
    {
        Node *current = open_set.top();
        open_set.pop();

        // std::cout << current->position.x() << " "
        //           << current->position.y() << " "
        //           << current->position.z() << std::endl;
        if ((current->position - goal).norm() < limit_findPath)
        {
            publishPath(current);
            limit_findPath = 1.0;
            return 0;
        }
        closed_set[current->position] = true;

        for (const auto &dir : directions)
        {
            octomap::point3d neighbor_pos = current->position + dir;
            if (!octree_ptr->search(neighbor_pos))
                continue; // Chưa được quét

            if (octree_ptr->isNodeOccupied(octree_ptr->search(neighbor_pos)))
                continue; // Vật cản

            if (closed_set.find(neighbor_pos) != closed_set.end())
                continue;

            if (!isSafe(octree_ptr.get(), neighbor_pos, 0.5))
                continue;

            double g_cost = current->g_cost + dir.norm();
            double h_cost = (goal - neighbor_pos).norm();

            open_set.push(new Node(neighbor_pos, g_cost, h_cost, current));
        }
    }
    std::cout << "No path found!" << std::endl;
}
