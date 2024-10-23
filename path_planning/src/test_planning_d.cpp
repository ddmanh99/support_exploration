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

// (The rest of your structures remain unchanged)

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

bool isSafeStep(const octomap::OcTree *octree, const octomap::point3d &start, const octomap::point3d &end, double min_distance)
{
    // Chia nhỏ khoảng cách giữa start và end thành các bước nhỏ và kiểm tra từng điểm
    double resolution = octree->getResolution();
    octomap::point3d direction = end - start;
    double distance = direction.norm();
    direction.normalize();

    // Di chuyển từ start đến end với bước nhỏ bằng resolution
    for (double d = 0; d <= distance; d += resolution)
    {
        octomap::point3d intermediate_point = start + direction * d;
        if (!isSafe(octree, intermediate_point, min_distance))
        {
            return false; // Phát hiện va chạm với vật cản
        }
    }

    // Nếu tất cả các điểm trên đường đi đều an toàn
    return true;
}

double mahattanDistance(const octomap::point3d p1, const octomap::point3d p2)
{
    double distance = std::abs(p1.x() - p2.x()) + std::abs(p1.y() - p2.y()) + std::abs(p1.z() - p2.z());
    return distance;
}

// Tính chi phí di chuyển giữa 2 điểm dựa trên số trục di chuyển
double getMoveCost(const octomap::point3d &current, const octomap::point3d &neighbor)
{
    octomap::point3d diff = neighbor - current;
    int num_axes = (diff.x() != 0) + (diff.y() != 0) + (diff.z() != 0);

    if (num_axes == 1)
        return 1.0; // Di chuyển theo 1 trục
    if (num_axes == 2)
        return sqrt(2.0); // Di chuyển theo 2 trục
    return sqrt(3.0);     // Di chuyển theo 3 trục
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

    if (isSafe(octree_ptr.get(), start, 0.5))
    {
        std::cout << "Start is safe" << std::endl;
    }
    else
    {
        std::vector<octomap::point3d> directions = {
            octomap::point3d(step, 0, 0), octomap::point3d(-step, 0, 0),
            octomap::point3d(0, step, 0), octomap::point3d(0, -step, 0),
            octomap::point3d(0, 0, step), octomap::point3d(0, 0, -step)};
        std::cout << "Start is NOT safe" << std::endl;
        for (const auto &dir : directions)
        {
            octomap::point3d neighbor_pos = start + dir;
            if (isSafe(octree_ptr.get(), neighbor_pos, 0.5))
            {
                start = neighbor_pos;
                std::cout << "Found neighbor start safe is: " << neighbor_pos.x() << " " << neighbor_pos.y() << " " << neighbor_pos.z() << std::endl;
                break;
            }
        }
    }

    double distance = mahattanDistance(start, goal);
    // double distance = (goal - start).norm();

    std::cout << "Distance Mahattan is: " << distance << std::endl;

    std::priority_queue<Node *, std::vector<Node *>, CompareNode> open_set;
    open_set.push(new Node(start, 0.0, mahattanDistance(start, goal), nullptr));
    // open_set.push(new Node(start, 0.0, (goal - start).norm(), nullptr));
    std::map<octomap::point3d, bool, ComparePoint3D> closed_set;

    auto start_time = std::chrono::steady_clock::now();
    auto start_time_noRestart = std::chrono::steady_clock::now();
    bool path_found = false;

    std::cout << "Nhap step: ";
    std::cin >> step;
    limit_findPath = step;

    while (!path_found)
    {
        open_set = std::priority_queue<Node *, std::vector<Node *>, CompareNode>();
        open_set.push(new Node(start, 0.0, mahattanDistance(start, goal), nullptr));
        // open_set.push(new Node(start, 0.0, (goal - start).norm(), nullptr));
        closed_set.clear();
        std::vector<octomap::point3d> directions = {
            octomap::point3d(step, 0, 0), octomap::point3d(-step, 0, 0),
            octomap::point3d(0, step, 0), octomap::point3d(0, -step, 0),
            octomap::point3d(0, 0, step), octomap::point3d(0, 0, -step)};
        // const std::vector<octomap::point3d> directions = {
        //     // 6 hướng cơ bản
        //     {1, 0, 0},
        //     {-1, 0, 0},
        //     {0, 1, 0},
        //     {0, -1, 0},
        //     {0, 0, 1},
        //     {0, 0, -1},

        //     // 12 hướng chéo theo 2 trục
        //     {1, 1, 0},
        //     {1, -1, 0},
        //     {-1, 1, 0},
        //     {-1, -1, 0},
        //     {1, 0, 1},
        //     {1, 0, -1},
        //     {-1, 0, 1},
        //     {-1, 0, -1},
        //     {0, 1, 1},
        //     {0, 1, -1},
        //     {0, -1, 1},
        //     {0, -1, -1},

        //     // 8 hướng chéo theo 3 trục
        //     {1, 1, 1},
        //     {1, 1, -1},
        //     {1, -1, 1},
        //     {1, -1, -1},
        //     {-1, 1, 1},
        //     {-1, 1, -1},
        //     {-1, -1, 1},
        //     {-1, -1, -1}};

        // std::cout << "Step is --- " << step << std::endl;

        while (!open_set.empty())
        {
            Node *current = open_set.top();
            open_set.pop();

            if ((current->position - goal).norm() < limit_findPath)
            {
                publishPath(current);
                path_found = true;
                auto end_time = std::chrono::steady_clock::now();
                std::chrono::duration<double> total_time = end_time - start_time_noRestart;
                std::cout << "Total time is: " << total_time.count() << std::endl;
                std::cout << "Step is: " << step << std::endl;
                break;
            }
            closed_set[current->position] = true;

            for (const auto &dir : directions)
            {
                octomap::point3d neighbor_pos = current->position + dir;
                if (!octree_ptr->search(neighbor_pos))
                    continue; // Not scanned

                if (octree_ptr->isNodeOccupied(octree_ptr->search(neighbor_pos)))
                    continue; // Obstacle

                if (closed_set.find(neighbor_pos) != closed_set.end())
                    continue;

                // if (!isSafe(octree_ptr.get(), neighbor_pos, 0.5))
                //     continue;

                if (!isSafeStep(octree_ptr.get(), current->position, neighbor_pos, 0.5))
                    continue;

                double g_cost = current->g_cost + dir.norm();
                // double h_cost = getMoveCost(current->position, neighbor_pos);
                double h_cost = mahattanDistance(current->position, neighbor_pos);

                open_set.push(new Node(neighbor_pos, g_cost, h_cost, current));
            }

            auto current_time = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed_seconds = current_time - start_time;

            if (elapsed_seconds.count() > 1.0)
            {
                step += 1.0; // Increase step size
                limit_findPath = step;
                std::cout << "Time exceeded 1 seconds. Increasing step to: " << step << std::endl;
                break; // Restart pathfinding with the new step size
            }
        }

        if (!path_found)
        {
            // std::cout << "Restarting pathfinding with step: " << step << std::endl;
            start_time = std::chrono::steady_clock::now(); // Reset timer for next attempt
        }
    }

    if (!path_found)
    {
        std::cout << "No path found!" << std::endl;
    }

    return 0;
}
