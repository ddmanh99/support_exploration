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

class UAVPathPlanner
{
public:
    UAVPathPlanner(const std::string &map_file)
    {
        ros::NodeHandle nh;

        // map_sub = nh.subscribe("/red/octomap_binary", 1, &UAVPathPlanner::mapCallback, this);
        pose_sub = nh.subscribe("/hummingbird/odometry_sensor1/pose", 10, &UAVPathPlanner::poseCallback, this);
        goal_sub = nh.subscribe("/red/exploration/goal", 1, &UAVPathPlanner::goalCallback, this);
        goalReached_sub = nh.subscribe("/red/exploration/point_reached", 1, &UAVPathPlanner::goalReachedCB, this);
        path_pub = nh.advertise<nav_msgs::Path>("/planning/uav_path", 1);
        // Đọc OctoMap từ file
        octree_ptr = std::make_shared<octomap::OcTree>(map_file);
        if (octree_ptr)
        {
            size_t num_nodes = octree_ptr->calcNumNodes();
            std::cout << "Số lượng node trong OctoMap: " << num_nodes << std::endl;
            ROS_INFO("Successfully loaded OctoMap from file.");
            // findPath();
        }
        else
        {
            ROS_ERROR("Failed to load OctoMap from file.");
        }
    }

    // void mapCallback(const octomap_msgs::Octomap::ConstPtr &msg)
    // {
    //     octomap::AbstractOcTree *tree = octomap_msgs::binaryMsgToMap(*msg);

    //     octomap::OcTree *octree = dynamic_cast<octomap::OcTree *>(tree);
    //     if (octree)
    //     {
    //         octree_ptr.reset(octree);
    //         findPath();
    //     }
    // }

    void poseCallback(const geometry_msgs::Pose::ConstPtr &msg)
    {
        current_pose = *msg;
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        goal_ = msg->pose;
        findPath();

        // std::cout << "2" << std::endl;
    }

    void goalReachedCB(const std_msgs::Bool::ConstPtr &msg)
    {
        reachedMsg = msg->data;
    }

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

    void findPath()
    {
        std::cout << "-------------------" << std::endl;
        // if (reachedMsg)
        // {
        //     return;
        // }

        octomap::point3d start(round(current_pose.position.x),
                               round(current_pose.position.y),
                               round(current_pose.position.z));
        // if (abs(goal_.position.x - current_pose.position.x) < limit_toGoal)
        // {
        //     return;
        // }
        // octomap::point3d goal(-5, -2, 3); // Vị trí đích
        if (goal_.position.z > 0.0)
        {
            int cout = 0;
            octomap::point3d goal(round(goal_.position.x),
                                  round(goal_.position.y),
                                  round(goal_.position.z));
            std::cout << "Current: " << current_pose.position.x << " " << current_pose.position.y << " " << current_pose.position.z
                      << " -- " << start.x() << " " << start.y() << " " << start.z() << std::endl;
            std::cout << "Goal: " << goal_.position.x << " " << goal_.position.y << " " << goal_.position.z
                      << " -- " << goal.x() << " " << goal.y() << " " << goal.z() << std::endl;

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
            if (distance > 36.0)
            {
                step = 3.0;
                limit_findPath = step;
            }
            else if (distance > 24.0)
            {
                step = 4.0;
                limit_findPath = step;
            }
            else if (distance > 16.0)
            {
                step = 3.0;
                limit_findPath = step;
            }
            else if (distance > 8.0)
            {
                step = 2.0;
                limit_findPath = step;
            }
            std::cout << "Distance Mahattan: " << distance << std::endl;
            std::cout << "Step: " << step << std::endl;

            auto cmp = [](Node *a, Node *b)
            { return a->f_cost() > b->f_cost(); };
            std::priority_queue<Node *, std::vector<Node *>, CompareNode> open_set;
            std::vector<octomap::point3d> directions = {
                octomap::point3d(step, 0, 0), octomap::point3d(-step, 0, 0),
                octomap::point3d(0, step, 0), octomap::point3d(0, -step, 0),
                octomap::point3d(0, 0, step), octomap::point3d(0, 0, -step)};

            open_set.push(new Node(start, 0.0, (goal - start).norm(), nullptr));
            std::map<octomap::point3d, bool, ComparePoint3D> closed_set;

            auto start_time = std::chrono::steady_clock::now();
            while (!open_set.empty())
            {
                Node *current = open_set.top();
                open_set.pop();
                // std::cout << "planning ..." << std::endl;
                // break;
                // std::cout << "Planning distance " << (current->position - goal).norm() << std::endl;
                if ((current->position - goal).norm() < limit_findPath)
                {
                    publishPath(current);
                    return;
                    // if ((current->position - goal).norm() < limit_findPath < limit_toGoal)
                    // {
                    //     return;
                    // }
                }

                // else
                // {
                //     limit_findPath = 1.0;
                // }

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
                auto current_time = std::chrono::steady_clock::now();
                auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

                // if (elapsed_time > 30)
                // {
                //     ROS_WARN("Timeout: Planning took longer than 30 seconds");
                //     limit_findPath += 0.5;
                //     std::cout << "limit_findPath :" << limit_findPath << std::endl;
                // }
            }

            ROS_WARN("No path found!");
        }
    }

    void publishPath(Node *goal_node)
    {
        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "world";

        Node *current = goal_node;
        while (current != nullptr)
        {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = current->position.x();
            pose.pose.position.y = current->position.y();
            pose.pose.position.z = current->position.z();
            path_msg.poses.push_back(pose);

            // In ra tọa độ của điểm trên đường đi
            std::cout
                << "Point: ("
                << current->position.x() << ", "
                << current->position.y() << ", "
                << current->position.z() << ")" << std::endl;

            current = current->parent;
        }
        geometry_msgs::PoseStamped poseG;
        poseG.pose.position.x = goal_.position.x;
        poseG.pose.position.y = goal_.position.y;
        poseG.pose.position.z = goal_.position.z;
        std::reverse(path_msg.poses.begin(), path_msg.poses.end());
        path_msg.poses.push_back(poseG);

        path_pub.publish(path_msg);
    }

private:
    ros::Subscriber map_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber goal_sub;
    ros::Subscriber goalReached_sub;
    ros::Publisher path_pub;
    geometry_msgs::Pose current_pose;
    geometry_msgs::Pose goal_;
    bool reachedMsg;
    std::shared_ptr<octomap::OcTree> octree_ptr;
    double limit_findPath = 1.0;
    double limit_toGoal = 1.5;
    double step = 1.0;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_path_planner");
    std::string map_file = "/home/manh/exploration/src/uav_frontier_exploration_3d/data/mb_lv1.binvox.bt";
    UAVPathPlanner planner(map_file);
    ros::spin();
    return 0;
}
