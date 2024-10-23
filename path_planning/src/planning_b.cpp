#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <octomap_msgs/conversions.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <queue>
#include <vector>
#include <unordered_map>
#include <functional>

// Custom hash function for octomap::point3d
struct Point3dHash
{
    std::size_t operator()(const octomap::point3d &p) const
    {
        std::size_t hx = std::hash<float>()(p.x());
        std::size_t hy = std::hash<float>()(p.y());
        std::size_t hz = std::hash<float>()(p.z());
        return hx ^ (hy << 1) ^ (hz << 2);
    }
};

struct Node
{
    octomap::point3d position;
    float g_cost; // Cost from start to this node
    float h_cost; // Heuristic cost from this node to the goal
    float f_cost; // g_cost + h_cost
    Node *parent;

    Node(octomap::point3d pos, float g, float h, Node *p = nullptr)
        : position(pos), g_cost(g), h_cost(h), f_cost(g + h), parent(p) {}

    bool operator<(const Node &other) const
    {
        return f_cost > other.f_cost; // Lower f_cost has higher priority
    }
};

class PathPlanner
{
public:
    PathPlanner(ros::NodeHandle &nh)
        : nh_(nh), octree_(nullptr)
    {
        octomap_sub_ = nh_.subscribe("/red/octomap_binary", 1, &PathPlanner::octomapCallback, this);
        pose_sub_ = nh_.subscribe("/hummingbird/odometry_sensor1/pose", 1, &PathPlanner::poseCallback, this);
        goal_sub_ = nh_.subscribe("/red/exploration/goal", 1, &PathPlanner::goalCallback, this);
        path_pub_ = nh_.advertise<nav_msgs::Path>("/uav/path", 1);
    }

    void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg)
    {
        octomap::AbstractOcTree *tree = octomap_msgs::binaryMsgToMap(*msg);
        octree_ = dynamic_cast<octomap::OcTree *>(tree);
        // std::cout << "0" << std::endl;
    }

    void poseCallback(const geometry_msgs::Pose::ConstPtr &msg)
    {
        current_pose_ = *msg;
        if (octree_)
        {
            // std::cout << "------" << std::endl;
            // std::cout << goal_.position.x << " " << goal_.position.y << " " << goal_.position.z << std::endl;
            computePath();
        }
        else
        {
            ROS_WARN("Octree is not available.");
        }
        // std::cout << "1" << std::endl;
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        goal_ = msg->pose;

        // std::cout << "2" << std::endl;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber octomap_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher path_pub_;

    octomap::OcTree *octree_;
    geometry_msgs::Pose current_pose_;
    geometry_msgs::Pose goal_;

    std::vector<octomap::point3d> getNeighbors(const octomap::point3d &point)
    {
        std::vector<octomap::point3d> neighbors;
        float step_size = octree_->getResolution();

        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dy = -1; dy <= 1; dy++)
            {
                for (int dz = -1; dz <= 1; dz++)
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;

                    octomap::point3d neighbor(point.x() + dx * step_size,
                                              point.y() + dy * step_size,
                                              point.z() + dz * step_size);

                    if (octree_->search(neighbor) == nullptr || octree_->search(neighbor)->getOccupancy() < 0.5)
                    {
                        neighbors.push_back(neighbor);
                    }
                }
            }
        }
        return neighbors;
    }

    float heuristic(const octomap::point3d &a, const octomap::point3d &b)
    {
        return (a - b).norm(); // Sử dụng khoảng cách Euclidean
    }

    void computePath()
    {
        std::cout << "------" << std::endl;
        std::cout << goal_.position.x << " " << goal_.position.y << " " << goal_.position.z << std::endl;
        octomap::point3d start(int(current_pose_.position.x), int(current_pose_.position.y), int(current_pose_.position.z));
        octomap::point3d goal(int(goal_.position.x), int(goal_.position.y), int(goal_.position.z));
        std::cout << "Current: " << current_pose_.position.x << " " << current_pose_.position.y << " " << current_pose_.position.z
                  << "--" << start.x() << " " << start.y() << " " << start.z() << std::endl;
        std::cout << "Goal: " << goal_.position.x << " " << goal_.position.y << " " << goal_.position.z
                  << "--" << goal.x() << " " << goal.y() << " " << goal.z() << std::endl;
        std::priority_queue<Node> open_set;
        std::unordered_map<octomap::point3d, Node *, Point3dHash> all_nodes;

        Node *start_node = new Node(start, 0, heuristic(start, goal));
        open_set.push(*start_node);
        all_nodes[start] = start_node;

        while (!open_set.empty())
        {
            Node current_node = open_set.top();
            open_set.pop();

            if ((current_node.position - goal).norm() < octree_->getResolution())
            {
                reconstructPath(current_node);
                return;
            }

            for (const auto &neighbor_pos : getNeighbors(current_node.position))
            {
                float tentative_g_cost = current_node.g_cost + (current_node.position - neighbor_pos).norm();

                if (all_nodes.find(neighbor_pos) == all_nodes.end() || tentative_g_cost < all_nodes[neighbor_pos]->g_cost)
                {
                    Node *neighbor_node = new Node(neighbor_pos, tentative_g_cost, heuristic(neighbor_pos, goal), &current_node);
                    open_set.push(*neighbor_node);
                    all_nodes[neighbor_pos] = neighbor_node;
                }
            }
        }

        ROS_WARN("No path found to the goal.");
    }

    void reconstructPath(const Node &goal_node)
    {
        nav_msgs::Path path;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = "world";

        const Node *current_node = &goal_node;
        while (current_node != nullptr)
        {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = current_node->position.x();
            pose.pose.position.y = current_node->position.y();
            pose.pose.position.z = current_node->position.z();
            path.poses.push_back(pose);

            // In ra tọa độ của điểm trên đường đi
            std::cout << "Point: ("
                      << current_node->position.x() << ", "
                      << current_node->position.y() << ", "
                      << current_node->position.z() << ")" << std::endl;

            current_node = current_node->parent;
        }

        std::reverse(path.poses.begin(), path.poses.end());
        path_pub_.publish(path);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_path_planner");
    ros::NodeHandle nh;
    PathPlanner planner(nh);

    ros::spin();
    return 0;
}
