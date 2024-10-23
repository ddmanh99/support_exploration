#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

class SubmapGenerator
{
public:
    SubmapGenerator()
    {
        // Subscriber to the point cloud topic
        pointcloud_sub_ = nh_.subscribe("/hummingbird/lidar/scan", 10, &SubmapGenerator::pointCloudCallback, this);

        // Publisher for the submap
        submap_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/submap", 10);
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
    {
        // Convert from ROS message to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);

        // Add the new point cloud to the buffer
        pointcloud_buffer_.push_back(cloud);

        // Keep only the last 3-4 point clouds
        if (pointcloud_buffer_.size() > 4)
        {
            pointcloud_buffer_.erase(pointcloud_buffer_.begin());
        }

        // Create a combined submap from the buffered point clouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (auto &pc : pointcloud_buffer_)
        {
            *combined_cloud += *pc;
        }

        // Apply some filtering (e.g., downsampling)
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(combined_cloud);
        voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f);
        voxel_filter.filter(*filtered_cloud);

        // Convert back to ROS message and publish
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*filtered_cloud, output);
        output.header.frame_id = "world";
        submap_pub_.publish(output);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber pointcloud_sub_;
    ros::Publisher submap_pub_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pointcloud_buffer_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "submap_generator");

    SubmapGenerator submap_generator;

    ros::spin();
    return 0;
}
