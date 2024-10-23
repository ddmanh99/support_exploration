#include <ros/ros.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

void publishMarkers(ros::Publisher &marker_pub, octomap::ColorOcTree &octree)
{ // Thay đổi ở đây
    visualization_msgs::MarkerArray marker_array;

    for (octomap::ColorOcTree::leaf_iterator it = octree.begin_leafs(), end = octree.end_leafs(); it != end; ++it)
    {
        geometry_msgs::Point p;
        p.x = it.getX();
        p.y = it.getY();
        p.z = it.getZ();

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map"; // Đặt frame_id tương ứng
        marker.header.stamp = ros::Time::now();
        marker.ns = "octomap_markers";
        marker.id = marker_array.markers.size();
        marker.type = visualization_msgs::Marker::CUBE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = p;
        marker.scale.x = marker.scale.y = marker.scale.z = octree.getResolution();

        if (octree.isNodeOccupied(*it))
        {
            marker.color.r = 1.0; // Màu đỏ cho occupied
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        }
        // else
        // {
        //     marker.color.g = 1.0; // Màu xanh cho free
        //     marker.color.r = 0.0;
        //     marker.color.b = 0.0;
        // }

        marker.color.a = 0.5; // Độ trong suốt
        marker_array.markers.push_back(marker);
    }

    marker_pub.publish(marker_array);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "octomap_marker_publisher");
    ros::NodeHandle nh;

    std::string octomap_file = "/home/manh/Documents/DATN/Result/LV2/lv2_raw.binvox.bt"; // Đường dẫn đến file OctoMap
    octomap::ColorOcTree octree(0.5);                                                    // Thay đổi kích thước resolution nếu cần

    if (!octree.readBinary(octomap_file))
    {
        ROS_ERROR("Failed to read OctoMap from file %s", octomap_file.c_str());
        return -1;
    }

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("octomap_markers", 1);

    ros::Rate rate(0.1); // Tần số phát

    while (ros::ok())
    {
        publishMarkers(marker_pub, octree); // Không cần thay đổi ở đây
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
