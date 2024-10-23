#include <octomap/octomap.h>
#include <iostream>

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " test_name" << std::endl;
        return -1;
    }
    std::string test_name = argv[1];
    std::string file_path = "/home/manh/Documents/DATN/Result/LV4/lv4_raw.binvox.bt";
    // std::string file_path_test = "/home/manh/Documents/DATN/Result/LV2/" + test_name + "/lv2_2022_" + test_name + ".binvox.bt";
    // std::string file_path_test = "/home/manh/Documents/DATN/Result/LV1/our1/" + test_name + "/" + test_name + ".binvox.bt";
    std::string file_path_test = "/home/manh/Documents/DATN/Result/LV4/dothi/2020/" + test_name + "/" + test_name + ".binvox.bt";
    octomap::OcTree *tree = new octomap::OcTree(0.5);
    octomap::OcTree *tree_test = new octomap::OcTree(0.5);
    size_t map_1;
    size_t map_1_;
    size_t map_2;
    size_t map_2_;

    // Process the first tree
    if (tree->readBinary(file_path))
    {
        std::cout << "Successfully loaded OctoMap from " << file_path << std::endl;

        // tree->expand();

        size_t free_count = 0;
        size_t occupied_count = 0;
        size_t occupied_z_ge_zero_count = 0; // Count for occupied nodes with z >= 0
        size_t occupied_z_ge_count = 0;
        size_t all_cells = 0;

        for (octomap::OcTree::iterator it = tree->begin(), end = tree->end(); it != end; ++it)
        {
            all_cells++;
            if (tree->isNodeOccupied(*it))
            {
                occupied_count++;

                // Get the z coordinate of the node
                octomap::point3d coord = it.getCoordinate();
                if (coord.z() >= 0.5)
                {
                    occupied_z_ge_zero_count++;
                }
                if (coord.z() >= -0.5)
                {
                    occupied_z_ge_count++;
                }
            }
            else
            {
                free_count++;
            }
        }
        map_1_ = occupied_z_ge_count;
        map_1 = occupied_z_ge_zero_count;
        std::cout << all_cells << std::endl;
        std::cout << "Number of nodes in the map: " << tree->size() << std::endl;
        std::cout << "Number of free nodes: " << free_count << std::endl;
        std::cout << "Number of occupied nodes: " << occupied_count << std::endl;
        std::cout << "Number of occupied nodes with z >= 0.5: " << occupied_z_ge_zero_count << std::endl;
        std::cout << "Number of occupied nodes with z >= -0.5: " << occupied_z_ge_count << std::endl;
    }
    else
    {
        std::cerr << "Failed to load OctoMap from " << file_path << std::endl;
        delete tree;
        return -1;
    }

    // // Input coordinates (x, y, z)
    // double x, y, z;
    // std::cout << "Enter the coordinates (x y z): ";
    // std::cin >> x >> y >> z;

    // // Search for the closest node to the given (x, y, z) coordinates
    // octomap::point3d query(x, y, z);
    // octomap::OcTreeNode *result = tree->search(query);

    // if (result != nullptr)
    // {
    //     if (tree->isNodeOccupied(result))
    //     {
    //         std::cout << "The nearest node to (" << x << ", " << y << ", " << z << ") is occupied." << std::endl;
    //     }
    //     else
    //     {
    //         std::cout << "The nearest node to (" << x << ", " << y << ", " << z << ") is free." << std::endl;
    //     }
    // }
    // else
    // {
    //     std::cout << "No node found near the given coordinates." << std::endl;
    // }

    delete tree;

    // Process the second tree
    if (tree_test->readBinary(file_path_test))
    {
        std::cout << "Successfully loaded OctoMap from " << file_path_test << std::endl;
        // tree_test->expand();
        size_t free_count = 0;
        size_t occupied_count = 0;
        size_t occupied_z_ge_zero_count = 0; // Count for occupied nodes with z >= 0
        size_t occupied_z_ge_count = 0;
        size_t all_cells = 0;

        for (octomap::OcTree::iterator it = tree_test->begin(), end = tree_test->end(); it != end; ++it)
        {
            all_cells++;
            if (tree_test->isNodeOccupied(*it))
            {
                occupied_count++;

                // Get the z coordinate of the node
                octomap::point3d coord = it.getCoordinate();
                if (coord.z() >= 0.5)
                {
                    occupied_z_ge_zero_count++;
                }
                if (coord.z() >= -0.5)
                {
                    occupied_z_ge_count++;
                }
            }
            else
            {
                free_count++;
            }
        }
        map_2_ = occupied_z_ge_count;
        map_2 = occupied_z_ge_zero_count;
        std::cout << all_cells << std::endl;
        std::cout << "Number of nodes in the map: " << tree_test->size() << std::endl;
        std::cout << "Number of free nodes: " << free_count << std::endl;
        std::cout << "Number of occupied nodes: " << occupied_count << std::endl;
        std::cout << "Number of occupied nodes with z >= 0.5: " << occupied_z_ge_zero_count << std::endl;
        std::cout << "Number of occupied nodes with z >= -0.5: " << occupied_z_ge_count << std::endl;
    }
    else
    {
        std::cerr << "Failed to load OctoMap from " << file_path_test << std::endl;
        delete tree_test;
        return -1;
    }

    delete tree_test;

    std::cout << "Exploration > 0.5 " << test_name << ": " << 100 * static_cast<double>(map_2) / map_1 << std::endl;
    std::cout << "Exploration > -0.5 " << test_name << ": " << 100 * static_cast<double>(map_2_) / map_1_ << std::endl;

    return 0;
}
