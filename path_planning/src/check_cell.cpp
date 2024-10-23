#include <octomap/octomap.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

int main(int argc, char **argv)
{
    std::string file_path = "/home/manh/Documents/DATN/Result/LV2/lv2_raw.binvox.bt";
    std::string txt_file_path = "/home/manh/sp_explor/src/path_planning/test/cells.txt"; // Path to your .txt file with coordinates

    octomap::OcTree *tree = new octomap::OcTree(0.5);

    // Load the OctoMap from file
    if (!tree->readBinary(file_path))
    {
        std::cerr << "Failed to load OctoMap from " << file_path << std::endl;
        delete tree;
        return -1;
    }

    std::cout << "Successfully loaded OctoMap from " << file_path << std::endl;

    // Open the text file containing coordinates
    std::ifstream infile(txt_file_path);
    if (!infile)
    {
        std::cerr << "Failed to open coordinates file " << txt_file_path << std::endl;
        delete tree;
        return -1;
    }

    std::string line;
    while (std::getline(infile, line))
    {
        std::istringstream iss(line);
        double x, y, z;
        if (!(iss >> x >> y >> z))
        {
            std::cerr << "Error reading coordinates from line: " << line << std::endl;
            continue; // Skip invalid lines
        }

        // Search for the nearest node to the current (x, y, z) coordinates
        octomap::point3d query(round(x), round(y), round(z));
        octomap::OcTreeNode *result = tree->search(query);

        if (result != nullptr)
        {
            if (tree->isNodeOccupied(result))
            {
                std::cout << "Point (" << x << ", " << y << ", " << z << ") is occupied." << std::endl;
            }
            else
            {
                std::cout << "Point (" << x << ", " << y << ", " << z << ") is free." << std::endl;
            }
        }
        else
        {
            std::cout << "No node found near point (" << x << ", " << y << ", " << z << ")." << std::endl;
        }
    }

    // Clean up
    infile.close();
    delete tree;

    return 0;
}
