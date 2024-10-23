#include <octomap/octomap.h>
#include <octomap/OcTree.h>

int main(int argc, char **argv)
{
    // if (argc < 3) {
    //     std::cerr << "Usage: " << argv[0] << " <input.binvox.bt> <output.bt>" << std::endl;
    //     return -1;
    // }

    std::string input_file = "/home/manh/Documents/DATN/Result/LV2/t8/lv2_2022_t8.binvox.bt";
    std::string output_file = "/home/manh/Documents/DATN/Result/LV2/t8/lv2_2022_t16.bt";

    // Tạo cây OctoMap mới
    octomap::OcTree tree(0.5); // Sử dụng độ phân giải 0.5, có thể thay đổi

    // Đọc tệp .binvox.bt
    if (!tree.readBinary(input_file))
    {
        std::cerr << "Failed to read OctoMap from " << input_file << std::endl;
        return -1;
    }

    // Lưu vào tệp .bt
    if (!tree.writeBinary(output_file))
    {
        std::cerr << "Failed to write OctoMap to " << output_file << std::endl;
        return -1;
    }

    std::cout << "Successfully converted " << input_file << " to " << output_file << std::endl;
    return 0;
}
