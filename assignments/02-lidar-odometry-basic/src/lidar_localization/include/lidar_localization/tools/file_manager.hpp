#ifndef LIDAR_LOCALIZATION_TOOLS_FILE_MANAGER_HPP_
#define LIDAR_LOCALIZATION_TOOLS_FILE_MANAGER_HPP_

#include <string>
#include <iostream>
#include <fstream>

namespace lidar_localization {
class FileManager{
  public:
    // Use ofstream to open file specified by file_path
    static bool CreateFile(std::ofstream& ofs, std::string file_path);
    // Create a directory specified by directory_path
    static bool CreateDirectory(std::string directory_path);
};
}

#endif
