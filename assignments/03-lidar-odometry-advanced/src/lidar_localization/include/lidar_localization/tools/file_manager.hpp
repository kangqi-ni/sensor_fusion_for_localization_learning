#ifndef LIDAR_LOCALIZATION_TOOLS_FILE_MANAGER_HPP_
#define LIDAR_LOCALIZATION_TOOLS_FILE_MANAGER_HPP_

#include <string>
#include <iostream>
#include <fstream>

namespace lidar_localization {
class FileManager{
  public:
    static bool CreateFile(std::ofstream& ofs, std::string file_path);
    static bool CreateDirectory(std::string directory_path);
};
}

#endif
