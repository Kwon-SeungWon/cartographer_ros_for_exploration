#ifndef __MAP_MANAGER_HPP__
#define __MAP_MANAGER_HPP__

#include <string>
#include <iostream>
#include <filesystem>
#include <vector>
#include <unordered_set>
#include <algorithm>

namespace AMR
{

class MapManager
{
private:
    std::string dir_;
    std::vector<std::string> file_list_;
    std::vector<std::string> map_list_;

    std::string removeExtension(const std::string& filename);
    std::vector<std::string> processFilenames(const std::vector<std::string>& filenames);
public:
    MapManager(std::string dir);
    ~MapManager();
    
    void loadMapList();
    void deleteMap(std::string name);
    std::vector<std::string> getMapList();
    bool checkMap(std::string name);
    bool changeMapName(std::string orig_name, std::string name);
    std::string getDir();


};

} // namespace AMR

#endif