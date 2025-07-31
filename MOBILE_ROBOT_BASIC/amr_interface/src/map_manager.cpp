#include "amr_interface/map_manager.hpp"

namespace AMR
{

MapManager::MapManager(std::string dir) : dir_(dir)
{
    getMapList();
}   

MapManager::~MapManager()
{

}

void MapManager::loadMapList()
{
    file_list_.clear();
    try {
        std::filesystem::path dir_path(dir_);

        if (std::filesystem::exists(dir_path) && std::filesystem::is_directory(dir_path)) {
            for (const auto& entry : std::filesystem::directory_iterator(dir_path)) {
                if (std::filesystem::is_regular_file(entry)) {
                    file_list_.push_back(entry.path().filename().string());
                }
            }
        } else {
            std::cerr << "Directory does not exist or is not a directory.\n";
        }
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Filesystem error: " << e.what() << '\n';
    }
}

std::vector<std::string> MapManager::getMapList()
{
    map_list_.clear();
    loadMapList();
    map_list_ = processFilenames(file_list_);
    std::cout << "dir_: " << dir_ << std::endl;

    return map_list_;
}

std::string MapManager::removeExtension(const std::string& filename) 
{
    std::filesystem::path p(filename);
    return p.stem().string();
}

std::vector<std::string> MapManager::processFilenames(const std::vector<std::string>& filenames) 
{
    std::unordered_set<std::string> unique_names;

    for (const auto& filename : filenames) 
    {
        std::string name_without_ext = removeExtension(filename);
        unique_names.insert(name_without_ext);
    }

    std::vector<std::string> result(unique_names.begin(), unique_names.end());
    return result;
}

void MapManager::deleteMap(std::string name)
{
    getMapList();
    try 
    {
        if (!std::filesystem::exists(dir_) || !std::filesystem::is_directory(dir_)) 
        {
            std::cerr << "Directory does not exist or is not a dir_.\n";
            return;
        }

        for (const auto& entry : std::filesystem::directory_iterator(dir_)) 
        {
            if (std::filesystem::is_regular_file(entry)) 
            {
                std::filesystem::path file_path = entry.path();
                std::string filename = file_path.stem().string();
                if (filename == name) 
                {
                    std::cout << "Deleting file: " << file_path << std::endl;
                    std::filesystem::remove(file_path);
                }
            }
        }
    } 
    catch (const std::filesystem::filesystem_error& e) {

        std::cerr << "Filesystem error: " << e.what() << '\n';
    }
}

bool MapManager::checkMap(std::string name)
{
    getMapList();
    auto it = std::find(map_list_.begin(), map_list_.end(), name);
    if (it != map_list_.end()) 
        return true;
    else {
        std::cout << "No Map !!!!!!!!!!!!!" << std::endl;
        return false;
    }
}

bool MapManager::changeMapName(std::string orig_name, std::string name)
{
    getMapList();
    if(checkMap(orig_name))
    {

        std::filesystem::path oldYaml = std::filesystem::path(dir_) / (orig_name+".yaml");
        std::filesystem::path oldPng = std::filesystem::path(dir_) / (orig_name+".pgm");

        std::filesystem::path newYaml = std::filesystem::path(dir_) / (name+".yaml");
        std::filesystem::path newPng = std::filesystem::path(dir_) / (name+".pgm");

        try 
        {
            if (std::filesystem::exists(oldYaml)) 
            {
                std::filesystem::rename(oldYaml, newYaml);
                std::cout << "Renamed " << oldYaml << " to " << newYaml << std::endl;
            } 
            else 
            {
                std::cout << oldYaml << " does not exist." << std::endl;
                return false;
            }

            if (std::filesystem::exists(oldPng)) 
            {
                std::filesystem::rename(oldPng, newPng);
                std::cout << "Renamed " << oldPng << " to " << newPng << std::endl;
            } 
            else 
            {
                std::cout << oldPng << " does not exist." << std::endl;
                return false;
            }
        } 
        catch (const std::filesystem::filesystem_error& e) 
        {
            std::cerr << "Filesystem error: " << e.what() << std::endl;
            return false;
        } 
        catch (const std::exception& e) 
        {
            std::cerr << "Error: " << e.what() << std::endl;
            return false;
        }
    }
    else
        return false;
    return true;
}

std::string MapManager::getDir()
{
    getMapList();
    return dir_;
}


} // namespace AMR