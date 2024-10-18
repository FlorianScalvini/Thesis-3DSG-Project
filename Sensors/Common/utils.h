//
// Created by Florian on 19/12/22.
//

#ifndef OUTDOORNAV_UTILS_H
#define OUTDOORNAV_UTILS_H
#include <map>
#include <dirent.h>
#include <cstring>

static char getFilesInDirectory(const std::string& directory, std::map<std::string, std::string>& files)
{
    DIR *dir;
    struct dirent *entry;
    if ((dir = opendir (directory.c_str())) != nullptr) {
        /* print all the files and directories within directory */
        while ((entry = readdir (dir)) != nullptr) {
            std::string file_name = entry->d_name;
            unsigned long posExtension = file_name.find_last_of('.');
            std::string file_without_extension = file_name.substr(0, posExtension);
            std::string file_extension = file_name.substr(posExtension + 1);
            if (file_extension == "wav") {
                files[file_without_extension] = directory + "/" + file_name;
            }
        }
        closedir (dir);
    } else {
        /* could not open directory */
        perror ("");
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

static float convertDegMinToDecDeg(std::string degMin){
    double decDeg = 0.0;
    float sign = 1.0;
    if(degMin[0] == '-')
        sign = -1.0;
    int pos = degMin.find('.');
    if (pos != std::string::npos) {
        if(sign == 1.0)
            decDeg = -(std::stof(degMin.substr(1, pos))) - std::stof(degMin.substr(pos + 1)) / 60.0;
        else
            decDeg = std::stof(degMin.substr(0, pos)) + std::stof(degMin.substr(pos + 1)) / 60.0;
    }
    return decDeg;
}

#endif //OUTDOORNAV_UTILS_H
