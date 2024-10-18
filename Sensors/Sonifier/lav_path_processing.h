//
// Created by Florian on 08/12/22.
//

#ifndef OUTDOORNAV_LAV_PATH_PROCESSING_H
#define OUTDOORNAV_LAV_PATH_PROCESSING_H

#include "Graph/Map.h"
#include "Sensor/GPS_Device/adafruit_gps.h"
#include "lav_constants.h"
#include <iostream>
#include <fstream>
#include <vector>

class lavPathProcessing {
public:
    lavPathProcessing();
    void init(const char*);
    char startPath(const std::string& nodeDst); // Start path to a desired destination node
    void updateCurrentCoor();
    char isUsersApproching();
    double getBearings(); // Return the bearing
    double getDistance(); // Return the distance to the next node
    double getLongitude();
    double getLatitude();
    bool isCrossingNode(); // The node related to a crossing zebra
    char nextNode(); // Return the next node index
    void showPath(); // Print the path to follow
    static lavPathProcessing *__singleton;
    static lavPathProcessing *getSingleton();
    void saveGPS(std::string path);
    bool openSaveFile(std::string);
    void closeSaveFile();
    GPSCapture* _captureGPS; // GPS sensor

private:
    unsigned int _currPosition;
    Map* _graph; // Graph map
    Node* _dstNode; // Destination node
    Node* _currentNode; // Current node
    std::vector<Node*> _currentPath; // List of node to reach
    GeographicCoordinate _previousCoor; // Previous user coordinate
    float _prevDistanceToTarget;
    char status;
};


#endif //OUTDOORNAV_LAV_PATH_PROCESSING_H
