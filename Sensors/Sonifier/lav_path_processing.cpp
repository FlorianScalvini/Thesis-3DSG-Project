//
// Created by Florian on 08/12/22.
//

#include "lav_path_processing.h"
#include <unistd.h>


lavPathProcessing* lavPathProcessing::__singleton = nullptr;

lavPathProcessing* lavPathProcessing::getSingleton()
{
    if (__singleton ==nullptr) {
        __singleton = new lavPathProcessing();
    }
    return __singleton;
}

lavPathProcessing::lavPathProcessing() {
    this->_dstNode = nullptr;
    this->_captureGPS = nullptr;
    this->_graph = nullptr;
    this->_currentNode = nullptr;
    _currPosition = -1;
    status = 0;
}



void lavPathProcessing::init(const char * osm) {
    _captureGPS = new AdafruitGPS();
    _captureGPS->start(UART_GPS, B9600);
    _graph = new Map(osm);
    status = 1;
}

bool lavPathProcessing::openSaveFile(std::string dir) {
    std::string path = dir + "/GPSData.txt";
    FILE *fp = fopen(path.c_str(), "w+");
    fclose(fp);
    return EXIT_SUCCESS;
}

void lavPathProcessing::closeSaveFile() {

}



/*
 * Start path from current position to nodeDst
 *
 * Return 0 if path is found
 * Return 1 if path is not found
 * Return 2 if destination not exist
 */

char lavPathProcessing::startPath(const std::string& nodeDst)
{
    if(status)
    {
        _captureGPS->start(UART_GPS, B9600);
        usleep(1000);
        _captureGPS->write(PMTK_SET_NMEA_OUTPUT_RMCONLY, sizeof(PMTK_SET_NMEA_OUTPUT_RMCONLY));
        _captureGPS->write(PMTK_SET_NMEA_UPDATE_10HZ, sizeof(PMTK_SET_NMEA_UPDATE_10HZ));
        _captureGPS->write(PMTK_ENABLE_SBAS, sizeof(PMTK_ENABLE_SBAS));
        _captureGPS->write(PMTK_SET_SBAS_TO_WAAS, sizeof(PMTK_SET_SBAS_TO_WAAS));
        usleep(100000);
        this->_dstNode = _graph->getNodeFromId(nodeDst);
        /*if(this->_dstNode == nullptr)
        {
            std::cerr<<"The node with the identifier "<< nodeDst << " doesn't exist in the current map" << std::endl;
            return 1;
        }*/
        this->_previousCoor = GeographicCoordinate(5.0735987, 47.3113556);

        do{
            while(this->_captureGPS->readNextValue())
                usleep(15000);
            //this->_captureGPS->data.coor2D = GeographicCoordinate(5.0735987, 47.3113556);
            this->_currentNode = _graph->getNearestPoint( this->_captureGPS->data.coor2D);
        } while (this->_currentNode == nullptr);
        this->_previousCoor = this->_captureGPS->data.coor2D;
        this->_currentPath = this->_graph->searchPath(this->_currentNode, this->_dstNode, Path_UCS);
        if(this->_currentPath.size() == 0)
        {
            std::cerr<<"No path found between the current position and the destination" << std::endl;
            return EXIT_FAILURE;
        }
        if(this->_currentPath.empty())
        {
            std::cerr<<"A path from the node "<< this->_currentNode->id << " to the node "<< nodeDst << " is unreachable." << std::endl;
            return 2;
        }
        this->_currPosition = 0;
        status = 1;
        return EXIT_SUCCESS;
    }
    std::cerr<<"The graph is not initialized" << std::endl;
    return EXIT_FAILURE;
}

/*
 * Return : 0 - Eloignement
 *          1 - Rapprochement
 *          2 - A cote de destination
 */

char lavPathProcessing::isUsersApproching() {
    if(status == 1)
    {
        bool result;
        float dstTarget =  GeographicCoordinate::toDistance(this->_captureGPS->data.coor2D, this->_currentNode->coordinate);
        printf("Distance : %f\n", dstTarget);
        if(dstTarget < THRESH_DST_TARGET)
            return 2;
        result = GeographicCoordinate::toDistance(this->_previousCoor, this->_currentNode->coordinate) < dstTarget;
        return (char)result;
    }
    return -1;
}

void lavPathProcessing::updateCurrentCoor() {
    _previousCoor = this->_captureGPS->data.coor2D; // Set the current coordinate to the previous
    while(this->_captureGPS->readNextValue()) // Update the current coordinate with the GPS
        usleep(15000);
}

double lavPathProcessing::getBearings() {
    if(status == 1)
    {
        return GeographicCoordinate::toBearing(this->_captureGPS->data.coor2D, this->_currentNode->coordinate);
    }
    return 0;
}

double lavPathProcessing::getDistance() {
    if(status == 1)
    {
        return GeographicCoordinate::toDistance(this->_captureGPS->data.coor2D, this->_currentNode->coordinate);
    }
    return 0;
}


char lavPathProcessing::nextNode() {
    if(_currPosition >= 0 && _currPosition < _currentPath.size() - 1 && status == 1)
    {
        _currPosition++;
        _currentNode = _currentPath[_currPosition];
        return 0;
    }
    else if(_currPosition == _currentPath.size() - 1)
    {
        return 1;
    }
    return -1;
}

double lavPathProcessing::getLatitude() {
    if(status == 1 && this->_captureGPS->fix)
        return this->_captureGPS->data.coor2D.longitude;
    else
        return 0;
}

double lavPathProcessing::getLongitude() {
    if(status == 1 && this->_captureGPS->fix)
        return this->_captureGPS->data.coor2D.longitude;
    else
        return 0;
}



void lavPathProcessing::saveGPS(std::string dir)
{

    _captureGPS->write(PMTK_SET_NMEA_OUTPUT_RMCONLY, sizeof(PMTK_SET_NMEA_OUTPUT_RMCONLY));
    _captureGPS->write(PMTK_SET_NMEA_UPDATE_10HZ, sizeof(PMTK_SET_NMEA_UPDATE_10HZ));
    _captureGPS->write(PMTK_ENABLE_SBAS, sizeof(PMTK_ENABLE_SBAS));
    _captureGPS->write(PMTK_SET_SBAS_TO_WAAS, sizeof(PMTK_SET_SBAS_TO_WAAS));
    usleep(10000);
    std::string path = dir + "/GPSData.txt";
    while(1)
    {

        FILE * fp = fopen(path.c_str(), "a+");
        this->updateCurrentCoor();
        float latitude = _captureGPS->data.coor2D.latitude;
        float longitude =  _captureGPS->data.coor2D.longitude;
        int heure = _captureGPS->data.time.hour;
        int min = _captureGPS->data.time.minute;
        int second = _captureGPS->data.time.seconds;
        int mil = _captureGPS->data.time.milliseconds;
        printf("%i;%i;%i;%i;%f;%f\n",  heure,  min,  second,  mil, latitude, longitude);
        fprintf(fp, "%i;%i;%i;%i;%f;%f\n",  heure,  min,  second,  mil, latitude, longitude);
        fclose(fp);
    }

}
bool lavPathProcessing::isCrossingNode() {
    return _currentNode->crossing;
}

void lavPathProcessing::showPath()
{
    for(unsigned int i = _currPosition; i < _currentPath.size(); i++)
    {
        std::cout<<_currentPath[i]->id;
        if(i < (_currentPath.size()-1))
            std::cout<<" --> ";
        else
            std::cout<<std::endl;
    }
}



