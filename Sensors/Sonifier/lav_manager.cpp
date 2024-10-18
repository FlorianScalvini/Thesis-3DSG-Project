//
// Created by Florian on 12/12/22.
//

#include "lav_manager.h"
#include <opencv2/core/utils/filesystem.hpp>
#include <iostream>
#include "Qt/Qt.h"

int lavManager::state;
bool lavManager::close_thread;
unsigned int lavManager::_dst;
lavPathProcessing* lavManager::_path;
int lavManager::_countPathEmission;



void lavManager::init(const char* path)
{
    printf("Load map\n");
    _path = new lavPathProcessing();
    _path->init(path);
    state = WAIT_DST;
    _countPathEmission = 0;
    lavVideoProcessor::changeStatus(lavVideoProcessor::SILENCE);
}

bool lavManager::isVoiceControl()
{
    if(state != IN_TRANSIT)
        return true;
    else
        return false;
}

void lavManager::reachDst() {
    std::cout<<"Nouvelle destination: (y/n) "<<std::endl;
    std::string str;
    std::getline(std::cin, str);
    if(str == "y")
        state = WAIT_DST;
    else
    {
        lavManager::release();
    }
}

void lavManager::nearTarget()
{
    //lavAudioMixer::readSound("destination");
    int rtn = _path->nextNode();
    switch (rtn) {
        case 0: {
            printf("Change to the next node\n");
            if(_path->isCrossingNode())
                printf("The next node is a crossway \n");

            int distance = (int)_path->getDistance();
            valueToSoundValue(distance);
            _countPathEmission = 0;
            state = IN_TRANSIT;
            break;
        }
        case 1:
            printf("Destination reached\n");
            state = DST_REACH;
            break;
        default:
            fprintf(stderr, "Error with the selection of the next target\n");
            state = WAIT_DST;
    }
}

void lavManager::start() {
    lavVocal::readSound("destination");
    state = WAIT_DST;
}

/*
 * Navigation mode
 */
void lavManager::inTransit()
{

    int soundPath;
    _path->updateCurrentCoor();
    char userApproch = _path->isUsersApproching();
    if(userApproch == 2)
    {
        state = NEAR_TARGET;
        return;
    }
    float angleTarget = _path->getBearings(); // Get the bearing between the user GPS position adn the destination coordonate
    float angleCompass = lavImuAcquisition::getCompass(); // Get the user orientation
    int realAngle = (int)(angleCompass - angleTarget); // The angle to follow is the correction of the target with the compass
    lavVideoProcessor::_computeSegmentation(); // perform the segmentation
    soundPath = lavVideoProcessor::getPathCorrection(realAngle); // Correct the angle on the navigable position
    //printf("Angle: %f %f %i %i\n", angleTarget, angleCompass, realAngle, soundPath);
    lavSonifier::setAngle(&soundPath); // Push the position sound
    usleep(250000);
    lavSonifier::setAngle(nullptr);
    usleep(250000);
}


void lavManager::process()
{
    auto time = std::chrono::system_clock::now();
    auto timer = std::chrono::system_clock::to_time_t(time);
    // convert to broken time
    std::tm bt = *std::localtime(&timer);
    cv::utils::fs::createDirectory(SAVE_PATH);
    std::string savepath = std::string(SAVE_PATH) + "/" + std::to_string(bt.tm_hour) + "_" + std::to_string(bt.tm_min) + "_" + std::to_string(bt.tm_sec);
    cv::utils::fs::createDirectory(savepath);
    usleep(30000);
    std::string path = savepath + "/GPSData.txt";
    FILE * fp = fopen(path.c_str(), "a+");
    state = WAIT_DST;
    lavVideoProcessor::changeStatus(lavVideoProcessor::VideoState::SILENCE);
    while(!close_thread)
    {
        while(lavVocal::isReading())
        {
            usleep(250);
        }
        switch (state) {
            case WAIT_DST:
                waitDst();
                lavVideoProcessor::changeStatus(lavVideoProcessor::VideoState::NAV);
                break;
            case IN_TRANSIT:
                inTransit();
                break;
            case NEAR_TARGET:
                nearTarget();
                break;
            case DST_REACH:
                reachDst();
                break;
            default:
                state = WAIT_DST;
                return;
        }

        float latitude = _path->_captureGPS->data.coor2D.latitude;
        float longitude =  _path->_captureGPS->data.coor2D.longitude;
        int heure = _path->_captureGPS->data.time.hour;
        int min = _path->_captureGPS->data.time.minute;
        int second = _path->_captureGPS->data.time.seconds;
        int mil = _path->_captureGPS->data.time.milliseconds;
        //printf("%i;%i;%i;%i;%f;%f\n",  heure,  min,  second,  mil, latitude, longitude);
        fprintf(fp, "%i;%i;%i;%i;%f;%f\n",  heure,  min,  second,  mil, latitude, longitude);
    }
}


void lavManager::waitDst()
{
    lavVocal::readSound("destination"); // Emit verbal sound

    // List of positions
    std::vector<std::string> choices = {"1725230613", "2171266122", "332295921", "1725230613", "3012267843"};
    std::string char_dst = choices[QtFrame::selectAttribut(choices)];

    // User indicates the destination
    std::cout<<"Enter the destination: "<<std::endl;
    std::locale::global(std::locale::classic());
    if(_path != nullptr && !char_dst.empty() && std::all_of(char_dst.begin(), char_dst.end(), [](const char i){return std::isdigit(i);}))
    {
        printf("Destination: %s\n", char_dst.c_str());
        char value = _path->startPath(char_dst);

        // If path is impossible
        if(value == EXIT_FAILURE)
        {
            usleep(1000);
        }
        else
        {
            _path->showPath(); // Show the path with the list of intermediate nodes
            state = IN_TRANSIT; // Change the manager state in navigation mode
            lavVideoProcessor::changeStatus(lavVideoProcessor::VideoState::NAV); // Set the video processing state in navigation mode
            std::cout<<"Start path"<< std::endl;
            int distance = (int)_path->getDistance(); // Get the distance to the next node
            valueToSoundValue(distance); // Emit the distance sound
        }
    }
    else
        usleep(1000);
}


void lavManager::valueToSoundValue(int value) {
    std::cout<<"Distance :"<< value << std::endl;
    if (value < 90) {
        lavVocal::readSound("distance");
        while (lavVocal::isReading())
            usleep(250);
        if(value < 20)
            lavVocal::readSound(std::to_string(value));
        else
        {
            lavVocal::readSound(std::to_string((int) floor(value / 10.0) * 10));
            while (lavVocal::isReading())
                usleep(250);
            lavVocal::readSound(std::to_string((int) (value % 10)));
        }
        while (lavVocal::isReading())
            usleep(250);
        lavVocal::readSound("metre");
    } else
    {
        lavVocal::readSound("too_far");
    }
}



void lavManager::release()
{
    close_thread = true;
}

void lavManager::saveMode()
{
    auto time = std::chrono::system_clock::now();
    auto timer = std::chrono::system_clock::to_time_t(time);
    // convert to broken time
    std::tm bt = *std::localtime(&timer);
    cv::utils::fs::createDirectory(SAVE_PATH);
    std::string savepath = std::string(SAVE_PATH) + "/" + std::to_string(bt.tm_hour) + "_" + std::to_string(bt.tm_min) + "_" + std::to_string(bt.tm_sec);
    cv::utils::fs::createDirectory(savepath);
    usleep(30000);
    lavImuAcquisition::openSaveFile(savepath);
    _path->openSaveFile(savepath);
    lavVideoProcessor::setSavePath(savepath);
    usleep(30000);
    lavVideoProcessor::changeStatus(lavVideoProcessor::SAVE);
    lavImuAcquisition::setSaveMode(true);
    _path->saveGPS(savepath);
}




void *lavManager::start_manager(void *args) {

    auto* thisPointer = (lavManager*) args;
    thisPointer->process();
    return nullptr;
}


void lavManager::start_thread_manager_stream() {

    std::cout<<"Start manager ..... : \n"<<std::endl;
    pthread_t thread_video_processing;
    pthread_create(&thread_video_processing, nullptr, start_manager, (void*)nullptr);
}
