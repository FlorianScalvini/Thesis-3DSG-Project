//
// Created by Florian on 12/12/22.
//

#ifndef OUTDOORNAV_LAVMANAGER_H
#define OUTDOORNAV_LAVMANAGER_H
#include "lav_video_processor.h"
#include "lav_path_processing.h"
#include "lav_vocal.h"
#include "lav_sonifier.h"
#include "lav_constants.h"

enum ENUM_STATE
{
    INIT_STATE = 0,
    WAIT_DST = 1,
    NEAR_TARGET = 2,
    IN_TRANSIT = 3,
    DST_REACH = 4
};


class lavManager {
public:
    static void init(const char* path); // Init the controller of the system
    static void start(); // Start the navigation system
    static bool isVoiceControl();
    static void waitDst(); // Wait for the destination
    static void inTransit(); // The user is in transit
    static void nearTarget(); // The user is near the target
    static void reachDst();
    static void* start_manager(void* args);
    static void start_thread_manager_stream();
    static void release(); // Release the system
    static void saveMode();


private:
    static int _countPathEmission;
    static void process();
    static bool close_thread;
    static int state;
    static unsigned int _dst;
    static lavPathProcessing* _path;
    static void valueToSoundValue(int value);
};


#endif //OUTDOORNAV_LAVMANAGER_H
