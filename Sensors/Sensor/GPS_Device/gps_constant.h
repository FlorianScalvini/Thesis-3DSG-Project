//
// Created by ubuntu on 26/10/22.
//

#ifndef OUTDOORNAV_GPS_CONSTANT_H
#define OUTDOORNAV_GPS_CONSTANT_H


#define MAXLINELENGTH  128 /// how long are max NMEA lines to parse?
#define NMEA_MAX_SENTENCE_ID 20 /// maximum length of a sentence ID name, including terminating 0
#define NMEA_MAX_SOURCE_ID   3 /// maximum length of a source ID name, including terminating 0



// different commands to set the update rate from once a second (1 Hz) to 10 times a second (10Hz)
#define PMTK_SET_NMEA_UPDATE_1HZ "$PMTK220,1000*1F" ///<  1 Hz
#define PMTK_SET_NMEA_UPDATE_2HZ "$PMTK220,500*2B"  ///<  2 Hz
#define PMTK_SET_NMEA_UPDATE_5HZ "$PMTK220,200*2C"  ///<  5 Hz
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F" ///< 10 Hz

#define PMTK_ENABLE_SBAS "$PMTK313,1*2E"
#define PMTK_SET_SBAS_TO_WAAS "$PMTK301,2*2D"


#define PMTK_SET_BAUD_115200 "$PMTK251,115200*1F" ///< 115200 bps
#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C"   ///<  57600 bps
#define PMTK_SET_BAUD_9600 "$PMTK251,9600*17"     ///<   9600 bps

// turn on only the first sentence (GPGLL)
#define PMTK_SET_NMEA_OUTPUT_GLLONLY  "$PMTK314,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on only the second sentence (GPRMC)
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn on ALL THE DATA
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn off output
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"



#define PMTK_LOCUS_STARTLOG "$PMTK185,0*22" ///< Start logging data
#define PMTK_LOCUS_STOPLOG "$PMTK185,1*23"  ///< Stop logging data
#define PMTK_LOCUS_STARTSTOPACK "$PMTK001,185,3*3C" ///< Acknowledge the start or stop command
#define PMTK_LOCUS_QUERY_STATUS "$PMTK183*38"  ///< Query the logging status
#define PMTK_LOCUS_ERASE_FLASH "$PMTK184,1*22" ///< Erase the log flash data
#define LOCUS_OVERLAP 0 ///< If flash is full, log will overwrite old data with new logs
#define LOCUS_FULLSTOP 1 ///< If flash is full, logging will stop

// standby command & boot successful message
#define PMTK_STANDBY "$PMTK161,0*28"
#define PMTK_STANDBY_SUCCESS "$PMTK001,161,3*3"  // Not needed currently
#define PMTK_AWAKE "$PMTK010,002*2D"

// ask for the release and version
#define PMTK_Q_RELEASE "$PMTK605*31"

// how long to wait when we're looking for a response
#define MAXWAITSENTENCE 5

#endif //OUTDOORNAV_GPS_CONSTANT_H
