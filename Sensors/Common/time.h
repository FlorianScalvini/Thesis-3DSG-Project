//
// Created by Florian on 08/12/22.
//

#ifndef OUTDOORNAV_TIME_H
#define OUTDOORNAV_TIME_H

struct Time
{
    unsigned char hour;          ///< GMT hours
    unsigned char minute;        ///< GMT minutes
    unsigned char seconds;       ///< GMT seconds
    unsigned short milliseconds; ///< GMT milliseconds
    unsigned int year;          ///< GMT year
    unsigned char month;         ///< GMT month
    unsigned char day;           ///< GMT day
    Time()
    {
        hour = minute = seconds = month = day = 0;
        milliseconds = 0;
        year = 0;
    }
};

#endif //OUTDOORNAV_TIME_H
