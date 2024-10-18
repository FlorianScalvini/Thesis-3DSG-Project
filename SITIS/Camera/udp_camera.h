
#ifndef LAV_UDP_CAMERA
#define LAV_UDP_CAMERA

#include <opencv2/opencv.hpp>

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>

#include "video_capture.h"

class UdpCamera: public VideoCapture
{
public:
    UdpCamera(const std::string& addr, int port);
    ~UdpCamera();

    int                 get_socket() const;
    int                 get_port() const;
    std::string         get_addr() const;

    int                 recv(char *msg, size_t max_size);
    int                 timed_recv(char *msg, size_t max_size, int max_wait_ms);

    virtual cv::Mat getNextFrame();
    virtual void release();

private:
    int                 f_socket;
    int                 f_port;
    std::string         f_addr;
    struct addrinfo *   f_addrinfo;

    char buffer[10000];
};

#endif
