#include "udp_camera.h"





UdpCamera::UdpCamera(const std::string& addr, int port)
    : f_port(port)
    , f_addr(addr)
{
    printf("UdpCamera::UdpCamera");

    char decimal_port[16];
    snprintf(decimal_port, sizeof(decimal_port), "%d", f_port);
    decimal_port[sizeof(decimal_port) / sizeof(decimal_port[0]) - 1] = '\0';
    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;
    int r(getaddrinfo(addr.c_str(), decimal_port, &hints, &f_addrinfo));

    if(r != 0 || f_addrinfo == NULL)
    {
        printf("invalid address or port for UDP socket: ");
        printf(addr.c_str());
        //lavLog::LAVLOG(("invalid address or port for UDP socket: ", addr + ":" + decimal_port + "\"").c_str());

        //throw udp_client_server_runtime_error(("invalid address or port for UDP socket: \"" + addr + ":" + decimal_port + "\"").c_str());
    }
    f_socket = socket(f_addrinfo->ai_family, SOCK_DGRAM | SOCK_CLOEXEC, IPPROTO_UDP);
    if(f_socket == -1)
    {
        freeaddrinfo(f_addrinfo);
        printf(("could not create UDP socket for: \"" + addr + ":" + decimal_port + "\"").c_str());
        //throw udp_client_server_runtime_error(("could not create UDP socket for: \"" + addr + ":" + decimal_port + "\"").c_str());
    }
    r = bind(f_socket, f_addrinfo->ai_addr, f_addrinfo->ai_addrlen);
    if(r != 0)
    {
        freeaddrinfo(f_addrinfo);
        close(f_socket);
        printf(("could not bind UDP socket with: \"" + addr + ":" + decimal_port + "\"").c_str());
        //throw udp_client_server_runtime_error(("could not bind UDP socket with: \"" + addr + ":" + decimal_port + "\"").c_str());
    }
}


UdpCamera::~UdpCamera()
{
    freeaddrinfo(f_addrinfo);
    close(f_socket);
}


int UdpCamera::get_socket() const
{
    return f_socket;
}


int UdpCamera::get_port() const
{
    return f_port;
}


std::string UdpCamera::get_addr() const
{
    return f_addr;
}


int UdpCamera::recv(char *msg, size_t max_size)
{
    return ::recv(f_socket, msg, max_size, 0);
}


int UdpCamera::timed_recv(char *msg, size_t max_size, int max_wait_ms)
{
    fd_set s;
    FD_ZERO(&s);
    FD_SET(f_socket, &s);
    struct timeval timeout;
    timeout.tv_sec = max_wait_ms / 1000;
    timeout.tv_usec = (max_wait_ms % 1000) * 1000;
    int retval = select(f_socket + 1, &s, &s, &s, &timeout);
    if(retval == -1)
    {
        // select() set errno accordingly
        return -1;
    }
    if(retval > 0)
    {
        // our socket has data
        return ::recv(f_socket, msg, max_size, 0);
    }

    // our socket has no data
    errno = EAGAIN;
    return -1;
}
/*
cv::Mat UdpCamera::getNextFrame()
{
    lavLog::LAVLOG("UdpCamera::getNextFrame/n");

    int nbByteReceived = recv(buffer, 10000);

    //printf("data received nbByteReceived=%i\n", nbByteReceived);

    std::vector<unsigned char> vect(buffer, buffer+nbByteReceived);    
    cv::Mat decodedImg = cv::imdecode(vect, 0);
    
    return decodedImg;
}
*/

cv::Mat UdpCamera::getNextFrame()
{
    /*
    int nbByteReceived = recv(buffer, 10000);

    printf("data received nbByteReceived=%i\n", nbByteReceived);

    std::vector<unsigned char> vect(buffer, buffer+nbByteReceived);    
    cv::Mat decodedImg = cv::imdecode(vect, 0);
    */

    int height = 240;
    int width = 320;

    //cv::Mat reconstructedImage(height,width, CV_8UC1, cv::Scalar(0));
    
    cv::Mat decodedImg(height,width, CV_8UC1, cv::Scalar(0));

    int expectedByteNumber = decodedImg.total() * decodedImg.elemSize();
    char receivedByte[expectedByteNumber];
    int totalReceivedByte = 0;

    totalReceivedByte = 0;

    while (totalReceivedByte<expectedByteNumber) {

        //printf("Waiting for data\n");
        int nbByteReceived = recv(receivedByte+totalReceivedByte, expectedByteNumber-totalReceivedByte);

        if ((nbByteReceived<12000) && (totalReceivedByte==0)) {
            printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!   PB\n");
        }
        else {
            //printf("OK\n");
            totalReceivedByte+=nbByteReceived;                        
        }
        //printf("%i\n", nbByteReceived);
        //printf("%i\n", totalReceivedByte);    
        
    }

    memcpy (decodedImg.data, receivedByte, expectedByteNumber);

    //imshow("received frame",reconstructedImage);

    
    return decodedImg;
}

void UdpCamera::release()
{
    freeaddrinfo(f_addrinfo);
    close(f_socket);
}
