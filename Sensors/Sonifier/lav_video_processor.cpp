//
// Created by Florian on 08/12/22.
//
#include "lav_video_processor.h"
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <unistd.h>

cv::Mat lavVideoProcessor::_inputMat;
cv::Mat lavVideoProcessor::_inputMatColor;
bool lavVideoProcessor::_newVideoProcessingIsAvailable;
bool lavVideoProcessor::_closeVideo;
int lavVideoProcessor::status;
Eigen::Quaterniond lavVideoProcessor::_previousOrientation;
Eigen::Quaterniond lavVideoProcessor::_orientation;
cv::Point3f lavVideoProcessor::_positionMotion;
Sort* lavVideoProcessor::_track;
Detecteur* lavVideoProcessor::_detectObstacle;
Segmentation* lavVideoProcessor::_segSidewalk;
std::mutex lavVideoProcessor::_mutexImg;
std::mutex lavVideoProcessor::_mutexSeg;
std::vector<ObjectBoundingBox>  lavVideoProcessor::_staticObstacle;
std::vector<ObjectBoundingBox> lavVideoProcessor::_dynamicObstacle;
std::string lavVideoProcessor::_savePath;
cv::Mat lavVideoProcessor::_segImg;
int lavVideoProcessor::pedestrianLightStatus;


void lavVideoProcessor::init(const char* detectionPath, const char* segmenPath) {

    _track = new Sort[NUM_TRACKED_ID];
    for(int i = 0; i < NUM_TRACKED_ID; i++)
    {
        _track[i] = Sort();
    }
    lavVideoProcessor::_detectObstacle = new Yolov8TRT(detectionPath);
    lavVideoProcessor::_segSidewalk = new SegmentationTensorRT(segmenPath);
    _segImg = cv::Mat(_segSidewalk->getOutputSize(), CV_8U);
    status = SILENCE;
    _mutexImg.unlock();
    _mutexSeg.unlock();
}


void lavVideoProcessor::release() {
	_closeVideo = true;
}

void lavVideoProcessor::start()
{
    _newVideoProcessingIsAvailable = false;
    status = SILENCE;
    lavVideoProcessor::_processVideo();
}

void lavVideoProcessor::_processVideo(){
    //printf("");
    //_captureFramesAndIMU();
    while (!_closeVideo) {
        switch (status) {
            case NAV:
                _captureFramesAndIMU();
                _computeObstacle();
                break;
            case CROSSWALK_SIGNAL_PATH:
                _captureFramesAndIMU();
                _computeCrosswalk();
                break;
            case TRAFFIC_LIGHT_SEARCH:
                _captureFramesAndIMU();
                _computeTrafficLight();
            case SAVE:
                _captureFramesAndIMU();
                _saveImg();
                break;
            default:
                usleep(500);
        }
    }
}


void lavVideoProcessor::_captureFramesAndIMU()
{
    _mutexImg.lock();
    _previousOrientation = _orientation;
    lavCameraAcquisition::getNextFrame(_inputMat, _inputMatColor);
    lavImuAcquisition::getNextValue(_orientation, _positionMotion);
    _mutexImg.unlock();
}

void lavVideoProcessor::_computeObstacle()
{
    cv::Mat _nearPixel = _computeNearPixel();
    cv::Mat  _obstacle = cv::Mat(cv::Size(FRAME_WIDTH_SONIFIED, FRAME_HEIGHT_SONIFIED), CV_8U);
    cv::resize(_nearPixel, _nearPixel, cv::Size(FRAME_WIDTH_SONIFIED, FRAME_HEIGHT_SONIFIED), cv::INTER_NEAREST);
    _obstacle.setTo(0);
    _staticObstacle.clear();
    _dynamicObstacle.clear();
    _computeObjectDetection();

    for(auto & i : _staticObstacle)
    {
        unsigned short depth = i.depth;
        if(depth < THRESH_DST_SONIFY)
        {
            int x = (int)i.bbox.cx() * FRAME_WIDTH_SONIFIED / COLOR_FRAME_WIDTH;
            int y = (int)i.bbox.cy() * FRAME_HEIGHT_SONIFIED / COLOR_FRAME_HEIGHT;
            if(depth < THRESH_DST_SONIFY_NEAR)
                _obstacle.at<unsigned char>(y,x) =  255;
            else
                _obstacle.at<unsigned char>(y,x) =  255 * ( 1 - (depth - THRESH_DST_SONIFY_NEAR) / (THRESH_DST_SONIFY - THRESH_DST_SONIFY_NEAR));
        }
    }

    for(auto & i : _dynamicObstacle)
    {
        unsigned short depth = i.depth;
        if(depth < THRESH_DST_SONIFY)
        {
            int x = (int)i.bbox.cx() * FRAME_WIDTH_SONIFIED / COLOR_FRAME_WIDTH;
            int y = (int)i.bbox.cy() * FRAME_HEIGHT_SONIFIED / COLOR_FRAME_HEIGHT;
            if(depth < THRESH_DST_SONIFY_NEAR)
                _obstacle.at<unsigned char>(y,x) =  255;
            else
                _obstacle.at<unsigned char>(y,x) =  255 * ( 1 - (depth - THRESH_DST_SONIFY_NEAR) / (THRESH_DST_SONIFY_DYNAMIC - THRESH_DST_SONIFY_NEAR));
        }
    }

    cv::max(_nearPixel, _obstacle, _obstacle);
    cv::imshow("Sonify", _nearPixel);
    cv::waitKey(1);
    lavSonifier::sonify(&_obstacle);
}

/*
 * Compute the segmentation to extract navigation spaces
 */
void lavVideoProcessor::_computeSegmentation() {
    cv::Mat copyMat, rgbSeg;

    // Copy the color image in a local Mat
    _mutexImg.lock();
    _inputMatColor.copyTo(copyMat);
    _mutexImg.unlock();

    // Perform the segmentation inference
    _mutexSeg.lock();
    _segSidewalk->doInference(copyMat);
    _segSidewalk->getOutput(_segImg);
    _mutexSeg.unlock();

    // Visualize the result
    cv::resize(copyMat, copyMat, cv::Size(_segImg.cols, _segImg.rows));
    cv::cvtColor(_segImg * 50, rgbSeg, cv::COLOR_GRAY2BGR);
    cv::addWeighted(copyMat, 0.7, rgbSeg, 0.3, 0, copyMat);
    cv::imshow("Segmentation", copyMat);
}

/*
 * Detect dynamic and static objects
 */
void lavVideoProcessor::_computeObjectDetection() {
    // Copy the color image in a local Mat
    cv::Mat copyMat;
    _mutexImg.lock();
    _inputMatColor.copyTo(copyMat);
    _mutexImg.unlock();

    // Perform the inference to detect objects on the color image
    _detectObstacle->doInference(copyMat);

    // Perform the tracking method
    _computeTracking();
}

/*
 * Compute tracking method
 */
void lavVideoProcessor::_computeTracking() {
    int index = 0;
    Eigen::Vector3d point_3d_t, point_3d_t_minus_1;
    Eigen::Quaterniond orientation_change = _orientation * _previousOrientation.inverse();
    std::map<unsigned int, std::vector<ObjectBoundingBox>> detection = _detectObstacle->getOutput();
    cv::Mat cpyMat;
    _inputMatColor.copyTo(cpyMat);
    float scaleX = (float)_inputMatColor.cols / (float)_inputMat.cols;
    float scaleY = (float)_inputMatColor.rows / (float)_inputMat.rows;

    for(int idxClass = 0; idxClass < NUM_CLASS; idxClass++) {
        bool trackerIdx = false;
        for(int t = 0; t < NUM_TRACKED_ID && !trackerIdx; t++)
        {
            if(idxClass == trackedID[t])
                trackerIdx = true;
        }

        if (detection.count(idxClass) > 0) {
            for (int idxObj = 0; idxObj < detection[idxClass].size();) {
                cv::Rect rect = detection[idxClass][idxObj].bbox.getRectCV();
                cv::rectangle(cpyMat, rect, cv::Scalar(0x27, 0xC1, 0x36), 2);
                unsigned short depth = 0xFFFF;
                int borneY = rect.height * scaleY;
                int borneX = rect.width * scaleX;
                for (int h = rect.y * scaleX; h < borneY; h++) {
                    for (int w = rect.x; w < borneX; w++) {
                        if(depth > _inputMat.at<unsigned short>(h, w) && _inputMat.at<unsigned short>(h, w) > 30)
                            depth =  _inputMat.at<unsigned short>(h, w);
                    }
                }
                detection[idxClass][idxObj].depth = depth;
            }

            if (trackerIdx) {
                std::vector<std::shared_ptr<ObjectTracking>> trackers = _track[index].update(detection[idxClass]);
                for (auto &track: trackers) {
                    float pixelCurrent[2] = {track->getRect().cx(), track->getRect().cy()};
                    lavCameraAcquisition::pixelToMeter(pixelCurrent, point_3d_t, track->getDepth());
                    float pixelPrevious[2] = {track->getRectPrev().cx(), track->getRectPrev().cy()};
                    lavCameraAcquisition::pixelToMeter(pixelPrevious, point_3d_t_minus_1, track->getPrevDepth());
                    Eigen::Vector3d point_3d_t_minus_1_corrected = orientation_change * point_3d_t_minus_1;
                    Eigen::Vector3d velocity = (point_3d_t - point_3d_t_minus_1_corrected) / 0.03;
                    if (_hazardMouvement(point_3d_t, velocity)) {
                        track->increaseCountHighMvt();
                        if (track->getCountHighMvt() > 100)
                            _dynamicObstacle.emplace_back(track->getObjectBoundingBox());
                        else
                            _staticObstacle.emplace_back(track->getObjectBoundingBox());
                    } else {
                        track->resetCountHighMvt();
                        _staticObstacle.emplace_back(track->getObjectBoundingBox());
                    }
                }
                index++;
            } else {
                for (auto &obj: detection[idxClass]) {
                    _staticObstacle.emplace_back(obj);
                }
            }
        }
        if(trackerIdx)
        {
            std::vector<ObjectBoundingBox> emptyVector = {};
            std::vector<std::shared_ptr<ObjectTracking>> trackers = _track[index].update(emptyVector);
        }
    }
}

/*
 * Set the path to record the frames
 */
void lavVideoProcessor::setSavePath(std::string path) {
    _mutexImg.lock();
    _savePath = std::move(path);
    _mutexImg.unlock();
}

/*
 * Save method
 */
void lavVideoProcessor::_saveImg() {
    lavCameraAcquisition::getNextFrame(_inputMat, _inputMatColor);
    auto time = std::chrono::system_clock::now();

    // get number of milliseconds for the current second
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch()) % 1000;

    // convert to std::time_t in order to convert to std::tm (broken time)
    auto timer = std::chrono::system_clock::to_time_t(time);

    // convert to broken time
    std::tm bt = *std::localtime(&timer);
    std::string strDate = std::to_string(bt.tm_hour) + "_" + std::to_string(bt.tm_min) + "_" + std::to_string(bt.tm_sec) + "_" + std::to_string(ms.count());
    cv::imwrite(_savePath + "/Color_" + strDate + ".png", _inputMatColor);
    cv::imwrite(_savePath + "/Depth_" + strDate + ".png", _inputMat);
}

bool lavVideoProcessor::_isPredominantlyGreen(const cv::Mat &img) {
    cv::Mat hsvImage;
    cv::cvtColor(img, hsvImage, cv::COLOR_BGR2HSV);

    cv::Mat redMask, greenMask;
    cv::inRange(hsvImage, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), redMask); // Threshold for red color in HSV
    cv::inRange(hsvImage, cv::Scalar(36, 70, 50), cv::Scalar(80, 255, 255), greenMask); // Threshold for green color in HSV

    cv::Mat valueChannel;
    cv::extractChannel(hsvImage, valueChannel, 2); // Extract the value channel (V) from HSV

    cv::Mat lowValueMask;
    cv::threshold(valueChannel, lowValueMask, 50, 255, cv::THRESH_BINARY); // Threshold for low value

    // Combine the masks
    redMask &= ~lowValueMask;
    greenMask &= ~lowValueMask;

    double redPixels = cv::countNonZero(redMask);
    double greenPixels = cv::countNonZero(greenMask);

    double redRatio = redPixels / (img.rows * img.cols);
    double greenRatio = greenPixels / (img.rows * img.cols);

    if (greenRatio > redRatio && greenRatio > 0.1) {
        return true;
    }
    return false;
}

void lavVideoProcessor::_computeTrafficLight() {
    _detectObstacle->doInference(_inputMatColor);
    std::map<unsigned int, std::vector<ObjectBoundingBox>> outDetection = _detectObstacle->getOutput();
    unsigned int numberOfPedestrianLight = outDetection.count(PEDESTRIAN_TRAFFIC_LIGHT_ID);
    char indexPTL = -1;
    if (numberOfPedestrianLight <= 0)
    {
        pedestrianLightStatus = -1;
        return;
    }

    unsigned short distanceFromUser = USHRT_MAX;
    for(int i = 0; i < numberOfPedestrianLight; i++)
    {
        cv::Mat roiImage = _inputMat(outDetection[PEDESTRIAN_TRAFFIC_LIGHT_ID][i].bbox.getRectCV());
        // Flatten the ROI image to a single row for filtering
        cv::Mat flatImage = roiImage.reshape(0, 1);
        // Filter out pixels with a value of 255
        cv::Mat filteredImage;
        cv::inRange(flatImage, 1000, 15000, filteredImage);
        // Convert the filtered image to a vector
        std::vector<unsigned short > pixels;
        if (filteredImage.isContinuous()) {
            pixels.assign(filteredImage.data, filteredImage.data + filteredImage.total());
        }
        // Sort the pixel values
        std::sort(pixels.begin(), pixels.end());
        // Calculate the median pixel value
        unsigned short  medianPixelValue = pixels[pixels.size() / 2];
        if(medianPixelValue < distanceFromUser)
        {
            distanceFromUser = medianPixelValue;
            indexPTL = i;
        }
    }
    if(indexPTL == -1)
    {
        pedestrianLightStatus = -1;
        return;
    }
    else
        indexPTL = 0;
    cv::Mat roiImage = _inputMatColor(outDetection[PEDESTRIAN_TRAFFIC_LIGHT_ID][indexPTL].bbox.getRectCV());
    bool value = _isPredominantlyGreen(roiImage);
    pedestrianLightStatus = value;
}

void lavVideoProcessor::_computeCrosswalk() {
    _computeSegmentation();
    _newVideoProcessingIsAvailable = true;
}

/*
 * Extract near element based on the depth map
 */
cv::Mat lavVideoProcessor::_computeNearPixel() {
    cv::Mat result = cv::Mat(cv::Size(_inputMat.cols, _inputMat.rows), CV_8U);
    for (int y = 0; y < _inputMat.rows; y++) {
        for (int x = 0; x < _inputMat.cols; x++) {
            unsigned short pixelValue = _inputMat.at<unsigned short>(y, x);
            // If the pixel position in between 30 cm and the THREAD limit
            if (pixelValue > 30 || pixelValue < THRESH_DST_SONIFY_NEAR) {
                result.at<unsigned char>(y, x) = 255;
            }
        }
    }
    return result;
}

/*
 * Correct the angular direction according to the segmentation map
 */
int lavVideoProcessor::getPathCorrection(int angle) {
    cv::Mat tempMat;
    _segImg.copyTo(tempMat);
    // Change the origin position of the azimuthal axis. From 0 == Center of img to O == Right side of the img
    int pixelElevation = _segImg.rows - 20;
    for (int y = tempMat.rows - 1; y >= 0; y--) {
        for (int x = 0; x < tempMat.cols; x++) {
            uchar pixelValue = tempMat.at<uchar>(y, x);
            if(y != tempMat.rows - 1)
            {
                if (tempMat.at<uchar>(y + 1, x) == 255 & (pixelValue != 0)) {
                    tempMat.at<uchar>(y, x) = 255;
                }
                else {
                    tempMat.at<uchar>(y, x) = 0;
                }
            }
            else
            {
                if (pixelValue !=0 ) {
                    tempMat.at<uchar>(y, x) = 255;
                }
                else {
                    tempMat.at<uchar>(y, x) = 0;
                }
            }
        }
    }
    // Create a kernel for erosion
    cv::Size kernelSize(7, 7);  // Adjust the kernel size as per your requirement
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, kernelSize);


    // Find contours in the eroded image
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(tempMat, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if(angle > - FOV_X / 2 and angle < FOV_X / 2)
    {
        int pixelPosition = RealsenseCamera::AngToPixel(angle + FOV_X / 2, FOV_X, tempMat.cols);
        // Extract the centroid of each contour
        std::vector<cv::Point> centroids;
        float distance = FLT_MAX;
        int correctedPosition = -1;
        for (const auto& contour : contours) {
            cv::Moments M = cv::moments(contour);
            if(M.m00 != 0)
            {
                int centroid_x = static_cast<int>(M.m10 / M.m00);
                int centroid_y = static_cast<int>(M.m01 / M.m00);
                centroids.emplace_back(centroid_x, centroid_y);
                auto dist_moments = (float)std::sqrt((centroid_x - pixelPosition) * (centroid_x - pixelPosition) + (centroid_y - pixelElevation) * (centroid_y - pixelElevation));
                if(dist_moments < distance)
                {
                    correctedPosition = centroid_x;
                    distance = dist_moments;
                }
            }
            else
                continue;
        }
        if(correctedPosition != -1)
        {
            int fovX = lavCameraAcquisition::getFov().width;
            cv::circle(tempMat, cv::Point(correctedPosition, pixelPosition), 10, cv::Scalar(0, 0, 255), -1); // Red color point with a radius of 2
            cv::imshow("correction", tempMat);
            // Change the origin position of the azimuthal axis. From O == Right side of the img to 0 == Center of img
            return VideoCapture::pixelToAng(correctedPosition, lavCameraAcquisition::getFov().width, tempMat.cols) - fovX / 2;
        }
    }
    return 180; // Impossible to rectify.

}

bool lavVideoProcessor::_hazardMouvement(const Eigen::Vector3d & position, const Eigen::Vector3d & velocity) {
    // If low motion, return false
    if(velocity.norm() < 3)
    {
        return false;
    }
    // Calculer le vecteur directionnel de leur position vers l'origine
    Eigen::Vector3d directionToOrigin = -position;

    // Normaliser les vecteurs
    directionToOrigin.normalize();
    Eigen::Vector3d normalizedVelocity = velocity.normalized();

    // Calculer le produit scalaire
    double dotProduct = directionToOrigin.dot(normalizedVelocity);

    // Calculer le seuil correspondant à une marge d'erreur de 40°
    double threshold = std::cos(40.0 * M_PI / 180.0);

    // Si le produit scalaire est supérieur au seuil, ils se dirigent vers l'origine
    return dotProduct > threshold;
}

bool lavVideoProcessor::newValueIsAvailable() {
    return _newVideoProcessingIsAvailable;
}

/*
 * Change the state of the process
 */
void lavVideoProcessor::changeStatus(VideoState state) {
    _mutexImg.lock();
    status = state;
    _newVideoProcessingIsAvailable = false;
    _mutexImg.unlock();
}


void *lavVideoProcessor::start_video_stream(void *args) {
    auto* thisPointer = (lavVideoProcessor*) args;
    _closeVideo = false;
    thisPointer->start();
    return nullptr;
}


void lavVideoProcessor::start_thread_video_stream() {
    //usleep(10000);
    pthread_t thread_video_processing;
    pthread_create(&thread_video_processing, nullptr, start_video_stream, (void *) nullptr);
}
