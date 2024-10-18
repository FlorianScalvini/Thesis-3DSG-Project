//
// Created by ubuntu on 04/07/23.
//

#ifndef OUTDOORNAV_KALMANFILTER_H
#define OUTDOORNAV_KALMANFILTER_H

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>



class KalmanFilter {
public:

    KalmanFilter(const float& std_weight_position = 1. / 20,
                 const float& std_weight_velocity = 1. / 160);

    void initiate(Eigen::Matrix<float, 1, 8, Eigen::RowMajor>& mean, Eigen::Matrix<float, 8, 8, Eigen::RowMajor>& covariance, const Eigen::Matrix<float, 1, 4, Eigen::RowMajor>& measurement);

    void predict(Eigen::Matrix<float, 1, 8, Eigen::RowMajor>& mean, Eigen::Matrix<float, 8, 8, Eigen::RowMajor>& covariance);

    void update(Eigen::Matrix<float, 1, 8, Eigen::RowMajor>& mean, Eigen::Matrix<float, 8, 8, Eigen::RowMajor>& covariance, const Eigen::Matrix<float, 1, 4, Eigen::RowMajor>& measurement);

private:
    float std_weight_position_;
    float std_weight_velocity_;

    Eigen::Matrix<float, 8, 8, Eigen::RowMajor> motion_mat_;
    Eigen::Matrix<float, 4, 8, Eigen::RowMajor> update_mat_;
};


#endif //OUTDOORNAV_KALMANFILTER_H
