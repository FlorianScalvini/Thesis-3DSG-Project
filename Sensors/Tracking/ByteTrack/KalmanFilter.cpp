//
// Created by ubuntu on 04/07/23.
//

#include "KalmanFilter.h"



#include <cstddef>

KalmanFilter::KalmanFilter(const float& std_weight_position,
                                       const float& std_weight_velocity) :
        std_weight_position_(std_weight_position),
        std_weight_velocity_(std_weight_velocity)
{
    constexpr int ndim = 4;
    constexpr float dt = 1;

    motion_mat_ = Eigen::MatrixXf::Identity(8, 8);
    update_mat_ = Eigen::MatrixXf::Identity(4, 8);

    for (int i = 0; i < ndim; i++)
    {
        motion_mat_(i, ndim + i) = dt;
    }
}


void KalmanFilter::initiate(Eigen::Matrix<float, 1, 8, Eigen::RowMajor> &mean, Eigen::Matrix<float, 8, 8, Eigen::RowMajor> &covariance, const Eigen::Matrix<float, 1, 4, Eigen::RowMajor> &measurement)
{
    mean.block<1, 4>(0, 0) = measurement.block<1, 4>(0, 0);
    mean.block<1, 4>(0, 4) = Eigen::Vector4f::Zero();

    Eigen::Matrix<float, 1, 8, Eigen::RowMajor> std;
    std(0) = 2 * std_weight_position_ * measurement[3];
    std(1) = 2 * std_weight_position_ * measurement[3];
    std(2) = 1e-2;
    std(3) = 2 * std_weight_position_ * measurement[3];
    std(4) = 10 * std_weight_velocity_ * measurement[3];
    std(5) = 10 * std_weight_velocity_ * measurement[3];
    std(6) = 1e-5;
    std(7) = 10 * std_weight_velocity_ * measurement[3];

    Eigen::Matrix<float, 1, 8, Eigen::RowMajor> tmp = std.array().square();
    covariance = tmp.asDiagonal();
}

void KalmanFilter::predict(Eigen::Matrix<float, 1, 8, Eigen::RowMajor> &mean, Eigen::Matrix<float, 8, 8, Eigen::RowMajor> &covariance)
{
    Eigen::Matrix<float, 1, 8, Eigen::RowMajor> std;
    std(0) = std_weight_position_ * mean(3);
    std(1) = std_weight_position_ * mean(3);
    std(2) = 1e-2;
    std(3) = std_weight_position_ * mean(3);
    std(4) = std_weight_velocity_ * mean(3);
    std(5) = std_weight_velocity_ * mean(3);
    std(6) = 1e-5;
    std(7) = std_weight_velocity_ * mean(3);

    Eigen::Matrix<float, 1, 8, Eigen::RowMajor> tmp = std.array().square();
    Eigen::Matrix<float, 8, 8, Eigen::RowMajor> motion_cov = tmp.asDiagonal();

    mean = motion_mat_ * mean.transpose();
    covariance = motion_mat_ * covariance * (motion_mat_.transpose()) + motion_cov;
}

void KalmanFilter::update(Eigen::Matrix<float, 1, 8, Eigen::RowMajor> &mean, Eigen::Matrix<float, 8, 8, Eigen::RowMajor> &covariance, const Eigen::Matrix<float, 1, 4, Eigen::RowMajor> &measurement)
{
    Eigen::Matrix<float, 1, 4, Eigen::RowMajor> projected_mean;
    Eigen::Matrix<float, 4, 4, Eigen::RowMajor> projected_cov;

    Eigen::Matrix<float, 1, 4, Eigen::RowMajor> std;
    std << std_weight_position_ * mean(3), std_weight_position_ * mean(3), 1e-1, std_weight_position_ * mean(3);

    projected_mean = update_mat_ * mean.transpose();
    projected_cov = update_mat_ * covariance * (update_mat_.transpose());

    Eigen::Matrix<float, 4, 4> diag = std.asDiagonal();
    projected_cov += diag.array().square().matrix();


    Eigen::Matrix<float, 4, 8> B = (covariance * (update_mat_.transpose())).transpose();
    Eigen::Matrix<float, 8, 4> kalman_gain = (projected_cov.llt().solve(B)).transpose();
    Eigen::Matrix<float, 1, 4> innovation = measurement - projected_mean;

    const auto tmp = innovation * (kalman_gain.transpose());
    mean = (mean.array() + tmp.array()).matrix();
    covariance = covariance - kalman_gain * projected_cov * (kalman_gain.transpose());
}
