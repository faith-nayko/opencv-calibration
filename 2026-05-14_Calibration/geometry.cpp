// geometry.cpp
// Implementation of FEMBUN-to-OpenCV pose conversion.

#include "geometry.h"

#include <cmath>

#include <opencv2/calib3d.hpp>

namespace {

    // Convert degrees to radians.
    double deg2rad(double degrees) {
        constexpr double pi = 3.141592653589793238462643383279502884;
        return degrees * pi / 180.0;
    }

    // Build the FEMBUN rotation matrix from omega, phi, kappa (in degrees).
    // Convention:
    //   R = R_kappa * R_phi * R_omega
    //   Result rotates a vector from the world frame to the camera frame
    cv::Mat fembunRotationMatrix(double omega_deg, double phi_deg, double kappa_deg) {
        double omega = deg2rad(omega_deg);
        double phi = deg2rad(phi_deg);
        double kappa = deg2rad(kappa_deg);

        cv::Mat R_omega = (cv::Mat_<double>(3, 3) <<
            1.0, 0.0, 0.0,
            0.0, std::cos(omega), -std::sin(omega),
            0.0, std::sin(omega), std::cos(omega));

        cv::Mat R_phi = (cv::Mat_<double>(3, 3) <<
            std::cos(phi), 0.0, std::sin(phi),
            0.0, 1.0, 0.0,
            -std::sin(phi), 0.0, std::cos(phi));

        cv::Mat R_kappa = (cv::Mat_<double>(3, 3) <<
            std::cos(kappa), -std::sin(kappa), 0.0,
            std::sin(kappa), std::cos(kappa), 0.0,
            0.0, 0.0, 1.0);

        return R_kappa * R_phi * R_omega;
    }

}  // anonymous namespace

namespace geometry {

    void eopToOpenCVPose(const EOP& eop, cv::Mat& rvec, cv::Mat& tvec) {
        // FEMBUN rotation: world frame -> camera frame (matches OpenCV directly)
        cv::Mat R_opencv = fembunRotationMatrix(eop.omega_deg, eop.phi_deg, eop.kappa_deg);

        // Translation: position of world origin in camera frame
        cv::Mat C = (cv::Mat_<double>(3, 1) << eop.X, eop.Y, eop.Z);
        tvec = -R_opencv * C;

        // Convert rotation matrix to Rodrigues vector
        cv::Rodrigues(R_opencv, rvec);
    }

}  // namespace geometry