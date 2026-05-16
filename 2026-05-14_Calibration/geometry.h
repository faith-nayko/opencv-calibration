// geometry.h
// Geometric conversions between FEMBUN and OpenCV conventions

#pragma once

#include <opencv2/core.hpp>

#include "types.h"

namespace geometry {

	// Convert a FEMBUN EOP to an OpenCV pose.
	//
	// Inputs:
	//   eop - exterior orientation in FEMBUN convention
	//         (position in mm; angles in degrees;
	//          rotation world-to-camera;
	//          rotation order R = R_kappa * R_phi * R_omega)
	//
	// Outputs:
	//   rvec - 3x1 Rodrigues rotation vector
	//   tvec - 3x1 translation vector (mm)
	void eopToOpenCVPose(const EOP& eop, cv::Mat& rvec, cv::Mat& tvec);

}  // namespace geometry