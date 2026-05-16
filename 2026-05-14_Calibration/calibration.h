// calibration.h
// Camera calibration using OpenCV
// Reads the MSc dataset, runs cv::calibrateCamera, returns refined parameters

#pragma once

#include <vector>
#include "types.h"

// Result of a calibration run
struct CalibrationResult {

	// Calibrated interior orientation parmaeters
	// (includes refined fx, fy, cx, cy, and all enabled distortion coefficients)
	IOPs calibrated_iops;

	// Refined exterior orientation parameters, one per input image,
	// converted back to FEMBUN convention (degrees, world-to-camera)
	std::vector<EOP> refined_eops;

	// Overall RMS reprojection error in pixels
	double rms_reprojection_error = 0.0;

};

namespace calibration {

	// Run camera calibration on the provided inputs
	//
	// Inputs:
	//	initial_iops	- starting estimates of camera instrinsics
	//	flags			- which OpenCV CALIB_* flags to set
	//	cfg				- general bundle adjustment configuration
	//  eops			- one initial EOP per image (FEMBUN convention)
	//  gcps			- ground control points (3D world coordinates)
	//  obs				- image point observations (links GCPs to images)
	//
	// Throws std::runtime_error if calibration fails for any reason
	CalibrationResult runCalibration(

		const IOPs& initial_iops,
		const APFlags& flags,
		const BundleConfig& cfg,
		const std::vector<EOP>& eops,
		const std::vector<ControlPoint>& gcps,
		const std::vector<ImageObservation>& obs);

} // namespace calibration
