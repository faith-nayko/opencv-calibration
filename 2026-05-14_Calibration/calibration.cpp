// calibration.cpp
// Impementation of camera calibration using OpenCV.

#include "calibration.h"

#include <map>
#include <stdexcept>
#include <string>
#include <vector>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include "geometry.h"

namespace {

	// One image's matched 3D-2D correspondenses, ready for cv::calibrateCamera
	struct ImageCorrespondences {
		
		std::string image_name;
		std::vector<cv::Point3f> object_points; // 3D world coordinates
		std::vector<cv::Point2f> image_points; // 2D pixel coordinates

	};

	// Group observations by image and pair each observation with its matching GCP
	// Returns one ImageCorrespondences per image that has at least one observation
	// Throws if an observation references a GCP that doesn't exist
	std::vector<ImageCorrespondences> groupByImage(

		const std::vector<ControlPoint>& gcps,
		const std::vector<ImageObservation>& obs) {

		// Build a lookup table: point_id -> ControlPoint
		std::map<int, const ControlPoint*> gcp_by_id;
		for (const auto& gcp : gcps) {
			gcp_by_id[gcp.point_id] = &gcp;

		}

		// Group observations by image name
		std::map<std::string, ImageCorrespondences> by_image;
		for (const auto& observation : obs) {
            auto it = gcp_by_id.find(observation.point_id);
            if (it == gcp_by_id.end()) {
                continue;  // No GCP for this point_id; silently skip the observation.
            } 

			const ControlPoint& gcp = *(it->second);

			ImageCorrespondences& ic = by_image[observation.image_name];
			ic.image_name = observation.image_name;
			ic.object_points.emplace_back(
				static_cast<float>(gcp.X),
				static_cast<float>(gcp.Y),
				static_cast<float>(gcp.Z));
			ic.image_points.emplace_back(
				static_cast<float>(observation.x),
				static_cast<float>(observation.y));
		}

		// Convert the map values into a vector for return
		std::vector<ImageCorrespondences> result;
		result.reserve(by_image.size());
		for (auto& [name, ic] : by_image) {
			result.push_back(std::move(ic));
		}
		return result;
	}

	// Translate APFlags struct into OpenCV's calibration flag bitmask.
	int buildCalibrationFlags(const APFlags& flags) {
		int result = 0;

		// Distortion model selection
		if (flags.use_rational_model) result |= cv::CALIB_RATIONAL_MODEL;
		if (flags.use_thin_prism_model) result |= cv::CALIB_THIN_PRISM_MODEL;
		if (flags.use_tilted_model) result |= cv::CALIB_TILTED_MODEL;

		// Fixing intrinsics
		if (flags.fix_aspect_ratio) result |= cv::CALIB_FIX_ASPECT_RATIO;
		if (flags.fix_focal_length) result |= cv::CALIB_FIX_FOCAL_LENGTH;
		if (flags.fix_principal_point) result |= cv::CALIB_FIX_PRINCIPAL_POINT;

		// Fixing distortion coefficients
		if (flags.fix_k1) result |= cv::CALIB_FIX_K1;
		if (flags.fix_k2) result |= cv::CALIB_FIX_K2;
		if (flags.fix_k3) result |= cv::CALIB_FIX_K3;
		if (flags.zero_tangent_dist) result |= cv::CALIB_ZERO_TANGENT_DIST;

		// Initial guess behaviour
		if (flags.use_intrinsic_guess) result |= cv::CALIB_USE_INTRINSIC_GUESS;

		return result;
	}

	// Extract omega, phi, kappa (in degrees) from an OpenCV rotation matrix.
// Inverse of geometry::fembunRotationMatrix.
// Assumes the matrix is world-to-camera in FEMBUN convention:
//   R = R_kappa * R_phi * R_omega
	void extractFembunAngles(const cv::Mat& R, double& omega_deg, double& phi_deg, double& kappa_deg) {
		constexpr double pi = 3.141592653589793238462643383279502884;
		const double rad_to_deg = 180.0 / pi;

		// From the structure of R = R_kappa * R_phi * R_omega, the elements work out to:
		//   R(0,2) =  sin(phi)
		//   R(1,2) = -cos(phi) * sin(omega)
		//   R(2,2) =  cos(phi) * cos(omega)
		//   R(0,1) = -sin(kappa) * cos(phi)
		//   R(0,0) =  cos(kappa) * cos(phi)

		double phi = std::asin(R.at<double>(0, 2));
		double omega = std::atan2(-R.at<double>(1, 2), R.at<double>(2, 2));
		double kappa = std::atan2(-R.at<double>(0, 1), R.at<double>(0, 0));

		omega_deg = omega * rad_to_deg;
		phi_deg = phi * rad_to_deg;
		kappa_deg = kappa * rad_to_deg;
	}

	// Convert an OpenCV pose (rvec, tvec) back to a FEMBUN EOP.
	// Inverse of geometry::eopToOpenCVPose.
	EOP openCVPoseToEop(const std::string& image_name, const cv::Mat& rvec, const cv::Mat& tvec) {
		EOP eop;
		eop.image_name = image_name;

		// rvec -> rotation matrix
		cv::Mat R;
		cv::Rodrigues(rvec, R);

		// Extract angles
		extractFembunAngles(R, eop.omega_deg, eop.phi_deg, eop.kappa_deg);

		// Recover camera position from tvec.
		// From eopToOpenCVPose: tvec = -R * C, where C is the camera position.
		// So: C = -R.t() * tvec   (equivalently, -R.inv() * tvec for a rotation matrix)
		cv::Mat C = -R.t() * tvec;
		eop.X = C.at<double>(0, 0);
		eop.Y = C.at<double>(1, 0);
		eop.Z = C.at<double>(2, 0);

		return eop;
	}

} // anonymous namespace

namespace calibration {

    CalibrationResult runCalibration(
        const IOPs& initial_iops,
        const APFlags& flags,
        const BundleConfig& cfg,
        const std::vector<EOP>& eops,
        const std::vector<ControlPoint>& gcps,
        const std::vector<ImageObservation>& obs) {

        // === Step 1: Group observations by image ===
        auto correspondences = groupByImage(gcps, obs);
        if (correspondences.empty()) {
            throw std::runtime_error("No image correspondences to calibrate against");
        }

        // OpenCV wants separate vectors of object points and image points,
        // ordered identically. Build those from the correspondences.
        std::vector<std::vector<cv::Point3f>> object_points;
        std::vector<std::vector<cv::Point2f>> image_points;
        std::vector<std::string> image_names;
        object_points.reserve(correspondences.size());
        image_points.reserve(correspondences.size());
        image_names.reserve(correspondences.size());
        for (const auto& ic : correspondences) {
            object_points.push_back(ic.object_points);
            image_points.push_back(ic.image_points);
            image_names.push_back(ic.image_name);
        }

        // === Step 2: Build camera matrix and distortion coefficients ===
        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
            initial_iops.fx, 0.0, initial_iops.cx,
            0.0, initial_iops.fy, initial_iops.cy,
            0.0, 0.0, 1.0);

        // Distortion vector layout depends on which model flags are set.
        // Standard 5-coefficient model: (k1, k2, p1, p2, k3).
        // Rational model adds: k4, k5, k6 (total 8).
        // Thin prism adds: s1, s2, s3, s4 (total 12).
        // Tilted adds: taux, tauy (total 14).
        int n_dist_coeffs = 5;
        if (flags.use_rational_model) n_dist_coeffs = 8;
        if (flags.use_thin_prism_model) n_dist_coeffs = 12;
        if (flags.use_tilted_model) n_dist_coeffs = 14;

        cv::Mat distCoeffs = cv::Mat::zeros(n_dist_coeffs, 1, CV_64F);
        distCoeffs.at<double>(0, 0) = initial_iops.k1;
        distCoeffs.at<double>(1, 0) = initial_iops.k2;
        distCoeffs.at<double>(2, 0) = initial_iops.p1;
        distCoeffs.at<double>(3, 0) = initial_iops.p2;
        distCoeffs.at<double>(4, 0) = initial_iops.k3;
        if (n_dist_coeffs >= 8) {
            distCoeffs.at<double>(5, 0) = initial_iops.k4;
            distCoeffs.at<double>(6, 0) = initial_iops.k5;
            distCoeffs.at<double>(7, 0) = initial_iops.k6;
        }
        if (n_dist_coeffs >= 12) {
            distCoeffs.at<double>(8, 0) = initial_iops.s1;
            distCoeffs.at<double>(9, 0) = initial_iops.s2;
            distCoeffs.at<double>(10, 0) = initial_iops.s3;
            distCoeffs.at<double>(11, 0) = initial_iops.s4;
        }
        if (n_dist_coeffs >= 14) {
            distCoeffs.at<double>(12, 0) = initial_iops.taux;
            distCoeffs.at<double>(13, 0) = initial_iops.tauy;
        }

        // === Step 3: Build initial rvecs and tvecs from input EOPs ===
        // Order must match the order of correspondences (i.e., image_names).
        std::map<std::string, const EOP*> eop_by_image;
        for (const auto& eop : eops) {
            eop_by_image[eop.image_name] = &eop;
        }

        std::vector<cv::Mat> rvecs;
        std::vector<cv::Mat> tvecs;
        rvecs.reserve(image_names.size());
        tvecs.reserve(image_names.size());
        for (const auto& name : image_names) {
            auto it = eop_by_image.find(name);
            if (it == eop_by_image.end()) {
                throw std::runtime_error("No EOP provided for image: " + name);
            }
            cv::Mat rvec, tvec;
            geometry::eopToOpenCVPose(*(it->second), rvec, tvec);
            rvecs.push_back(rvec);
            tvecs.push_back(tvec);
        }

        // === Step 4: Build flag bitmask and termination criteria ===
        int cv_flags = buildCalibrationFlags(flags);
        cv::TermCriteria criteria(
            cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
            cfg.max_iterations,
            cfg.epsilon);

        cv::Size image_size(cfg.image_width, cfg.image_height);

        // === Step 5: Run calibration ===
        if (cfg.verbose) {
            std::cout << "Running calibration with " << object_points.size() << " images...\n";
        }

        double rms = cv::calibrateCamera(
            object_points,
            image_points,
            image_size,
            cameraMatrix,
            distCoeffs,
            rvecs,
            tvecs,
            cv_flags,
            criteria);

        if (cfg.verbose) {
            std::cout << "Calibration finished. RMS reprojection error: " << rms << " px\n";
        }

        // === Step 6: Extract refined IOPs from output matrices ===
        IOPs refined;
        refined.fx = cameraMatrix.at<double>(0, 0);
        refined.fy = cameraMatrix.at<double>(1, 1);
        refined.cx = cameraMatrix.at<double>(0, 2);
        refined.cy = cameraMatrix.at<double>(1, 2);
        refined.k1 = distCoeffs.at<double>(0, 0);
        refined.k2 = distCoeffs.at<double>(1, 0);
        refined.p1 = distCoeffs.at<double>(2, 0);
        refined.p2 = distCoeffs.at<double>(3, 0);
        refined.k3 = distCoeffs.at<double>(4, 0);
        if (n_dist_coeffs >= 8) {
            refined.k4 = distCoeffs.at<double>(5, 0);
            refined.k5 = distCoeffs.at<double>(6, 0);
            refined.k6 = distCoeffs.at<double>(7, 0);
        }
        if (n_dist_coeffs >= 12) {
            refined.s1 = distCoeffs.at<double>(8, 0);
            refined.s2 = distCoeffs.at<double>(9, 0);
            refined.s3 = distCoeffs.at<double>(10, 0);
            refined.s4 = distCoeffs.at<double>(11, 0);
        }
        if (n_dist_coeffs >= 14) {
            refined.taux = distCoeffs.at<double>(12, 0);
            refined.tauy = distCoeffs.at<double>(13, 0);
        }

        // === Step 7: Convert refined rvecs/tvecs back to FEMBUN EOPs ===
        std::vector<EOP> refined_eops;
        refined_eops.reserve(image_names.size());
        for (size_t i = 0; i < image_names.size(); ++i) {
            refined_eops.push_back(openCVPoseToEop(image_names[i], rvecs[i], tvecs[i]));
        }

        // === Step 8: Assemble result ===
        CalibrationResult result;
        result.calibrated_iops = refined;
        result.refined_eops = refined_eops;
        result.rms_reprojection_error = rms;
        return result;
    }

} // namespace calibration