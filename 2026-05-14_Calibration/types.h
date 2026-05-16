// types.h
// Data structures for calibration input/output

#pragma once

#include <string>
#include <vector>

// Interior Orientation Parameters (initial estimates or calibration results).
// Values in pixels. Matches OpenCV's instrinsic + distortion model
struct IOPs {

	// Focal length
	double fx = 0.0;
	double fy = 0.0;

	// Principal point
	double cx = 0.0;
	double cy = 0.0;

	// Radial distortion
	double k1 = 0.0;
	double k2 = 0.0;
	double k3 = 0.0;
	double k4 = 0.0;
	double k5 = 0.0;
	double k6 = 0.0;

	// Tangential (decentering) distortion
	double p1 = 0.0;
	double p2 = 0.0;

	// Thin prism distortion
	double s1 = 0.0;
	double s2 = 0.0;
	double s3 = 0.0;
	double s4 = 0.0;

	// Tilted sensor model
	double taux = 0.0;
	double tauy = 0.0;

};

// Additional parameters configuration
// Boolean flags controlling which OpenCV CALIB_* flags are set.
struct APFlags {

	// Distortion model selection;
	bool use_rational_model = false;
	bool use_thin_prism_model = false;
	bool use_tilted_model = false;

	// Fixing instrinstics
	bool fix_aspect_ratio = false;
	bool fix_focal_length = false;
	bool fix_principal_point = false;

	// Fixing distortion coefficients
	bool fix_k1 = false;
	bool fix_k2 = false;
	bool fix_k3 = false;
	bool zero_tangent_dist = false;

	// Initial guess behaviour
	bool use_intrinsic_guess = false;

};

// General bundle adjustment configuration
struct BundleConfig {

	int image_width = 0;
	int image_height = 0;
	int max_iterations = 30;
	double epsilon = 1e-8;
	bool verbose = true;
	bool print_summary = true;

};

// One image's exterior orientation in FEMBUN convention
// Position in mm; angles in degrees
// Rotation: camera-to-world, sensor frame right-up-back
// Rotation order: R = R_kappa * R_phi * R_omega
struct EOP {

	std::string image_name;
	double X = 0.0;
	double Y = 0.0;
	double Z = 0.0;
	double omega_deg = 0.0;
	double phi_deg = 0.0;
	double kappa_deg = 0.0;

};

// One ground control point in world frame
// Coordinates in mm
struct ControlPoint {

	int point_id = 0;
	double X = 0.0;
	double Y = 0.0;
	double Z = 0.0;

};

// One image point observation
// Pixel coordinates with origin at top peft, x rightward, y downward
struct ImageObservation {

	int point_id  = 0;
	std::string image_name;
	double x = 0.0;
	double y = 0.0;

};