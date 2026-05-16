// main.cpp
// Indoor camera calibration using OpenCV.
// Loads MSc dataset and (eventually) runs cv::calibrateCamera.

#include <iostream>
#include <stdexcept>
#include <string>
#include <iomanip>

#include <opencv2/calib3d.hpp>

#include "config.h"
#include "types.h"
#include "geometry.h"
#include "calibration.h"



int main() {
    try {
        // Load all input files
        const std::string data_dir = "data/";

        BundleConfig cfg = config::readBundleConfig(data_dir + "bundle_config.txt");
        APFlags flags = config::readAPFlags(data_dir + "additional_parameters.txt");
        IOPs iops = config::readIOPs(data_dir + "iops.txt");

        std::vector<EOP> eops = config::readEOPs(data_dir + "eops.txt");
        std::vector<ControlPoint> gcps = config::readControlPoints(data_dir + "control_points.txt");
        std::vector<ImageObservation> obs = config::readImageObservations(data_dir + "image_observations.txt");

        // Print summary
        std::cout << "=== Calibration inputs loaded ===\n";
        std::cout << "Image dimensions: " << cfg.image_width << " x " << cfg.image_height << "\n";
        std::cout << "Initial focal length: fx=" << iops.fx << ", fy=" << iops.fy << "\n";
        std::cout << "Initial principal point: cx=" << iops.cx << ", cy=" << iops.cy << "\n";
        std::cout << "Fix aspect ratio: " << (flags.fix_aspect_ratio ? "true" : "false") << "\n";
        std::cout << "Zero tangent dist: " << (flags.zero_tangent_dist ? "true" : "false") << "\n";
        std::cout << "Number of images: " << eops.size() << "\n";
        std::cout << "Number of control points: " << gcps.size() << "\n";
        std::cout << "Number of image observations: " << obs.size() << "\n";

        // Spot-check: print first row of each tabular file
        if (!eops.empty()) {
            const auto& e = eops.front();
            std::cout << "\nFirst EOP: " << e.image_name
                << " X=" << e.X << " Y=" << e.Y << " Z=" << e.Z
                << " omega=" << e.omega_deg << " phi=" << e.phi_deg
                << " kappa=" << e.kappa_deg << "\n";
        }
        if (!gcps.empty()) {
            const auto& g = gcps.front();
            std::cout << "First GCP: id=" << g.point_id
                << " X=" << g.X << " Y=" << g.Y << " Z=" << g.Z << "\n";
        }
        if (!obs.empty()) {
            const auto& o = obs.front();
            std::cout << "First observation: point_id=" << o.point_id
                << " image=" << o.image_name
                << " x=" << o.x << " y=" << o.y << "\n";
        }

        // Test FEMBUN -> OpenCV pose conversion
        if (!eops.empty()) {
            cv::Mat rvec, tvec;
            geometry::eopToOpenCVPose(eops.front(), rvec, tvec);

            std::cout << "\nFirst pose in OpenCV form:\n";
            std::cout << "rvec: " << rvec.t() << "\n";
            std::cout << "tvec: " << tvec.t() << "\n";
        }

        // Project a known GCP using the converted pose and compare to observation
        if (!eops.empty() && !gcps.empty() && !obs.empty()) {
            cv::Mat rvec, tvec;
            geometry::eopToOpenCVPose(eops.front(), rvec, tvec);

            // Build camera matrix from IOPs
            cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
                iops.fx, 0.0, iops.cx,
                0.0, iops.fy, iops.cy,
                0.0, 0.0, 1.0);

            // Distortion coefficients (zero for the initial estimate)
            cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);

            // Find an observation in the first image that we have a GCP for
            const std::string& target_image = eops.front().image_name;
            for (const auto& observation : obs) {
                if (observation.image_name != target_image) continue;

                // Find the matching GCP
                for (const auto& gcp : gcps) {
                    if (gcp.point_id != observation.point_id) continue;

                    // Project the GCP
                    std::vector<cv::Point3d> object_points = { cv::Point3d(gcp.X, gcp.Y, gcp.Z) };
                    std::vector<cv::Point2d> projected_points;
                    cv::projectPoints(object_points, rvec, tvec, cameraMatrix, distCoeffs, projected_points);

                    std::cout << "\nProjection check for image " << target_image
                        << ", point " << gcp.point_id << ":\n";
                    std::cout << "  Observed:  (" << observation.x << ", " << observation.y << ")\n";
                    std::cout << "  Projected: (" << projected_points[0].x << ", " << projected_points[0].y << ")\n";
                    std::cout << "  Residual:  (" << projected_points[0].x - observation.x
                        << ", " << projected_points[0].y - observation.y << ")\n";
                    break;  // just check the first match
                }
                break;  // just check the first observation
            }
        }

        // Run calibration
        auto result = calibration::runCalibration(iops, flags, cfg, eops, gcps, obs);

        // Print results
        std::cout << "\n=== Calibration Results ===\n";
        std::cout << "RMS reprojection error: " << result.rms_reprojection_error << " px\n";
        std::cout << "\nCalibrated IOPs:\n";
        std::cout << "  fx = " << result.calibrated_iops.fx << "\n";
        std::cout << "  fy = " << result.calibrated_iops.fy << "\n";
        std::cout << "  cx = " << result.calibrated_iops.cx << "\n";
        std::cout << "  cy = " << result.calibrated_iops.cy << "\n";
        std::cout << "  k1 = " << result.calibrated_iops.k1 << "\n";
        std::cout << "  k2 = " << result.calibrated_iops.k2 << "\n";
        std::cout << "  k3 = " << result.calibrated_iops.k3 << "\n";
        std::cout << "  p1 = " << result.calibrated_iops.p1 << "\n";
        std::cout << "  p2 = " << result.calibrated_iops.p2 << "\n";

        // Print comparison to FEMBUN reference values
        std::cout << "\n=== Comparison to FEMBUN Reference Calibration ===\n";
        std::cout << std::fixed;
        std::cout << "Parameter |  FEMBUN      |  OpenCV      |  Diff        |  Rel diff (%)\n";
        std::cout << "----------|--------------|--------------|--------------|---------------\n";

        auto printRow = [](const std::string& name, double fembun, double opencv) {
            double diff = opencv - fembun;
            double rel_diff = (fembun != 0.0) ? (diff / fembun) * 100.0 : 0.0;
            std::cout << "  " << std::left << std::setw(8) << name
                << "|  " << std::setw(12) << std::setprecision(4) << fembun
                << "|  " << std::setw(12) << std::setprecision(4) << opencv
                << "|  " << std::setw(12) << std::setprecision(4) << diff
                << "|  " << std::setprecision(2) << rel_diff << "\n";
            };

        const auto& r = result.calibrated_iops;
        printRow("fx", 3020.6737, r.fx);
        printRow("cx", 2030.7731, r.cx);
        printRow("cy", 1512.2849, r.cy);
        printRow("k1", 0.09771, r.k1);
        printRow("k2", -0.18865, r.k2);
        printRow("k3", 0.11160, r.k3);

        return 0;

    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}