// main.cpp
// Indoor camera calibration using OpenCV.
// Loads MSc dataset and (eventually) runs cv::calibrateCamera.

#include <iostream>
#include <stdexcept>
#include <string>

#include "config.h"
#include "types.h"

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

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}