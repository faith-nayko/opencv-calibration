#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <sstream>
#include <iomanip>
#include <fstream>

// ============================================================================
// Configuration
// ============================================================================

const cv::Size PATTERN_SIZE(9, 6);           // 9 x 6 internal corners
const float SQUARE_SIZE = 1.0f;              // Placeholder unit
const int NUM_IMAGES = 14;
const std::string IMAGE_DIR = "C:/opencv/sources/samples/data/";
const std::string OUTPUT_FILE = "calibration_results.txt";

// ============================================================================
// Helper functions
// ============================================================================

// Build the 3D coordinates of the checkerboard corners (same for every image).
// Origin is at one corner, X along columns, Y along rows, Z=0 (the board is flat).
std::vector<cv::Point3f> buildObjectPointsTemplate(cv::Size patternSize, float squareSize) {
    std::vector<cv::Point3f> points;
    for (int row = 0; row < patternSize.height; row++) {
        for (int col = 0; col < patternSize.width; col++) {
            points.push_back(cv::Point3f(col * squareSize, row * squareSize, 0.0f));
        }
    }
    return points;
}

// Walk through the calibration images, detect corners, and collect them.
// Returns the number of images that were successfully processed.
int collectImagePoints(
    const std::string& imageDir,
    cv::Size patternSize,
    int numImages,
    const std::vector<cv::Point3f>& objectPointsTemplate,
    std::vector<std::vector<cv::Point2f>>& imagePoints,
    std::vector<std::vector<cv::Point3f>>& objectPoints,
    cv::Size& imageSize)
{
    cv::TermCriteria subPixCriteria(
        cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);

    int successCount = 0;

    for (int i = 1; i <= numImages; i++) {
        std::stringstream filename;
        filename << imageDir << "left"
            << std::setw(2) << std::setfill('0') << i << ".jpg";

        cv::Mat image = cv::imread(filename.str());
        if (image.empty()) {
            std::cerr << "  Image " << std::setw(2) << i << ": file not found" << std::endl;
            continue;
        }

        // Record image size from the first successfully loaded image
        if (imageSize.width == 0) {
            imageSize = image.size();
        }

        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, patternSize, corners);

        if (!found) {
            std::cout << "  Image " << std::setw(2) << i << ": corners not detected" << std::endl;
            continue;
        }

        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), subPixCriteria);

        imagePoints.push_back(corners);
        objectPoints.push_back(objectPointsTemplate);
        successCount++;

        std::cout << "  Image " << std::setw(2) << i << ": " << corners.size() << " corners found" << std::endl;
    }

    return successCount;
}

// ============================================================================
// Main
// ============================================================================

int main() {
    std::cout << "Camera Calibration via OpenCV\n";
    std::cout << "=============================\n\n";

    // Build the 3D coordinates of the checkerboard corners (same for every image)
    std::vector<cv::Point3f> objectPointsTemplate =
        buildObjectPointsTemplate(PATTERN_SIZE, SQUARE_SIZE);

    // Collect detected corners across all images
    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints;
    cv::Size imageSize;

    std::cout << "Processing calibration images:\n";
    int successCount = collectImagePoints(
        IMAGE_DIR, PATTERN_SIZE, NUM_IMAGES,
        objectPointsTemplate, imagePoints, objectPoints, imageSize);

    std::cout << "\nSuccessfully processed " << successCount
        << " of " << NUM_IMAGES << " images.\n";
    std::cout << "Image size: " << imageSize.width << " x " << imageSize.height << " pixels\n\n";

    if (successCount < 3) {
        std::cerr << "Not enough images for calibration. Aborting.\n";
        return 1;
    }

    // Run the calibration
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;

    double rmsError = cv::calibrateCamera(
        objectPoints, imagePoints, imageSize,
        cameraMatrix, distCoeffs, rvecs, tvecs);

    // Print results
    std::cout << "Calibration Results\n";
    std::cout << "===================\n";
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "RMS reprojection error: " << rmsError << " pixels\n\n";

    double fx = cameraMatrix.at<double>(0, 0);
    double fy = cameraMatrix.at<double>(1, 1);
    double cx = cameraMatrix.at<double>(0, 2);
    double cy = cameraMatrix.at<double>(1, 2);

    std::cout << "Intrinsic parameters:\n";
    std::cout << "  Focal length (x):  " << fx << " pixels\n";
    std::cout << "  Focal length (y):  " << fy << " pixels\n";
    std::cout << "  Principal point:   (" << cx << ", " << cy << ")\n\n";

    std::cout << "Distortion coefficients:\n";
    std::cout << "  k1: " << distCoeffs.at<double>(0) << "\n";
    std::cout << "  k2: " << distCoeffs.at<double>(1) << "\n";
    std::cout << "  p1: " << distCoeffs.at<double>(2) << "\n";
    std::cout << "  p2: " << distCoeffs.at<double>(3) << "\n";
    std::cout << "  k3: " << distCoeffs.at<double>(4) << "\n\n";

    // Save results to a plain text file
    std::ofstream outFile("calibration_results.txt");
    if (outFile.is_open()) {
        outFile << std::fixed << std::setprecision(4);
        outFile << "Camera Calibration Results\n";
        outFile << "==========================\n\n";
        outFile << "Image size: " << imageSize.width << " x " << imageSize.height << " pixels\n";
        outFile << "Number of images used: " << successCount << "\n";
        outFile << "RMS reprojection error: " << rmsError << " pixels\n\n";

        outFile << "Intrinsic parameters:\n";
        outFile << "  Focal length (x): " << fx << " pixels\n";
        outFile << "  Focal length (y): " << fy << " pixels\n";
        outFile << "  Principal point:  (" << cx << ", " << cy << ")\n\n";

        outFile << "Distortion coefficients:\n";
        outFile << "  k1: " << distCoeffs.at<double>(0) << "\n";
        outFile << "  k2: " << distCoeffs.at<double>(1) << "\n";
        outFile << "  p1: " << distCoeffs.at<double>(2) << "\n";
        outFile << "  p2: " << distCoeffs.at<double>(3) << "\n";
        outFile << "  k3: " << distCoeffs.at<double>(4) << "\n";

        outFile.close();
        std::cout << "Results written to: calibration_results.txt\n";
    }
    else {
        std::cerr << "Failed to open output file.\n";
    }

    return 0;
}