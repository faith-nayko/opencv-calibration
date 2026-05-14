#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
	std::cout << "OpenCV version: " << CV_VERSION << std::endl;
	cv::Mat image = cv::Mat::zeros(100, 100, CV_8UC3);
	std::cout << "Created a " << image.rows << "x" << image.cols << " image." << std::endl;
	return 0;
}