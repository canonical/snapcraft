#include <iostream>

#include <opencv2/core/core.hpp>

int main()
{
    cv::Mat matrix(2, 2, CV_8UC1);

    matrix.at<uint8_t>(cv::Point(0, 0)) = 1;
    matrix.at<uint8_t>(cv::Point(0, 1)) = 2;
    matrix.at<uint8_t>(cv::Point(1, 0)) = 3;
    matrix.at<uint8_t>(cv::Point(1, 1)) = 4;

    std::cout << matrix << std::endl;
}
