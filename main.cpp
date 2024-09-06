#include <iostream>
#include <chrono> 
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

int main(int argc, char *argv[])
{
    cv::Mat inputImg, outputImg, outputImg8UC1;
    double min_val, max_val, ellapsedus, gamma = 0.6;
    inputImg = cv::imread(argv[1], -1);
    outputImg = cv::Mat::zeros(inputImg.rows, inputImg.cols, CV_32FC1);

    if(argc >= 3) gamma = std::stod(argv[2]);

    auto begin = std::chrono::high_resolution_clock::now();
    cv::minMaxIdx(inputImg, &min_val, &max_val);
    for(size_t i = 0; i < inputImg.rows; i++)
    {
        for(size_t j = 0; j < inputImg.cols; j++)
        {
            outputImg.at<float>(cv::Point2d(i, j)) = std::min<float>(std::powf(static_cast<float>(inputImg.at<uint16_t>(cv::Point2d(i, j))) / static_cast<float>(static_cast<uint16_t>(max_val) >> 3), gamma), 1);
        }
    }
    outputImg.convertTo(outputImg8UC1, CV_8UC1, 255.0);
    auto end = std::chrono::high_resolution_clock::now();
    ellapsedus = std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count();

    std::cout << "Processing time :" << ellapsedus << "us (" << 1 / (ellapsedus * 1e-6) << " FPS)\n";
    
    cv::namedWindow("AKIRS", cv::WINDOW_GUI_NORMAL);
    cv::imshow("AKIRS", outputImg8UC1);
    cv::waitKey(0);
    if(argc >= 4) cv::imwrite(argv[3], outputImg8UC1);
    return 0;
}