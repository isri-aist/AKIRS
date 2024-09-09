#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

class IRImageProcessor
{
public:
    IRImageProcessor(ros::NodeHandle& nh)
    {
        // Subscribe to the 16-bit IR image topic
        sub_ = nh.subscribe("/ir/image_raw", 1, &IRImageProcessor::imageCallback, this);

        // Publish the processed 8-bit image topic
        pub_ = nh.advertise<sensor_msgs::Image>("/camera/image_raw", 1);
    }

    // Callback function to process the incoming image
    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        // Convert ROS image message to OpenCV Mat (16-bit grayscale)
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat image_16bit = cv_ptr->image;

        // Apply image processing steps
        cv::Mat outputImg, outputImg8UC1;
        double min_val, max_val;
        double gamma = 0.6;

        // Initialize output image with float type
        outputImg = cv::Mat::zeros(image_16bit.rows, image_16bit.cols, CV_32FC1);

        // Get min and max values from the image
        cv::minMaxIdx(image_16bit, &min_val, &max_val);

        // Apply gamma correction and normalize the image
        for (int i = 0; i < image_16bit.rows; i++)
        {
            for (int j = 0; j < image_16bit.cols; j++)
            {
                outputImg.at<float>(cv::Point2d(i, j)) = std::min<float>(std::pow(static_cast<float>(image_16bit.at<uint16_t>(cv::Point2d(i, j))) / static_cast<float>(static_cast<uint16_t>(max_val) >> 3), gamma), 1);
            }
        }

        // Convert the float image to 8-bit for display and publishing
        outputImg.convertTo(outputImg8UC1, CV_8UC1, 255.0);

        // Convert processed 8-bit image to ROS message and publish
        sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", outputImg8UC1).toImageMsg();
        pub_.publish(output_msg);
    }

private:
    ros::Subscriber sub_;
    ros::Publisher pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ir_image_processor");
    ros::NodeHandle nh;

    IRImageProcessor processor(nh);

    ros::spin();
    return 0;
}
