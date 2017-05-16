#include "ros/ros.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/String.h"
#include <string>

using namespace std;
using namespace cv;

class LabelDetector
{
    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_;
        std::string str_filename_;
        cv::Mat image_op_;
        cv::Mat image_ip_;
    public:
        LabelDetector(const std::string &str_filename);
        bool is_image_open;
        vector<vector<Point> > contours;
        void ReadImage();
        void Detect();
        void FindContours();
};
