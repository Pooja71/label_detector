#include "label_detector/LabelDetector.h"

int main(int argc, char** argv)
{
 //   std::string str_filename = "/home/pooja/catkin_git/src/label_detector/src/test.jpg";
    ros::init(argc,argv, "label_detector_node");
    LabelDetector ld;
    if(!ld.is_image_open)
        return -1;
    //while(ros::ok())
    {
    ld.Detect();
    }
    return 0;
}
