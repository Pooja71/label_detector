#include "label_detector/LabelDetector.h"

LabelDetector::LabelDetector():is_image_open(true)
{
    ros::NodeHandle pnh("~");
    if(!pnh.getParam("image_src", str_filename_))
        ROS_INFO("Filename could not be fetched");
 //   ROS_INFO("filename: %s",str_filename_.c_str());
 //   pub_= nh_.advertise<std_msgs::String>("labels",100);   
    ReadImage();
}

void LabelDetector::ReadImage()
{
    image_ip_ = imread(str_filename_);
    if (image_ip_.empty())
    {
        is_image_open = false;
        ROS_INFO("Cannot open image.");
    }
}

void LabelDetector::FindContours()
{
    vector<Vec4i> hierarchy;
    RNG rng(12345);
      findContours(image_op_, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    
    vector<Point> approx_curve;
    vector<Moments> mu(contours.size());        //finding image moments
    vector<Point2f> mc(contours.size());        //finding centroid of rectangles using image moments

    Mat final_drawing = Mat::zeros(image_op_.size(), CV_8UC3);  //Image showing the contours

    for (int i = 0; i< contours.size(); i++)
    {
        cv::approxPolyDP(cv::Mat(contours[i]), approx_curve, cv::arcLength(cv::Mat(contours[i]), true)*0.04, true);
        if (std::fabs(cv::contourArea(contours[i])) < 500 )
            continue;
       int vertices = approx_curve.size();
        
        //Assuming only labels will have 4 vertices
       if (vertices == 4 )
       {
            for (int j = 0; j < approx_curve.size(); j++)
                cout << "Vertices : " << (int)approx_curve[j].x << ',' << (int)approx_curve[j].y << '\n';
            //get moments
            mu[i] = moments(contours[i], false);

ROS_INFO("Line50");
            //get mass centers
            mc[i] = Point2f((float)(mu[i].m10 / (mu[i].m00)) , (float)(mu[i].m01 / (mu[i].m00)));
            cv::Point2f location = Point2f(mc[i].x / image_ip_.cols, mc[i].y/image_ip_.rows);
ROS_INFO("Line54");
            Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            drawContours(final_drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
ROS_INFO("Line57");
            cout << "Contour Area : " << contourArea(contours[i]) << '\t' << "Contour Perimeter : " << arcLength(contours[i], true) << '\t' << "Location : " << location << '\n';
            string text = "Area-" + std::to_string(contourArea(contours[i])) + " Perimeter-" + std::to_string(arcLength(contours[i], true)) + '\n'; // +"Centroid-" + std::to_string(mc[i]);
            putText(final_drawing, text, mc[i], FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(200, 200, 250), 1, CV_AA);
       }
    }

    imshow("Windows", final_drawing);             
}

void LabelDetector::Detect()
{
    ros::Rate rate(100);
   // std_msgs::String pub_msg;
   // pub_msg.data= "Hello";
  //  pub_.publish(pub_msg);
    //rate.sleep();

    //blur the image
    GaussianBlur(image_ip_, image_op_, Size(5, 5), 3);
    cvtColor(image_op_, image_op_, CV_BGR2GRAY);
    erode(image_op_, image_op_, Mat(), Point(-1, -1), 2, 1, 1);
    namedWindow("SHARP", CV_WINDOW_AUTOSIZE);
    imshow("SHARP", image_op_);
    Canny(image_op_, image_op_, 840, 850, 5, true);
    Size size_kernel(5, 5);    
    Mat element = getStructuringElement(MORPH_RECT, size_kernel, Point(1, 1));
    morphologyEx(image_op_, image_op_, MORPH_CLOSE, element);
    namedWindow("Edges", CV_WINDOW_AUTOSIZE);
    imshow("Edges", image_op_);
    FindContours();
    waitKey();
}
