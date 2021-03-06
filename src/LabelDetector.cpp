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

void drawAxis(Mat& img, Point p, Point q, Scalar colour, const float scale = 0.2)
{
    double angle;
    double hypotenuse;
    angle = atan2( (double) p.y - q.y, (double) p.x - q.x ); // angle in radians
    hypotenuse = sqrt( (double) (p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));
    //    double degrees = angle * 180 / CV_PI; // convert radians to degrees (0-180 range)
    //    cout << "Degrees: " << abs(degrees - 180) << endl; // angle in 0-360 degrees range
    // Here we lengthen the arrow by a factor of scale
    q.x = (int) (p.x - scale * hypotenuse * cos(angle));
    q.y = (int) (p.y - scale * hypotenuse * sin(angle));
    line(img, p, q, colour, 1, CV_AA);
    // create the arrow hooks
    p.x = (int) (q.x + 9 * cos(angle + CV_PI / 4));
    p.y = (int) (q.y + 9 * sin(angle + CV_PI / 4));
    line(img, p, q, colour, 1, CV_AA);
    p.x = (int) (q.x + 9 * cos(angle - CV_PI / 4));
    p.y = (int) (q.y + 9 * sin(angle - CV_PI / 4));
    line(img, p, q, colour, 1, CV_AA);
}


double getOrientation(const vector<Point> &pts, Mat &img)
{
    //Construct a buffer used by the pca analysis
    int sz = static_cast<int>(pts.size());
    Mat data_pts = Mat(sz, 2, CV_64FC1);
    for (int i = 0; i < data_pts.rows; ++i)
    {
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }
    //Perform PCA analysis
    PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);
    //Store the center of the object
    Point cntr = Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
    static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
    //Store the eigenvalues and eigenvectors
    vector<Point2d> eigen_vecs(2);
    vector<double> eigen_val(2);
    for (int i = 0; i < 2; ++i)
    {
        eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
        pca_analysis.eigenvectors.at<double>(i, 1));
        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
    }
    // Draw the principal components
    circle(img, cntr, 3, Scalar(255, 0, 255), 2);
    Point p1 = cntr + 0.02 * Point(static_cast<int>(eigen_vecs[0].x * eigen_val[0]), static_cast<int>(eigen_vecs[0].y * eigen_val[0]));
    Point p2 = cntr - 0.02 * Point(static_cast<int>(eigen_vecs[1].x * eigen_val[1]), static_cast<int>(eigen_vecs[1].y * eigen_val[1]));
    drawAxis(img, cntr, p1, Scalar(0, 255, 0), 1);
    drawAxis(img, cntr, p2, Scalar(255, 255, 0), 5);
    double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x); // orientation in radians
    return angle;
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

            //get mass centers
            mc[i] = Point2f((float)(mu[i].m10 / (mu[i].m00)) , (float)(mu[i].m01 / (mu[i].m00)));
            cv::Point2f location = Point2f(mc[i].x / image_ip_.cols, mc[i].y/image_ip_.rows);
            Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            //drawContours(final_drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
//            drawContours(image_ip_, contours, i, color, 2, 8, hierarchy, 0, Point());
              drawContours(image_ip_, contours, i, 255, CV_FILLED, 8, hierarchy, 0, Point());  
            cout << "Contour Area : " << contourArea(contours[i]) << '\t' << "Contour Perimeter : " << arcLength(contours[i], true) << '\t' << "Location : " << location << '\n';
            string text = "Area-" + std::to_string(contourArea(contours[i])) + " Perimeter-" + std::to_string(arcLength(contours[i], true)) + '\n'; // +"Centroid-" + std::to_string(mc[i]);
          //  putText(image_ip_, text, mc[i], FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(200, 200, 250), 1, CV_AA);
            // Find the orientation of each shape
               getOrientation(contours[i], image_ip_);
       }
    }
    namedWindow("With contour", CV_WINDOW_NORMAL);
    imshow("With Contour", image_ip_);

    //imshow("Windows", final_drawing);             
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
    namedWindow("SHARP", CV_WINDOW_NORMAL);
    imshow("SHARP", image_op_);
    Canny(image_op_, image_op_, 840, 850, 5, true);
    Size size_kernel(5, 5);    
    Mat element = getStructuringElement(MORPH_RECT, size_kernel, Point(1, 1));
    morphologyEx(image_op_, image_op_, MORPH_CLOSE, element);
    namedWindow("Edges", CV_WINDOW_NORMAL);
    imshow("Edges", image_op_);
    FindContours();
    waitKey();
}
