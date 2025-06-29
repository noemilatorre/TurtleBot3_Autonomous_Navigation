/* -------------------------------------------------------------------------- */
/*                                  INCLUDES                                  */
/* -------------------------------------------------------------------------- */
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>

/* -------------------------------------------------------------------------- */
/*                         GLOBAL VARIABLES                                   */
/* -------------------------------------------------------------------------- */

// ROSSO
int r_hmin = 22, r_smin = 70, r_vmin = 50;
int r_hmax = 46, r_smax = 255, r_vmax = 255;
int r2_hmin = 160, r2_hmax = 180;  // Per rosso doppio intervallo
int r2_smin = 70, r2_smax = 255;
int r2_vmin = 50, r2_vmax = 255;

// GIALLO
int y_hmin = 0, y_smin = 100, y_vmin = 100;
int y_hmax = 13, y_smax = 255, y_vmax = 255;

// VERDE
int g_hmin = 45, g_smin = 100, g_vmin = 100;
int g_hmax = 75, g_smax = 255, g_vmax = 255;


// Soglie
int avg_hue_min = 0;
int avg_hue_max = 360;
int illumination_threshold = 75;
int area_min = 110;
int area_max = 1050;

bool use_cosine_correction = false;
int morph_op = 0;           // 0: none, 1: open, 2: close, 3: erode, 4: dilate
int kernel_size = 5;        // Deve essere dispari
int apply_blur = 0;         // 0: no, 1: sì


/* -------------------------------------------------------------------------- */
/*                         FUNCTION PROTOTYPES                                */
/* -------------------------------------------------------------------------- */

void applyMorphOps(cv::Mat& mask) {
  int k = kernel_size;
  if (k % 2 == 0) k += 1; // Forza valore dispari
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));

  switch (morph_op) {
    case 1: cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel); break;
    case 2: cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel); break;
    case 3: cv::erode(mask, mask, kernel); break;
    case 4: cv::dilate(mask, mask, kernel); break;
    default: break;
  }
}

void nothing(int, void*) {}

double avgValInContour(const cv::Mat& img, const std::vector<cv::Point>& contour, bool cosine = false) {
  double sum = 0;
  int count = 0;
  cv::Rect bbox = cv::boundingRect(contour);

  for (int y = bbox.y; y < bbox.y + bbox.height; ++y) {
    for (int x = bbox.x; x < bbox.x + bbox.width; ++x) {
      if (cv::pointPolygonTest(contour, cv::Point(x, y), false) >= 0) {
        uchar val = img.at<uchar>(y, x);
        if (cosine)
          sum += std::acos(std::cos(val * 2.0 * CV_PI / 180.0)) * 180.0 / CV_PI / 2.0;
        else
          sum += val;
        ++count;
      }
    }
  }
  return (count > 0) ? (sum / count) : 0;
}

void processColor(const cv::Mat& hsv, const cv::Mat& frame, const cv::Scalar& lower, const cv::Scalar& upper, const std::string& label, cv::Mat& output) {
  cv::Mat mask;
  cv::inRange(hsv, lower, upper, mask);
  
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));

  applyMorphOps(mask);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

  std::vector<cv::Mat> hsv_channels;
  cv::split(hsv, hsv_channels);

  for (const auto& contour : contours) {
    double area = cv::contourArea(contour);
    if (area < area_min || area > area_max) continue;

             double perimeter = cv::arcLength(contour, true);
         double circularity = 4 * CV_PI * area / (perimeter * perimeter);

         ROS_INFO("circularity %f",circularity); //vedere circularity??
    if (circularity < 0.6) continue;  // scarta se non abbastanza rotondo (non si può alzare!)
         

    double illum = avgValInContour(hsv_channels[2], contour, false);
    if (illum < illumination_threshold) continue;

    double hue = 2* avgValInContour(hsv_channels[0], contour, true);
    if (hue < avg_hue_min || hue > avg_hue_max) continue;
    //AGGIUNTA STAMPA:
    ROS_INFO("%s: Avg Hue: %.2f, Avg V: %.2f", label.c_str(), hue, illum);

    cv::drawContours(output, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(255, 0, 0), 2);
    cv::Rect bbox = cv::boundingRect(contour);
    std::ostringstream text;
    text << label << " H:" << (int)hue << " V:" << (int)illum;
    cv::putText(output, text.str(), bbox.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
  }
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
  cv::Mat frame;
  try {
    frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat hsv;
  cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
  //APPLICO SU HSV TUTTO
  if (apply_blur == 1) {
    int k = (kernel_size % 2 == 0) ? kernel_size + 1 : kernel_size;
    cv::GaussianBlur(hsv, hsv, cv::Size(k, k), 0);
  }

  std::vector<cv::Mat> hsv_channels;
  cv::split(hsv, hsv_channels);
  cv::merge(hsv_channels, hsv);
  cv::Mat result = frame.clone();

  // Rosso ha doppia soglia
  processColor(hsv, frame, cv::Scalar(r_hmin, r_smin, r_vmin), cv::Scalar(r_hmax, r_smax, r_vmax), "Red", result);
  processColor(hsv, frame, cv::Scalar(r2_hmin, r_smin, r_vmin), cv::Scalar(r2_hmax, r_smax, r_vmax), "Red", result);

  processColor(hsv, frame, cv::Scalar(y_hmin, y_smin, y_vmin), cv::Scalar(y_hmax, y_smax, y_vmax), "Yellow", result);
  processColor(hsv, frame, cv::Scalar(g_hmin, g_smin, g_vmin), cv::Scalar(g_hmax, g_smax, g_vmax), "Green", result);

  cv::imshow("Original", frame);
  cv::imshow("Detection", result);
  cv::waitKey(1);
}

/* -------------------------------------------------------------------------- */
/*                                    MAIN                                    */
/* -------------------------------------------------------------------------- */


int main(int argc, char** argv) {
  ros::init(argc, argv, "multi_color_hsv_detector");
  ros::NodeHandle nh;

  cv::namedWindow("HSV Trackbars");
  cv::namedWindow("HSV Trackbars3");
  cv::namedWindow("HSV Trackbars2");

  // ROSSO
  cv::createTrackbar("R Hmin", "HSV Trackbars", &r_hmin, 180, nothing);
  cv::createTrackbar("R Hmax", "HSV Trackbars", &r_hmax, 180, nothing);

  cv::createTrackbar("R Smin", "HSV Trackbars", &r_smin, 255, nothing);
  cv::createTrackbar("R Smax", "HSV Trackbars", &r_smax, 255, nothing);
  cv::createTrackbar("R Vmin", "HSV Trackbars", &r_vmin, 255, nothing);
  cv::createTrackbar("R Vmax", "HSV Trackbars", &r_vmax, 255, nothing);

  cv::createTrackbar("R2 Hmin", "HSV Trackbars", &r2_hmin, 180, nothing);
  cv::createTrackbar("R2 Hmax", "HSV Trackbars", &r2_hmax, 180, nothing);
  cv::createTrackbar("R2 Smin", "HSV Trackbars", &r2_smin, 255, nothing);
  cv::createTrackbar("R2 Smax", "HSV Trackbars", &r2_smax, 255, nothing);
  cv::createTrackbar("R2 Vmin", "HSV Trackbars", &r2_vmin, 255, nothing);
  cv::createTrackbar("R2 Vmax", "HSV Trackbars", &r2_vmax, 255, nothing);

  // GIALLO
  cv::createTrackbar("Y Hmin", "HSV Trackbars3", &y_hmin, 180, nothing);
  cv::createTrackbar("Y Hmax", "HSV Trackbars3", &y_hmax, 180, nothing);
  cv::createTrackbar("Y Smin", "HSV Trackbars3", &y_smin, 255, nothing);
  cv::createTrackbar("Y Smax", "HSV Trackbars3", &y_smax, 255, nothing);
  cv::createTrackbar("Y Vmin", "HSV Trackbars3", &y_vmin, 255, nothing);
  cv::createTrackbar("Y Vmax", "HSV Trackbars3", &y_vmax, 255, nothing);

  // VERDE
  cv::createTrackbar("G Hmin", "HSV Trackbars2", &g_hmin, 180, nothing);
  cv::createTrackbar("G Hmax", "HSV Trackbars2", &g_hmax, 180, nothing);
  cv::createTrackbar("G Smin", "HSV Trackbars2", &g_smin, 255, nothing);
  cv::createTrackbar("G Smax", "HSV Trackbars2", &g_smax, 255, nothing);
  cv::createTrackbar("G Vmin", "HSV Trackbars2", &g_vmin, 255, nothing);
  cv::createTrackbar("G Vmax", "HSV Trackbars2", &g_vmax, 255, nothing);

  // Soglie
  cv::createTrackbar("Illum Thresh", "HSV Trackbars2", &illumination_threshold, 255, nothing);
  cv::createTrackbar("avgHue Min", "HSV Trackbars2", &avg_hue_min, 360, nothing);
  cv::createTrackbar("avgHue Max", "HSV Trackbars2", &avg_hue_max, 360, nothing);
  cv::createTrackbar("Area Min", "HSV Trackbars2", &area_min, 10000, nothing);
  cv::createTrackbar("Area Max", "HSV Trackbars2", &area_max, 20000, nothing);
  
//SELEZIONO SOLO UNA ALLA VOLTA
  cv::createTrackbar("Morph Op", "HSV Trackbars2", &morph_op, 4, nothing);
// 0: None, 1: Open, 2: Close, 3: Erode, 4: Dilate
  cv::createTrackbar("Kernel Size", "HSV Trackbars2", &kernel_size, 20, nothing);
  cv::createTrackbar("Apply Blur", "HSV Trackbars2", &apply_blur, 1, nothing);


  ros::Subscriber sub = nh.subscribe("/camera/image", 1, imageCallback);

  while (ros::ok()) {
    ros::spinOnce();
    cv::waitKey(1);
  }

  return 0;
}
