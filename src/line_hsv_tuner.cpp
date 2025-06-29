/* -------------------------------------------------------------------------- */
/*                                  INCLUDES                                  */
/* -------------------------------------------------------------------------- */
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

/* -------------------------------------------------------------------------- */
/*                         GLOBAL VARIABLES                                   */
/* -------------------------------------------------------------------------- */


// ROSSO
int h1_min = 0, s1_min = 70, v1_min = 50;
int h1_max = 50, s1_max = 255, v1_max = 255;
int h2_min = 150, s2_min = 70, v2_min = 50;
int h2_max = 200 , s2_max = 255, v2_max = 255;

// BIANCO
int w_min = 0, ws_min = 0, wv_min = 175;
int w_max = 93, ws_max = 205, wv_max = 255;



// Blur e Sharpen
int blur_ksize = 1; 
int sharpen_amount = 0;
int v_brightness = 100;  // Valore centrale = nessuna modifica
int equalize_v = 0;       // 0 = disattivo, 1 = attivo

int morph_op = 0;  // 0 = None, 1 = Open, 2 = Close, 3 = Erode, 4 = Dilate
int morph_kernel_size = 5;  
int morph_op_red = 0, morph_kernel_red = 5;
int morph_op_white = 0, morph_kernel_white = 5;

int alpha_slider = 100;  // 100 rappresenta 1.0
int sigma_space=0, sigma_color=0;
int gamma_slider = 100; 

/* -------------------------------------------------------------------------- */
/*                         FUNCTION PROTOTYPES                                */
/* -------------------------------------------------------------------------- */

void applyMorph(cv::Mat& mask, int operation, int kernelSizeInput) {
  if (operation <= 0 || operation > 4) return;

  // Assicura che il kernel sia dispari e >= 1
  int ksize = kernelSizeInput;
  if (ksize < 1) ksize = 1;
  if (ksize % 2 == 0) ksize += 1;

  int morph_type;
  switch (operation) {
    case 1: morph_type = cv::MORPH_OPEN; break;
    case 2: morph_type = cv::MORPH_CLOSE; break;
    case 3: morph_type = cv::MORPH_ERODE; break;
    case 4: morph_type = cv::MORPH_DILATE; break;
  }

  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ksize, ksize));
  cv::morphologyEx(mask, mask, morph_type, kernel);
}



void nothing(int, void*) {}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
  cv::Mat frame, hsv;
  try {
    frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }


  // Applica sharpening, evidenzia i bordi e i dettagli fini in un'immagine
  if (sharpen_amount > 0) {
    cv::Mat kernel = (cv::Mat_<float>(3,3) <<
        0, -1, 0,
       -1, 5 + sharpen_amount, -1,
        0, -1, 0);
    cv::filter2D(frame, frame, -1, kernel);
  }

  cv::Mat denoised;
  cv::bilateralFilter(frame, denoised, 0, sigma_color, sigma_space);
  // (d (Se è≤0, calcola automaticamente in base a sigmaSpace),sigma_color(quanto i colori devono essere simili per essere messi insieme,alto-> sfoca colori anche se molto diversi),
  //  sigma_space (quanto il filtro deve influenzare pixel lontani spazialmente, alto →> sfoca su area spaziale più ampia))
  frame = denoised;

  // Converti in HSV
  cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

  // Split dei canali HSV
  std::vector<cv::Mat> hsv_channels;
  cv::split(hsv, hsv_channels);
  // Applica sfocatura (se > 1)
  if (blur_ksize % 2 == 0) blur_ksize++;  // Assicura kernel dispari
  if (blur_ksize > 1) {
    cv::GaussianBlur(hsv_channels[2], hsv_channels[2], cv::Size(blur_ksize, blur_ksize), 0); //si può provare solo sul canale V o sull'intera immagine
  }

  double alpha = alpha_slider / 100.0;  // converti da intero a double
	hsv_channels[2].convertTo(hsv_channels[2], -1, alpha, 0);

  // Correzione gamma, gamma < 1 scurisce, gamma > 1 schiarisce
  cv::Mat lut(1, 256, CV_8U);
  double gamma = gamma_slider / 100.0;  
  for (int i = 0; i < 256; i++) {
      lut.at<uchar>(i) = cv::saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
  }
  cv::LUT(hsv_channels[2], lut, hsv_channels[2]);


  // Modifica luminosità (V) tramite trackbar
  int delta = v_brightness - 100;
  if (delta != 0) {
    hsv_channels[2] += delta; //delta rappresenta la quantità di aumento o diminuzione della luminosità.
    // Clipping manuale tra 0 e 255
    cv::threshold(hsv_channels[2], hsv_channels[2], 255, 255, cv::THRESH_TRUNC);
    cv::threshold(hsv_channels[2], hsv_channels[2], 0, 0, cv::THRESH_TOZERO);
  }

  // Equalizzazione del canale V (opzionale)
  // Migliora il contrasto (è meglio farlo insieme al blur)
  if (equalize_v == 1) {
    cv::equalizeHist(hsv_channels[2], hsv_channels[2]);
  }

  // Merge dei canali HSV modificati
  cv::merge(hsv_channels, hsv);

  // Maschere per ROSSO
  cv::Mat red_mask1, red_mask2, red_mask;
  cv::inRange(hsv, cv::Scalar(h1_min, s1_min, v1_min), cv::Scalar(h1_max, s1_max, v1_max), red_mask1);
  cv::inRange(hsv, cv::Scalar(h2_min, s2_min, v2_min), cv::Scalar(h2_max, s2_max, v2_max), red_mask2);
  cv::bitwise_or(red_mask1, red_mask2, red_mask);

  int height = red_mask.rows;
  int width  = red_mask.cols;
  cv::Rect roi_linea_rossa    = cv::Rect(0, height * 2 / 3, width, height / 3);

  // Maschera per BIANCO
  cv::Mat white_mask;
  cv::inRange(hsv, cv::Scalar(w_min, ws_min, wv_min), cv::Scalar(w_max, ws_max, wv_max), white_mask);
  cv::Rect roi_linea_bianca    = cv::Rect(0, height * 2 / 3, width, height / 3);

  cv::Mat roi_bianca = white_mask(roi_linea_bianca).clone();
  cv::Mat roi_rossa = red_mask(roi_linea_rossa).clone();

  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
  cv::Mat kernel3 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
  cv::morphologyEx(roi_rossa, roi_rossa, cv::MORPH_CLOSE, kernel);

  cv::morphologyEx(roi_bianca, roi_bianca, cv::MORPH_OPEN, kernel); 
  cv::dilate(roi_bianca, roi_bianca, kernel);
  cv::morphologyEx(roi_bianca, roi_bianca, cv::MORPH_CLOSE, kernel); 
  cv::morphologyEx(roi_bianca, roi_bianca, cv::MORPH_DILATE, kernel3);
  
  applyMorph(roi_rossa, morph_op_red, morph_kernel_red);
  applyMorph(roi_bianca, morph_op_white, morph_kernel_white);



  // Visualizzazione
  cv::imshow("Original", frame);
  //cv::imshow("After Gaussian", hsv);
  cv::imshow("Red Mask", roi_rossa);
  cv::imshow("White Mask", roi_bianca);
  cv::waitKey(1);
}

/* -------------------------------------------------------------------------- */
/*                                    MAIN                                    */
/* -------------------------------------------------------------------------- */


int main(int argc, char** argv) {
  ros::init(argc, argv, "hsv_tuner_red_white");
  ros::NodeHandle nh;

  // Finestre
  cv::namedWindow("Operazioni immagine intera");
  cv::namedWindow("Red HSV Tuner");
  cv::namedWindow("White HSV Tuner");

 
  cv::createTrackbar("V Brightness", "Operazioni immagine intera", &v_brightness, 200, nothing);
  cv::createTrackbar("Equalize V", "Operazioni immagine intera", &equalize_v, 1, nothing);
  cv::createTrackbar("Luminosità convert_to", "Operazioni immagine intera", &alpha_slider, 255, nothing);
  cv::createTrackbar("Bilateral_filter sigma_color", "Operazioni immagine intera", &sigma_color, 255, nothing);
  cv::createTrackbar("Bilateral_filter sigma_space", "Operazioni immagine intera", &sigma_space, 255, nothing);
  cv::createTrackbar("Correzione gamma", "Operazioni immagine intera", &gamma_slider, 255, nothing);

  cv::createTrackbar("Blur", "Operazioni immagine intera", &blur_ksize, 20, nothing);
  // Trackbar per ROSSO
  cv::createTrackbar("H1 min", "Red HSV Tuner", &h1_min, 180, nothing);
  cv::createTrackbar("H1 max", "Red HSV Tuner", &h1_max, 180, nothing);
  cv::createTrackbar("S1 min", "Red HSV Tuner", &s1_min, 255, nothing);
  cv::createTrackbar("S1 max", "Red HSV Tuner", &s1_max, 255, nothing);
  cv::createTrackbar("V1 min", "Red HSV Tuner", &v1_min, 255, nothing);
  cv::createTrackbar("V1 max", "Red HSV Tuner", &v1_max, 255, nothing);

  cv::createTrackbar("H2 min", "Red HSV Tuner", &h2_min, 180, nothing);
  cv::createTrackbar("H2 max", "Red HSV Tuner", &h2_max, 180, nothing);
  cv::createTrackbar("S2 min", "Red HSV Tuner", &s2_min, 255, nothing);
  cv::createTrackbar("S2 max", "Red HSV Tuner", &s2_max, 255, nothing);
  cv::createTrackbar("V2 min", "Red HSV Tuner", &v2_min, 255, nothing);
  cv::createTrackbar("V2 max", "Red HSV Tuner", &v2_max, 255, nothing);
 
  cv::createTrackbar("Morph Op (Red)", "Red HSV Tuner", &morph_op_red, 4, nothing);
  cv::createTrackbar("Morph Kernel (Red)", "Red HSV Tuner", &morph_kernel_red, 21, nothing);

  cv::createTrackbar("Morph Op (White)", "White HSV Tuner", &morph_op_white, 4, nothing);
  cv::createTrackbar("Morph Kernel (White)", "White HSV Tuner", &morph_kernel_white, 21, nothing);


  // Trackbar per BIANCO
  cv::createTrackbar("H min", "White HSV Tuner", &w_min, 180, nothing);
  cv::createTrackbar("H max", "White HSV Tuner", &w_max, 180, nothing);
  cv::createTrackbar("S min", "White HSV Tuner", &ws_min, 255, nothing);
  cv::createTrackbar("S max", "White HSV Tuner", &ws_max, 255, nothing);
  cv::createTrackbar("V min", "White HSV Tuner", &wv_min, 255, nothing);
  cv::createTrackbar("V max", "White HSV Tuner", &wv_max, 255, nothing);


  // ROS
  //ros::Subscriber sub = nh.subscribe("/camera/rgb/image_raw", 1, imageCallback);
  ros::Subscriber sub = nh.subscribe("/camera/image", 1, imageCallback);

  // Ciclo
  while (ros::ok()) {
    ros::spinOnce();
    cv::waitKey(1);
  }

  return 0;
}


