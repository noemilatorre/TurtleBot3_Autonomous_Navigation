/* -------------------------------------------------------------------------- */
/*                                  INCLUDES                                  */
/* -------------------------------------------------------------------------- */
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/String.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"


/* -------------------------------------------------------------------------- */
/*                          GLOBAL VARIABLES                                  */
/* -------------------------------------------------------------------------- */

cv::Mat mask_1, mask_2;
std::vector<cv::Point> contour;
std::vector<std::vector<cv::Point>> contours_after;
cv::Point2f punto1,prev_punto_sx,prev_punto_dx, punto2;

std::string current_color;
std::string stable_color; 
int stable_color_counter = 0;
const int stability_threshold = 3; // Numero minimo di frame per stabilizzare colore

const int red_hue_min1 =0; 
const int red_hue_max1 = 15;
const int red_hue_min2 = 200;
const int red_hue_max2 = 360;
const int yellow_hue_min = 15;
const int yellow_hue_max = 30;
const int green_hue_min = 110;
const int green_hue_max = 150;

const int min_illumination = 73;
const double min_area_green = 235.0;
const double max_area_green = 1050.0;
const double min_area = 110.0;
const double max_area = 300.0;

const double min_circularity = 0.6;
const double min_red_saturation = 150.0;

// -------------------- Costanti di soglia per cartello STOP --------------------

const double stop_sign_min_area = 3000.0;
const double stop_sign_max_area = 6000.0;
const double stop_sign_min_circularity = 0.5;
const double stop_sign_min_illumination = 140.0;


int stop=0;
int slow=0;
int go=0;

double previous_goodness = 0.00;
double traffic_light_distance = std::numeric_limits<float>::infinity();

cv::Mat mask_yellow, mask_green,mask_red;
cv::Mat roi_red; 
cv::Mat roi_yellow; 
cv::Mat roi_green; 
cv::Mat sign_roi; 
cv::Rect line_roi(0,0,0,0); 
std::vector<std::vector<cv::Point>> contours;
std_msgs::String color;

bool traffic_light_present; // Variabile per indicare la presenza di un semaforo davanti
bool sign_present; // Variabile per indicare la presenza di un cartello davanti


/* -------------------------------------------------------------------------- */
/*                          PUBLISHER AND SUBSCRIBER                          */
/* -------------------------------------------------------------------------- */

ros::Subscriber scanSub;
ros::Subscriber imageSub;
ros::Publisher traffic_light_pub;
ros::Publisher red_line_pub;
ros::Publisher white_line_pub;


/* -------------------------------------------------------------------------- */
/*                                  FUNCTIONS                                 */
/* -------------------------------------------------------------------------- */

// Calcola la "bontà" di un contorno in base alla sua illuminazione media.
double goodness(double avgV)
{
    return ((avgV / 255.0));
}

// Analizza i dati del laser frontale per valutare la distanza da semafori e cartelli di stop.

void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan) {
    double min_distance =  traffic_light_distance;

    // Analizza un settore frontale di +10 gradi rispetto all'asse centrale
    int center_index = 0;
    double target_angle_deg = 10.00;
    
    int delta_index = static_cast<int>(target_angle_deg * M_PI / 180.0 / scan->angle_increment);

    for (int i = center_index; i <= center_index + delta_index; ++i) { 

        double distance = scan->ranges[i];
        ROS_INFO("Distanza: %f", distance); 

        if (distance < 4.0 && traffic_light_present) 
        {   
            ROS_INFO("COLORE STABILIZZATO: %s", color.data.c_str());
            traffic_light_pub.publish(color);
            min_distance = distance;
        }
       
        if (distance < 2.0 && sign_present)
        {
            color.data="stop";
            ROS_INFO("CARTELLO: %s", color.data.c_str());
            traffic_light_pub.publish(color);
            min_distance = distance;
        }
    }
    traffic_light_distance = min_distance;
}

// Conversione da radianti a gradi 
double rad2deg(double radians)
{
    return radians * (180.0 / CV_PI);
}

// Conversione da gradi a radianti 
double deg2rad(double degrees)
{
    return degrees * (CV_PI/ 180.0);
}

// Trova il contorno più grande in una (ROI)
bool findcontours(const cv::Mat& roi, std::vector<cv::Point>& contour_out)
{
  bool found=false;
  std::vector<std::vector<cv::Point>> contour;
  cv::findContours(roi, contour, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  int max_len = 0;
  for (const auto& c : contour) {
    if (c.size() > max_len)  // Seleziona il contorno più grande
    {
      max_len = c.size();
      contour_out = c;
      found=true;
    }
  }
  return found;
}

// Calcola il valore medio dei pixel all'interno di un contorno
double avgValInContour(
    const cv::Mat& img,
    const std::vector<cv::Point>& object,
    bool cosine_correction = false)
{
    double sum = 0;
    int count = 0;
    cv::Rect bbox = cv::boundingRect(object);
    for (int y = bbox.y; y < bbox.y + bbox.height; y++)
    {
        const unsigned char* yRow = img.ptr<unsigned char>(y);
        for (int x = bbox.x; x < bbox.x + bbox.width; x++)
            if (cv::pointPolygonTest(object, cv::Point(x, y), false) > 0) //restituisce valore positivo se il punto all'interno del contorno
            {
                if (cosine_correction)
                    sum += rad2deg(std::acos(std::cos(deg2rad(yRow[x] * 2.0)))) / 2.0;
                else
                    sum += yRow[x];
                count++;
            }
    }

    return sum / count;
}

// Funzione per calcolare le caratteristiche dei contorni del semaforo
// Classifica i contorni in base alla loro tinta media (hue), area, circolarità, illuminazione media e saturazione
void processTrafficLightContours(std::vector<std::vector<cv::Point>> contours, cv::Mat hue_roi, cv::Mat ill_roi, cv::Rect roi, cv::String color,cv::Mat sat_roi)
{
    for (size_t k=0; k<contours.size(); k++)
    {
         double avg_hue = 2 * avgValInContour(hue_roi, contours[k], true); // Hue va da 0 a 180 //controlla true e false
         
         //ROS_INFO("avghue %f",avg_hue);

         double area = cv::contourArea(contours[k]);

        // Area per contorni verdi (semaforo verde) 
        if ((area < min_area_green || area > max_area_green) && color=="green") continue; 

        // Area per contorni rossi e gialli (semaforo rosso/giallo)
        if ((area < min_area || area > max_area) && color!="green")  continue; 

        //ROS_INFO("area %f",area);
           
        double perimeter = cv::arcLength(contours[k], true);
        double circularity = 4 * CV_PI * area / (perimeter * perimeter);

        
        if (circularity < min_circularity) continue; 
        //ROS_INFO("circularity %f",circularity); 

        double avg_illumination = avgValInContour(ill_roi, contours[k], false);
        

        if (avg_illumination < min_illumination) continue;
        //ROS_INFO("ill %f",avg_illumination);

        double avg_saturation = avgValInContour(sat_roi, contours[k]);
        //ROS_INFO("sat_roi %f",avg_saturation);

        if(avg_saturation< min_red_saturation && color=="red")
        continue;

        double g = goodness(avg_illumination);

        if (g > previous_goodness)
        {
            previous_goodness = g;

            // Gestisco quando sono accesi sia il rosso che il giallo
            if (stop == 1 && color == "yellow")
            {
                slow = 0;
                go = 0;
            }
            else if (slow == 1 && color == "red")
            {
                stop = 0;
                go = 0;
            }
            else 
            {
                stop = 0;
                slow = 0;
                go = 0;
                if (!contours_after.empty())
                    contours_after.clear();
            }
        }
        else
            continue;

        // Traslazione del contorno nella ROI globale
        std::vector<cv::Point> translated_contour = contours[k];
        for (auto& pt : translated_contour) {
            pt.y += roi.y;
            pt.x += roi.x;
        }

        contours_after.push_back(translated_contour);

        // Classificazione in base alla tinta media (hue) del contorno
        if ((avg_hue >= red_hue_min1 && avg_hue <= red_hue_max1) ||
            (avg_hue >= red_hue_min2 && avg_hue <= red_hue_max2)) 
        {
            stop++;
            if(color=="red")
                traffic_light_present=true;
            else
            {                 
                contours_after.pop_back();
                stop -= 1;
            }   
        }
        else if (avg_hue >= yellow_hue_min && avg_hue <= yellow_hue_max)
        {
            slow++;
            if(color=="yellow")
                traffic_light_present=true;
            else
            { 
                contours_after.pop_back();
                slow -= 1;
            }
        }
        else if (avg_hue >= green_hue_min && avg_hue <= green_hue_max)
        {
            go++;
            if(color=="green")
                traffic_light_present=true;
            else
            { 
                contours_after.pop_back();
                go -= 1;
            }
        }
    }
}

// Funzione per calcolare le caratteristiche dei contorni del cartello STOP
// Classifica i contorni in base alla loro tinta media (hue), area, circolarità, illuminazione media
void processSignContours(std::vector<std::vector<cv::Point>> contours, cv::Mat hue_roi, cv::Mat ill_roi, cv::Rect roi,cv::Mat sat_roi)
{
    for (size_t k=0; k<contours.size(); k++)
    {
        double avg_hue = 2 * avgValInContour(hue_roi, contours[k], true); 
         
        //ROS_INFO("avghue %f",avg_hue);

        double area = cv::contourArea(contours[k]);

        //ROS_INFO("area %f",area);
         
        if ((area < stop_sign_min_area || area > stop_sign_max_area)) continue; 

        double perimeter = cv::arcLength(contours[k], true);
        double circularity = 4 * CV_PI * area / (perimeter * perimeter);

        //ROS_INFO("circularity %f",circularity); 
        if (circularity < stop_sign_min_circularity) continue;  
         

        double avg_illumination = avgValInContour(ill_roi, contours[k], false);
        //ROS_INFO("ill %f",avg_illumination);

        if (avg_illumination < stop_sign_min_illumination) continue;

        std::vector<cv::Point> translated_contour = contours[k];

        for (auto& pt : translated_contour) {
            pt.y += roi.y;
            pt.x += roi.x;
        }

        contours_after.push_back(translated_contour);

        if ((avg_hue >= red_hue_min1 && avg_hue <= red_hue_max1) ||
            (avg_hue >= red_hue_min2 && avg_hue <= red_hue_max2)) 
        {
            stop++; 
            sign_present=true;
        }
    }
}

/* -------------------------------------------------------------------------- */
/*                      TRAFFIC LIGHT SEGMENTATION                            */
/* -------------------------------------------------------------------------- */

// Rileva il colore attivo del semaforo (rosso, giallo o verde) tramite segmentazione HSV e analisi dei contorni
std_msgs::String detectTrafficLightColor(const cv::Mat& img_bgr) {

    cv::Mat img_hsv_red;
    cv::Mat img_hsv_yellow;
    cv::Mat img_hsv_green;

    cv::cvtColor(img_bgr, img_hsv_red, cv::COLOR_BGR2HSV);
    cv::cvtColor(img_bgr, img_hsv_yellow, cv::COLOR_BGR2HSV);
    cv::cvtColor(img_bgr, img_hsv_green, cv::COLOR_BGR2HSV);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7,7));
    cv::Mat kernel1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15)); 
    cv::Mat kernel2 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::Mat kernel3 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7,7));

    // Split dei canali HSV
    std::vector<cv::Mat> img_hsv_chans_red,img_hsv_chans_yellow,img_hsv_chans_green;

    // Operazioni per maschera gialla 
    cv::split(img_hsv_yellow, img_hsv_chans_yellow);
    cv::equalizeHist(img_hsv_chans_yellow[2], img_hsv_chans_yellow[2]);
    cv::GaussianBlur(img_hsv_chans_yellow[0], img_hsv_chans_yellow[0], cv::Size(3,3), 0);
    cv::merge(img_hsv_chans_yellow, img_hsv_yellow);

    double alpha = 2.0; // Fattore di moltiplicazione per aumentare la luminosità
    cv::split(img_hsv_green, img_hsv_chans_green);
    img_hsv_chans_green[2].convertTo(img_hsv_chans_green[2], -1, alpha, 0);
    cv::merge(img_hsv_chans_green, img_hsv_green);

    cv::split(img_hsv_red, img_hsv_chans_red);
    cv::merge(img_hsv_chans_red, img_hsv_red);

    // Maschere per colore rosso
    cv::Mat mask_red1, mask_red2;
    cv::inRange(img_hsv_red, cv::Scalar(22, 70, 50), cv::Scalar(46, 255, 255), mask_red1);
    cv::inRange(img_hsv_red, cv::Scalar(160, 70, 50), cv::Scalar(180, 255, 255), mask_red2);
    cv::bitwise_or(mask_red1, mask_red2, mask_red);

    // Operazioni morfologiche maschera rossa
    cv::morphologyEx(mask_red, mask_red, cv::MORPH_OPEN, kernel2);
    cv::morphologyEx(mask_red, mask_red, cv::MORPH_DILATE, kernel2);
    cv::morphologyEx(mask_red, mask_red, cv::MORPH_CLOSE, kernel); 

    // Maschera per colore giallo
    cv::inRange(img_hsv_yellow, cv::Scalar(0, 100, 85), cv::Scalar(13, 255, 255), mask_yellow);  

    // Operazioni morfologiche maschera gialla
    cv::morphologyEx(mask_yellow, mask_yellow, cv::MORPH_DILATE, kernel3); 

    // Maschera per colore verde
    cv::inRange(img_hsv_green, cv::Scalar(45, 100, 100), cv::Scalar(75, 255, 255), mask_green);
    
    // Operazioni morfologiche maschera verde
    cv::morphologyEx(mask_green, mask_green, cv::MORPH_DILATE, kernel1);
    cv::morphologyEx(mask_green, mask_green, cv::MORPH_ERODE, kernel2);

    int height = mask_red.rows;
    int width  = mask_red.cols;

    // Dichiarazioni roi: Rect(x, y, width, height); 
    cv::Rect roi_yellow1(width*2/5, height*2/6, width*3/5, height*2/7);
    cv::Rect roi_red1(width*2/5, height*2/7, width*3/5, height/5);
    cv::Rect roi_green1(width*2/5, height/3, width*3/5, height/3); 
    cv::Rect roi_sign(width*3/10, height*2/20, width*5/10, height*5/15);
   
    contours_after.clear();
    traffic_light_present = false;
    sign_present=false;

    if (!mask_red.empty() || !mask_yellow.empty() || !mask_green.empty())  
    { 
        // ROI applicate alle varie maschere
        roi_red = mask_red(roi_red1).clone();
        roi_yellow = mask_yellow(roi_yellow1).clone();
        roi_green = mask_green(roi_green1).clone();
        sign_roi= mask_red(roi_sign).clone();

        // Tinta roi rossa per il cartello e rossa, gialla e verde per il semaforo
        cv::Mat hue_roi_sign = img_hsv_chans_red[0](roi_sign);
        cv::Mat hue_roi_red = img_hsv_chans_red[0](roi_red1);
        cv::Mat hue_roi_yellow = img_hsv_chans_yellow[0](roi_yellow1);
        cv::Mat hue_roi_green = img_hsv_chans_green[0](roi_green1);

        // Illuminazione roi rossa per il cartello e rossa, gialla e verde per il semaforo
        cv::Mat ill_roi_sign = img_hsv_chans_red[2](roi_sign);
        cv::Mat ill_roi_red = img_hsv_chans_red[2](roi_red1);
        cv::Mat ill_roi_yellow = img_hsv_chans_yellow[2](roi_yellow1);
        cv::Mat ill_roi_green = img_hsv_chans_green[2](roi_green1);

        // Saturazione roi rossa per il cartello e rossa, gialla e verde per il semaforo
        cv::Mat sat_roi_sign = img_hsv_chans_red[1](roi_sign);
        cv::Mat sat_roi_red = img_hsv_chans_red[1](roi_red1);
        cv::Mat sat_roi_yellow = img_hsv_chans_yellow[1](roi_yellow1);
        cv::Mat sat_roi_green = img_hsv_chans_green[1](roi_green1);

        // Contorni
        std::vector<std::vector<cv::Point>> contours_red, contours_yellow, contours_green, contours_cartello;
        cv::findContours(roi_red, contours_red, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::findContours(roi_yellow, contours_yellow, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::findContours(roi_green, contours_green, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::findContours(sign_roi, contours_cartello, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Inizializzo variabili per rconoscimento semaforo
        stop = 0;
        slow = 0;
        go = 0;
        previous_goodness = 0;

        // Classifica i contorni per colore per semaforo
        processTrafficLightContours(contours_red, hue_roi_red, ill_roi_red, roi_red1, "red", sat_roi_red);
        processTrafficLightContours(contours_yellow, hue_roi_yellow, ill_roi_yellow, roi_yellow1, "yellow", sat_roi_yellow);
        processTrafficLightContours(contours_green, hue_roi_green, ill_roi_green, roi_green1, "green", sat_roi_green);

        // Classifica i contorni per cartello
        processSignContours(contours_cartello, hue_roi_sign, ill_roi_sign, roi_sign, sat_roi_sign);
        
        std::string detected_color;

        if (stop >= slow && stop > go)
            detected_color= "stop";
        else if (slow > stop && slow > go)
            detected_color= "slow";
        else if (go > stop && go > slow)
            detected_color= "go";
        else
            detected_color = stable_color;

        // Tracking temporale
        if (detected_color == current_color) {
            stable_color_counter++;
        } else {
            current_color = detected_color;
            stable_color_counter = 1;
        }

        if (stable_color_counter >= stability_threshold) {
            if (stable_color != current_color) 
                stable_color = current_color;
        }

        color.data = stable_color;
    } else {
        color.data = "unknown";
    }
    return color;
}


/* -------------------------------------------------------------------------- */
/*                          LINE SEGMENTATION                                 */
/* -------------------------------------------------------------------------- */

// Rileva la linea (bianca o rossa) nell'immagine e restituisce i punti di interesse
geometry_msgs::Point detectLine(const cv::Mat& img_bgr, bool line) {
    cv::Mat img_hsv;
    cv::cvtColor(img_bgr, img_hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask;

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)); //(3,3)
    cv::Mat kernel1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));

    if(line==true) // Linea trovata bianca
    {
        // Segmentazione BIANCO
        cv::inRange(img_hsv, cv::Scalar(0, 0, 175), cv::Scalar(135, 205, 255), mask_2); 
        cv::morphologyEx(mask_2, mask_2, cv::MORPH_OPEN, kernel); 
        cv::dilate(mask_2, mask_2, kernel);
        cv::morphologyEx(mask_2, mask_2, cv::MORPH_CLOSE, kernel);  
        cv::morphologyEx(mask_2, mask_2, cv::MORPH_DILATE, kernel1);
       
    }else          // Linea trovata rossa
    {
        // Segmentazione ROSSO 
        cv::Mat red_mask1, red_mask2;
        cv::inRange(img_hsv, cv::Scalar(0, 70, 50), cv::Scalar(50, 255, 255), red_mask1);
        cv::inRange(img_hsv, cv::Scalar(150, 70, 50), cv::Scalar(200, 255, 255), red_mask2); 
        cv::bitwise_or(red_mask1, red_mask2, mask_1);
        cv::morphologyEx(mask_1, mask_1, cv::MORPH_CLOSE, kernel);

    }
    
    if(line==true) // Linea trovata bianca
        mask = mask_2.clone();
    else            // Linea trovata rossa
        mask = mask_1.clone();

    int height = mask.rows;
    int width  = mask.cols;
    line_roi    = cv::Rect(0, height * 2 / 3, width, height / 3); 

    bool found_color = false;

    if (!mask.empty()) 
    {
        cv::Mat roi = mask(line_roi).clone();
        found_color = findcontours(roi, contour);

        if (found_color && !contour.empty()) // Se trovo la linea
        {
            punto2.x = contour[0].x;
            punto2.y = contour[0].y;
            double x_sx = 1e9;
            double x_dx = contour[0].x;

            for (const auto& pt : contour) {
              if(line==true) // Linea trovata bianca prendo sul contorno il punto più a sinistra
              {
                if (pt.x < x_sx) {
                    x_sx = pt.x;
                    punto1.x = pt.x;
                    punto1.y = pt.y;
                }
              }else{         // Linea trovata rossa prendo sul contorno il punto più a destra
                if (pt.x > x_dx) {
                    x_dx = pt.x;
                    punto1.x = pt.x;
                    punto1.y=pt.y;
                }
              }
            }
        } else {
           if(line==true) // bianca
            {
                punto1.x = prev_punto_sx.x;
                punto1.y = prev_punto_sx.y;
            }else{
                punto1.x = prev_punto_dx.x;
                punto1.y = prev_punto_dx.y;
            }
        }
    }

    double norm;
    cv::Point2f tangent; 
    geometry_msgs::Point found_point;

    if (found_color) 
    {
        // Calcolo della tangente
        tangent = punto1 - punto2;
        norm = cv::norm(tangent);

        if (norm > 0.001) 
            tangent /= norm;

        // Prevedo quale può essere il prossimo punto a sinistra o a destra 
        // (da usare nel caso in cui non vengano trovate le due linee)
        cv::Point2f prediction = punto1 + 100.0f * tangent;
        
        if(line==true) 
        {
            prev_punto_sx.x = prediction.x;
            prev_punto_sx.y = prediction.y;
        }else{
            prev_punto_dx.x = prediction.x;
            prev_punto_dx.y = prediction.y;
        }
    }

    found_point.x=punto1.x;
    found_point.y = punto1.y;
    return found_point;
}

// Callback che elabora l'immagine ricevuta dalla telecamera
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {

    cv::Mat current_frame;
    try 
    {
        current_frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) 
    {
        ROS_ERROR("cv_bridge error: %s", e.what());
        return;
    }
    
    geometry_msgs::Point punto_sx = detectLine(current_frame, true);
    white_line_pub.publish(punto_sx);
    
    geometry_msgs::Point punto_dx = detectLine(current_frame, false);
    red_line_pub.publish(punto_dx);
    
    color = detectTrafficLightColor(current_frame);
    
    // Mostra contorni trovati semaforo
    cv::Mat contour_view = current_frame.clone();
    if (!contours_after.empty()) 
    {
        std::vector<std::vector<cv::Point>> draw = {contours_after};
        cv::drawContours(contour_view, draw, -1, cv::Scalar(0, 255, 255), 2);
    }
    cv::imshow("current_frame", contour_view);

    // Mostra maschera linee
    cv::imshow("Mask linea bianca", mask_2(line_roi));
    cv::imshow("Mask linea rossa", mask_1(line_roi));

    // Mostra maschera semaforo e cartello
    cv::imshow("Mask semaforo rossa", roi_red);
    cv::imshow("Mask semaforo gialla",  roi_yellow);
    cv::imshow("Mask semaforo verde",  roi_green);
    cv::imshow("Mask cartello", sign_roi);
    
    cv::waitKey(1);
}


/* -------------------------------------------------------------------------- */
/*                                    MAIN                                    */
/* -------------------------------------------------------------------------- */

int main(int argc, char** argv){
    ros::init(argc, argv, "vision_processing");
    ros::NodeHandle nh;

    scanSub = nh.subscribe("/scan", 10, processLaserScan);  
    imageSub = nh.subscribe("/camera/image", 5, imageCallback);
    traffic_light_pub = nh.advertise<std_msgs::String>("/semaforo_status", 5);
    white_line_pub = nh.advertise<geometry_msgs::Point>("/white_line", 5);
    red_line_pub = nh.advertise<geometry_msgs::Point>("/red_line", 5);
   
    ros::spin();
    return 0;
}
