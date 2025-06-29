/* -------------------------------------------------------------------------- */
/*                                  INCLUDES                                  */
/* -------------------------------------------------------------------------- */
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Pose2D.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <std_msgs/String.h>

/* -------------------------------------------------------------------------- */
/*                         GLOBAL VARIABLES                                   */
/* -------------------------------------------------------------------------- */


cv::Mat current_frame;
double error;
cv::Point2f punto_sx,punto_dx;
std_msgs::String traffic_light_status; 
std_msgs::String stop;
std_msgs::String slow;
std_msgs::String empty;

// Costanti di controllo per il lane following
const double max_lane_error_threshold     = 35.0;
const double default_speed                = 0.20;
const double reduced_speed                = 0.13;
const double penalty_lane_alignment       = -0.002;
const double penalty_depth_difference     = -0.0014;
const double min_positive_y_diff          = 50.0;
const double max_positive_y_diff          = 90.0;
const double min_negative_y_diff          = -90.0;
const double max_negative_y_diff          = -150.0;



/* -------------------------------------------------------------------------- */
/*                                   PUBLISHER                                */
/* -------------------------------------------------------------------------- */


ros::Publisher cmdel_pub;

/* -------------------------------------------------------------------------- */
/*                                  FUNCTIONS                                 */
/* -------------------------------------------------------------------------- */

// Callback per ricevere lo stato del semaforo
void trafficLightCallback(const std_msgs::String::ConstPtr& msg) {
  if(msg->data.c_str()!=empty.data)
    traffic_light_status.data = msg->data;
  ROS_INFO("semaforo: %s", traffic_light_status.data.c_str());
}

// Callback per ricevere la posizione della linea bianca (sinistra)
void lineWhiteCallback(const geometry_msgs::Point::ConstPtr& msg) {
  punto_sx.x = msg->x;
  punto_sx.y = msg->y;
}

// Callback per ricevere la posizione della linea rossa (destra)
void lineRedCallback(const geometry_msgs::Point::ConstPtr& msg) {
  punto_dx.x = msg->x;
  punto_dx.y = msg->y;
}


/* -------------------------------------------------------------------------- */
/*                           IMAGE PROCESSING LOGIC                               */
/* -------------------------------------------------------------------------- */

// Callback principale per processare l'immagine
void processImage(const sensor_msgs::Image::ConstPtr& msg) {
try {
  current_frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
} catch (cv_bridge::Exception& e) {
  ROS_ERROR("cv_bridge error: %s", e.what());
  return;
}

geometry_msgs::Twist cmd;
cv::Point2f punto_lane;

// Calcolo centro corsia e centro immagine
double lane_center = (punto_sx.x + punto_dx.x) / 2.0;
double image_center=current_frame.cols / 2.0;
double error = lane_center-image_center;
  
// Punto medio tra le due linee
punto_lane.x=lane_center;
punto_lane.y=punto_sx.y;

// Parametri di penalizzazione e velocità iniziali
double balance_penalty = 0.00;
cmd.linear.x = default_speed;

// Se il semaforo non è rosso, continua a muoverti
if (traffic_light_status.data != stop.data)
{
  if ((std::abs(error)> max_lane_error_threshold)) 
  { 
    balance_penalty = penalty_lane_alignment;  // penalità errore orizzontale
    cmd.linear.x = reduced_speed;
      
  }

  double balance_penalty_1=0.00;

// Correzione in base alla differenza di profondità tra le due linee
if (((punto_sx.y-punto_dx.y)> min_positive_y_diff && (punto_sx.y-punto_dx.y)< max_positive_y_diff))
  { 

  balance_penalty_1 = penalty_depth_difference; // penalità dislivello tra linee
  cmd.linear.x = reduced_speed;

  }else if((punto_sx.y-punto_dx.y)< min_negative_y_diff && (punto_sx.y-punto_dx.y)> max_negative_y_diff)
  {
  balance_penalty_1 = penalty_depth_difference; // penalità dislivello tra linee
  cmd.linear.x = reduced_speed;
  }
 
  ROS_WARN("Errore_x %f",error);
  ROS_WARN("Errore_y %f",(punto_sx.y-punto_dx.y));

  // Velocità angolare in base all'errore

  cmd.angular.z = balance_penalty*error+ balance_penalty_1*((punto_sx.y-punto_dx.y));
}else{
  // Se semaforo rosso, fermati
  cmd.linear.x = 0.0;   
  cmd.angular.z = 0.0;
}

// Se semaforo giallo, dimezza velocità
if (traffic_light_status.data == slow.data) {
  cmd.linear.x /= 2.0;  
  cmd.angular.z /=2.0;
} 

cmdel_pub.publish(cmd);
cv::waitKey(1);
}

/* -------------------------------------------------------------------------- */
/*                                    MAIN                                    */
/* -------------------------------------------------------------------------- */

int main(int argc, char **argv) {
  ros::init(argc, argv, "turtlebot_control");
  ros::NodeHandle nh;

  // Valori di default per il semaforo
  traffic_light_status.data = "go"; 
  stop.data = "stop";
  slow.data = "slow";
  empty.data= "";

  cmdel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  ros::Subscriber imageSub = nh.subscribe("/camera/image", 5, processImage);
  ros::Subscriber white_line = nh.subscribe("/white_line", 5, lineWhiteCallback);
  ros::Subscriber red_line = nh.subscribe("/red_line", 5, lineRedCallback);
  ros::Subscriber semaforo_sub = nh.subscribe("/semaforo_status", 5, trafficLightCallback);
 

  ros::spin();
  return 0;
}