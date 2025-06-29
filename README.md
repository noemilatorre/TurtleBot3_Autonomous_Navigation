# Applied_Robotics_project

# 🚧 TurtleBot3 Lane Following & Traffic Light Recognition

This ROS package enables a **TurtleBot3 Waffle Pi** to autonomously navigate inside a lane, stop at traffic lights, and recognize traffic signs (like STOP).

---


## 🎯 Project Goals

- Follow a colored lane using a camera
- Detect and respond to **red, yellow, and green** traffic lights
- Recognize a **STOP** sign
- Combine camera and LiDAR data for robust behavior

---
## 📁 File Structure

src/
<br>
├── turtlebot_control.cpp # Main node: decision-making, movement control <br>
├── vision_processing.cpp # Traffic light and line segmentation and color classification <br>
├── line_hsv_tuner.cpp # Debugging tool for visualizing lane lines (optional) <br>
├── trafficlight_hsv_tuner.cpp # Used to test thresholds and parameters for traffic lights (optional) <br>


---

## 📄 File Descriptions

### `progetto.cpp`
Main node that:
- Subscribes to camera 
- Controls robot movement based on lane and traffic light detection
- Manages state transitions (go, slow, stop)
- Publishes traffic status 

### `vision_processing.cpp`
Handles:
- Image segmentation to detect red, yellow, and green traffic lights
- Lane line segmentation
- STOP sign segmentation and recognition
- Hue, saturation, and brightness (HSV) filtering
- ROI analysis and contour-based classification

---

## 🛠️ How to Build
cd ~/catkin_ws <br>
catkin_make <br>
source devel/setup.bash <br>

---

## 🚀 How to Run
rosrun autorace vision_processing_node <br>
rosrun autorace turtlebot_control_node 

---

## Autors
- Colacicco Nunziamaria
- La Torre Noemi 