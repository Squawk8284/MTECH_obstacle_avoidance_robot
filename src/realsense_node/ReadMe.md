# RealSense Node for ROS  

This package provides a ROS node for publishing RealSense camera data, including color images, aligned depth images, and raw depth images.  

## Features  
- Publishes color images to a ROS topic.  
- Publishes aligned depth images to a ROS topic.  
- Publishes raw depth images to a ROS topic.  

## Prerequisites  
- Python 3  
- ROS (Robot Operating System)  
- `pyrealsense2` library  
- `numpy` library  
- `cv_bridge` package  
- `sensor_msgs` and `geometry_msgs` ROS message types  

## Installation  
1. Install the required dependencies:  
    ```bash  
    pip install pyrealsense2 numpy  
    sudo apt-get install ros-<distro>-cv-bridge ros-<distro>-sensor-msgs ros-<distro>-geometry-msgs  
    ```  
    Replace `<distro>` with your ROS distribution (e.g., `noetic`, `melodic`).  

2. Clone this repository into your ROS workspace:  
    ```bash  
    cd ~/catkin_ws/src  
    git clone <repository_url>  
    cd ~/catkin_ws  
    catkin_make  
    ```  

## Usage  
1. Launch the ROS core:  
    ```bash  
    roscore  
    ```  

2. Run the RealSense node:  
    ```bash  
    rosrun realsense_node realsense_publisher.py  
    ```  

## Parameters  
The following parameters can be set via the ROS parameter server:  
- `camera_color_image_raw_topic`: Topic name for publishing color images (default: `/camera/color/image_raw`).  
- `depth_image_topic`: Topic name for publishing aligned depth images (default: `/camera/depth/image_raw`).  

## Topics  
The node publishes the following topics:  
- **Color Image**: Publishes color images to the topic specified by `camera_color_image_raw_topic`.  
- **Aligned Depth Image**: Publishes aligned depth images to the topic specified by `depth_image_topic`.  
- **Raw Depth Image**: Publishes raw depth images to `/camera/depth/image_rect_raw`.  

## Code Overview  
The node uses the Intel RealSense SDK (`pyrealsense2`) to capture frames from the camera. It aligns the depth frames to the color frames and publishes the following data:  
- **Color Image**: Converted to a ROS `sensor_msgs/Image` message using `cv_bridge`.  
- **Aligned Depth Image**: Depth data aligned to the color frame, converted to a ROS `sensor_msgs/Image` message.  
- **Raw Depth Image**: Unaligned depth data, converted to a ROS `sensor_msgs/Image` message.  

The node runs in a loop until it is shut down, ensuring continuous data publishing.   

## Acknowledgments  
- Intel RealSense SDK  
- ROS community  

For any issues or contributions, feel free to open an issue or submit a pull request.  