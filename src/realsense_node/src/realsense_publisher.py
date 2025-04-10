#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped

def main():
    rospy.init_node('realsense_publisher', anonymous=True)
    
    color_camera_raw_topic = rospy.get_param('camera_color_image_raw_topic','/camera/color/image_raw')
    depth_image_topic = rospy.get_param('depth_image_topic','/camera/depth/image_raw')
    # Publishers for topics a, b, c
    pub_color = rospy.Publisher(color_camera_raw_topic, Image, queue_size=10)  # Topic for color image
    pub_aligned_depth = rospy.Publisher(depth_image_topic, Image, queue_size=10)  # Topic for aligned depth
    pub_raw_depth = rospy.Publisher('/camera/depth/image_rect_raw', Image, queue_size=30)  # Topic for raw depth
    # pub_pose = rospy.Publisher('/mavros/local_position/pose', PoseStamped, queue_size=10)
    
    # Create a pipeline
    pipeline = rs.pipeline()
    config = rs.config()

    # Enable streams
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    
    # Start the pipeline
    pipeline.start(config)

    # Create alignment object
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Create a CvBridge object
    bridge = CvBridge()
    
    # rate = rospy.Rate(30)

    try:
        while not rospy.is_shutdown():
            # Wait for frames
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)

            # Get frames
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not aligned_depth_frame or not color_frame or not depth_frame:
                continue
            time_ros = rospy.Time.now()
            # Convert frames to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            aligned_depth_image = np.asanyarray(aligned_depth_frame.get_data())
            raw_depth_image = np.asanyarray(depth_frame.get_data())
            
            color_image = color_image.astype(np.uint8)
            aligned_depth_image = aligned_depth_image.astype(np.uint16)            
            raw_depth_image = raw_depth_image.astype(np.uint16)            
            
            # Publish color image
            color_image = bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            color_image.header.frame_id = "camera_link"
            color_image.header.stamp = time_ros
            

            # Publish aligned depth image
            aligned_depth_image = bridge.cv2_to_imgmsg(aligned_depth_image, encoding="16UC1")
            aligned_depth_image.header.frame_id = "camera_link"
            aligned_depth_image.header.stamp = time_ros

            # Publish raw depth image
            raw_depth_image = bridge.cv2_to_imgmsg(raw_depth_image, encoding="16UC1")
            raw_depth_image.header.frame_id = "camera_link"
            raw_depth_image.header.stamp = time_ros
            
            
            pub_color.publish(color_image)
            pub_raw_depth.publish(raw_depth_image)
            pub_aligned_depth.publish(aligned_depth_image)
            
            # rate.sleep()

    finally:
        pipeline.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
