import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import numpy as np

from peer_detector.msg import PeerDetection, PeerDetections

class PeerDetector(Node):
    def __init__(self):
        super().__init__(
            'peer_detector',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )
        
        camera_params = self.get_parameters_by_prefix('cameras')
        
        self.marker_R = self.get_parameter('marker_R').value
        
        self.cameras_info = {}
        self.camera_subscribers = {}
        
        self.publisher = self.create_publisher(
            msg_type=PeerDetections,
            topic='peer_detection',
            qos_profile=10
        )
        
        self.bridge = CvBridge()
        
        for key, param in camera_params.items():
            cam_id, prop = key.split('.', 1)
            if cam_id not in self.cameras_info:
                self.cameras_info[cam_id] = {}
            self.cameras_info[cam_id][prop] = param.value

        for cam_id, info in self.cameras_info.items():
            self.get_logger().info(f"Loaded [{cam_id}]: {info}")
        
        for cam_id, info in self.cameras_info.items():
            subscriber = self.create_subscription(
                Image,
                info['topic'],
                lambda img_msg, cam_id=cam_id: self.image_callback(img_msg, cam_id),
                10
            )
            self.camera_subscribers[cam_id] = subscriber
        
        for cam_id, info in self.cameras_info.items():
            info['fov_rad'] = np.radians(info['fov'])
            info['heading_rad'] = np.radians(info['heading'])
            info['width'] = info['resolution'][0]
            
    def image_callback(self, img_msg, cam_id):
        msg = PeerDetections()
        msg.header.stamp = self.get_clock().now().to_msg()

        #cv2 HSV masking and contouring right here
        detected_markers = {} #x_camera, y_camera, marker_R_camera, [x,y,R]
        
        cam = self.cameras_info[cam_id]
        fov_rad = cam['fov_rad']
        heading_rad = cam['heading_rad']
        width = cam['width']
        cam_x, cam_y = cam['position'][0], cam['position'][1]
        
        for marker in detected_markers:
            phi_rad = fov_rad * (marker['x'] / width - 0.5) #right hand with z ground facing
            d = (self.marker_R * width) / (marker['R'] * fov_rad) #distance to marker in meters
            total_heading_rad = phi_rad + heading_rad
            x_robot = d * np.sin(total_heading_rad) + cam_x
            y_robot = d * np.cos(total_heading_rad) + cam_y
            detection = PeerDetection()
            detection.relative_x = float(x_robot)
            detection.relative_y = float(y_robot)
            detection.confidence = 1.0
            msg.peer_detections.append(detection)
            
        self.publisher.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    peer_detector = PeerDetector()
    rclpy.spin(peer_detector)
    peer_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()