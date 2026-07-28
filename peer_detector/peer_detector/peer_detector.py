import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import numpy as np

from geometry_msgs.msg import PolygonStamped, Point32

class PeerDetector(Node):
    def __init__(self):
        super().__init__(
            'peer_detector',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )
        
        camera_params = self.get_parameters_by_prefix('cameras')
        
        self.marker_R = self.get_parameter('marker_R').value
        self.lower_color = np.array(self.get_parameter('lower_color').value)
        self.upper_color = np.array(self.get_parameter('upper_color').value)
        self.min_area_percent = self.get_parameter('min_area_percent').value
        self.max_area_percent = self.get_parameter('max_area_percent').value
        self.enable_morph = self.get_parameter('enable_morph').value
        self.min_radius = self.get_parameter('min_radius').value
        
        self.cameras_info = {}
        self.camera_subscribers = {}
        
        self.publisher = self.create_publisher(
            msg_type=PolygonStamped,
            topic='peer_detection',
            qos_profile=10
        )
        
        self.bridge = CvBridge()
        
        self.latest_images = {}
        
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
                info['topic_name'],
                lambda img_msg, cam_id=cam_id: self.image_callback(img_msg, cam_id),
                10
            )
            self.camera_subscribers[cam_id] = subscriber
        
        for cam_id, info in self.cameras_info.items():
            info['fov_rad'] = np.radians(info['fov'])
            info['heading_rad'] = np.radians(info['heading'])
            info['width'] = info['resolution'][0]
            
        timer_frequency = self.get_parameter('detection_frequency').value  # Hz
        self.timer = self.create_timer(1.0 / timer_frequency, self.timer_callback)
            
    def image_callback(self, img_msg, cam_id):
        self.latest_images[cam_id] = img_msg

    def timer_callback(self):
        if not self.latest_images:
            return

        msg = PolygonStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        for cam_id, img_msg in list(self.latest_images.items()):
            try:
                cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().error(f'CV Bridge Error on {cam_id}: {e}')
                continue
                
            detected_markers = image_to_detection(cv_image, self.lower_color, self.upper_color, self.min_area_percent, self.max_area_percent, self.enable_morph, self.min_radius)
            cam = self.cameras_info[cam_id]
            fov_rad = cam['fov_rad']
            heading_rad = cam['heading_rad']
            width = cam['width']
            cam_pos_x, cam_pos_y = cam['position'][0], cam['position'][1]
            
            for marker in detected_markers:
                x_robot, y_robot = detection_to_robot_coords(
                    marker['x'], marker['y'], marker['R'], fov_rad, heading_rad, width, self.marker_R, cam_pos_x, cam_pos_y
                )
                point = Point32()
                point.x = float(x_robot)
                point.y = float(y_robot)
                point.z = 0.0
                msg.polygon.points.append(point)
        
        self.publisher.publish(msg)
        self.latest_images.clear()
    
def main(args=None):
    rclpy.init(args=args)
    peer_detector = PeerDetector()
    rclpy.spin(peer_detector)
    peer_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

def image_to_detection(image, lower_hsv, upper_hsv, min_area_percent=0.01, max_area_percent=50.0, enable_morph=True, min_radius=5):
    
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    hmin, smin, vmin = lower_hsv
    hmax, smax, vmax = upper_hsv
    
    if hmin <= hmax:
        mask = cv2.inRange(hsv_image, np.array([hmin, smin, vmin]), np.array([hmax, smax, vmax]))
    else:
        mask1 = cv2.inRange(hsv_image, np.array([hmin, smin, vmin]), np.array([179, smax, vmax]))
        mask2 = cv2.inRange(hsv_image, np.array([0, smin, vmin]), np.array([hmax, smax, vmax]))
        mask = cv2.bitwise_or(mask1, mask2)
        
    if enable_morph:
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
    total_pixels = image.shape[0] * image.shape[1]
    amin = (min_area_percent / 100.0) * total_pixels
    amax = (max_area_percent / 100.0) * total_pixels
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    detected_markers = []
    
    for contour in contours:
        area = cv2.contourArea(contour)
        if amin <= area <= amax:
            ((x, y), radius) = cv2.minEnclosingCircle(contour)
            if radius > min_radius:
                detected_markers.append({
                    'x': x,
                    'y': y,
                    'R': radius
                })
                
    return detected_markers
    
def detection_to_robot_coords(x_cam, y_cam, marker_R_cam, fov_rad, heading_rad, width, marker_R, cam_pos_x, cam_pos_y):
    phi_rad = fov_rad * (x_cam / width - 0.5) #right hand with z ground facing
    d = (marker_R * width) / (marker_R_cam * fov_rad) #distance to marker in meters
    total_heading_rad = phi_rad + heading_rad
    x_robot = d * np.cos(total_heading_rad) + cam_pos_x
    y_robot = d * np.sin(total_heading_rad) + cam_pos_y
    return x_robot, y_robot