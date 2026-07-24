import rclpy
from rclpy.node import Node

import numpy as np

from peer_detector.msg import PeerDetection, PeerDetections
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped

class SwarmCaptain(Node):
    def __init__(self):
        super().__init__(
            'swarm_captain',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )
        
        self.avoidance_radius = self.get_parameter('avoidance_radius').value
        self.quark_saturation_distance = self.get_parameter('quark_saturation_distance').value
        self.quark_max_force = self.get_parameter('quark_max_force').value
        
        self.eps = self.get_parameter('eps').value
        
        self.k_avoidance = self.get_parameter('k_avoidance').value
        self.k_quark = self.get_parameter('k_quark').value
        self.k_directional = self.get_parameter('k_directional').value
        
        self.k_velocity = self.get_parameter('k_velocity').value
        self.k_heading = self.get_parameter('k_heading').value
        
        self.peer_detection_subscriber = self.create_subscription(
            PeerDetections,
            'peer_detection',
            self.peer_detection_callback,
            10
        )
        self.peer_detection_subscriber  # prevent unused variable warning
        
        self.temp_subscriber = self.create_subscription(
            Float64,
            'temp',
            self.temp_callback,
            10
        )
        self.temp_subscriber  # prevent unused variable warning
        
        self.velocity_publisher = self.create_publisher(
            Float64,
            '/mavros/setpoint_velocity/cmd_vel',
            10
        )
        
        self.peer_detections = np.empty((0, 2))  # (x, y)
        self.curr_temp = None
        self.prev_temp = None
        
    def peer_detection_callback(self, msg):
        if not msg.peer_detections:
            self.peer_detections = np.empty((0, 2)) # (x, y)
            return
        
        coords = [[d.relative_x, d.relative_y] for d in msg.peer_detections]
        self.peer_detections = np.array(coords)
        
    def temp_callback(self, msg):
        self.prev_temp = self.curr_temp if self.curr_temp is not None else msg.data
        self.curr_temp = msg.data

    def make_decision(self):
        net_force = self.temp_subscriber.get_avoidance() * self.k_avoidance + self.get_quark() * self.k_quark + self.get_directional() * self.k_directional
        
        vel = TwistStamped()
        vel.header.stamp = self.get_clock().now().to_msg()
        vel.twist.linear.x = net_force[0] * self.k_velocity
        vel.twist.linear.y = 0.0
        vel.twist.linear.z = 0.0
        vel.twist.angular.x = 0.0
        vel.twist.angular.y = 0.0
        vel.twist.angular.z = net_force[1] * self.k_heading
        
        self.velocity_publisher.publish(vel)
    
    def get_avoidance(self):
        force = np.zeros(2)
        for peer in self.peer_detections:
            d = np.linalg.norm(peer)
            if d < self.avoidance_radius:
                repulsion = - peer * (1.0 / (d + self.eps))
                force += repulsion
        return force

    def get_quark(self):
        force = np.zeros(2)
        for peer in self.peer_detections:
            d = np.linalg.norm(peer)
            if d == 0:
                continue
            unit_d = peer / d
            saturation = d / (d + self.quark_saturation_distance)
            magnitude = self.quark_max_force * saturation
            force += unit_d * magnitude
        return force
    
    def get_directional(self):
        pass
    
def main(args=None):
    rclpy.init(args=args)
    swarm_captain = SwarmCaptain()
    rclpy.spin(swarm_captain)
    swarm_captain.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()