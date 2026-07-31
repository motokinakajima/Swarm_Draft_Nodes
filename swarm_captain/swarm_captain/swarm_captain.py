import rclpy
from rclpy.node import Node

import numpy as np

#RGBA, sensor_msg/image

from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped, PolygonStamped, Vector3Stamped, Point32

class SwarmCaptain(Node):
    def __init__(self):
        super().__init__(
            'swarm_captain',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )
        
        self.linear_movement_threshold_degrees = self.get_parameter('linear_movement_threshold_degrees').value
        self.heading_deadzone_eps = self.get_parameter('heading_deadzone_eps').value
        self.heading_tiebreak_rate = self.get_parameter('heading_tiebreak_rate').value
        
        self.k_avoidance = self.get_parameter('k_avoidance').value
        self.k_quark = self.get_parameter('k_quark').value
        self.k_directional = self.get_parameter('k_directional').value
        
        self.k_velocity = self.get_parameter('k_velocity').value
        self.k_heading = self.get_parameter('k_heading').value
        
        self.avoidance_radius = self.get_parameter('avoidance_radius').value
        self.quark_saturation_distance = self.get_parameter('quark_saturation_distance').value
        self.quark_max_force = self.get_parameter('quark_max_force').value
        
        self.pub_force_avoid = self.create_publisher(Vector3Stamped, 'debug/force_avoidance', 10)
        self.pub_force_quark = self.create_publisher(Vector3Stamped, 'debug/force_quark', 10)
        self.pub_force_dir = self.create_publisher(Vector3Stamped, 'debug/force_directional', 10)
        self.pub_force_net = self.create_publisher(Vector3Stamped, 'debug/force_net', 10)
        
        self.eps = self.get_parameter('eps').value
        self.directional_eps = self.get_parameter('directional_eps').value
        self.directional_ascend = self.get_parameter('directional_ascend').value
        self.directional_sign = 1.0 if self.directional_ascend else -1.0
        
        self.peer_detection_subscriber = self.create_subscription(
            PolygonStamped,
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

        self.velocity_subscriber = self.create_subscription(
            TwistStamped,
            'velocity',
            self.velocity_callback,
            10
        )
        self.velocity_subscriber  # prevent unused variable warning

        self.velocity_publisher = self.create_publisher(
            TwistStamped,
            'mavros/setpoint_velocity/cmd_vel',
            10
        )
        
        self.peer_detections = np.empty((0, 2))  # (x, y)
        self.curr_temp = None
        self.prev_temp_sample = None  # temp value captured at the previous control tick
        self.body_velocity = np.zeros(2)  # (forward, left), m/s, body frame
        self.yaw_rate = 0.0  # rad/s

        control_frequency = 10.0  # Hz
        self.dt = 1.0 / control_frequency
        self.timer = self.create_timer(self.dt, self.make_decision)
        
    def peer_detection_callback(self, msg):
        if not msg.polygon.points:
            self.peer_detections = np.empty((0, 2)) # (x, y)
            return
        
        coords = [[p.x, p.y] for p in msg.polygon.points]
        self.peer_detections = np.array(coords)
        
    def temp_callback(self, msg):
        self.curr_temp = msg.data

    def velocity_callback(self, msg):
        self.body_velocity = np.array([msg.twist.linear.x, msg.twist.linear.y])
        self.yaw_rate = msg.twist.angular.z

    def make_decision(self):
        now = self.get_clock().now().to_msg()
    
        f_avoid = self.get_avoidance() * self.k_avoidance
        f_quark = self.get_quark() * self.k_quark
        f_dir = self.get_directional() * self.k_directional
        net_force = f_avoid + f_quark + f_dir
        net_force_magnitude = np.linalg.norm(net_force)
        magnitude_threshold_x = net_force_magnitude * np.cos(np.radians(self.linear_movement_threshold_degrees))
    
        def publish_vector(publisher, force_array):
            msg = Vector3Stamped()
            msg.header.stamp = now
            msg.header.frame_id = 'base_link'
            msg.vector.x = float(force_array[0])
            msg.vector.y = float(force_array[1])
            msg.vector.z = 0.0
            publisher.publish(msg)

        publish_vector(self.pub_force_avoid, f_avoid)
        publish_vector(self.pub_force_quark, f_quark)
        publish_vector(self.pub_force_dir, f_dir)
        publish_vector(self.pub_force_net, net_force)
        
        vel = TwistStamped()
        vel.header.stamp = self.get_clock().now().to_msg()
        if net_force[0] > magnitude_threshold_x:
            vel.twist.linear.x = net_force[0] * self.k_velocity
        else:
            vel.twist.linear.x = 0.0
        vel.twist.linear.y = 0.0
        vel.twist.linear.z = 0.0
        vel.twist.angular.x = 0.0
        vel.twist.angular.y = 0.0
        if abs(net_force[1]) < self.heading_deadzone_eps and net_force[0] < 0.0:
            # net_force points (almost) directly opposite our heading. The
            # y-component that normally drives yaw vanishes right at this
            # antipodal point, so without a tie-breaker the robot would sit
            # frozen instead of turning around. Force a deterministic turn
            # direction to break the symmetry.
            vel.twist.angular.z = self.heading_tiebreak_rate
        else:
            vel.twist.angular.z = net_force[1] * self.k_heading
        
        self.velocity_publisher.publish(vel)
    
    def get_avoidance(self):
        force = np.zeros(2)
        for peer in self.peer_detections:
            d = np.linalg.norm(peer)
            if d == 0:
                continue
            if d < self.avoidance_radius:
                unit_away = -peer / d
                magnitude = 1.0 / (d + self.eps)
                force += unit_away * magnitude
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
        force = np.zeros(2)

        if self.prev_temp_sample is not None and self.curr_temp is not None:
            # Displacement isn't measured directly (no absolute position input);
            # estimate this tick's step from body-frame velocity instead.
            delta_pos = self.body_velocity * self.dt
            step_sq = float(np.dot(delta_pos, delta_pos))

            if step_sq > self.directional_eps:
                delta_f = self.curr_temp - self.prev_temp_sample
                projection_scale = delta_f / (step_sq + self.directional_eps)
                projected_gradient = delta_pos * projection_scale
                force = self.directional_sign * projected_gradient

        # Snapshot this tick's temp as the "previous" sample for the next tick,
        # keeping delta_pos and delta_f synchronized to the same time window.
        self.prev_temp_sample = self.curr_temp

        return force
    
def main(args=None):
    rclpy.init(args=args)
    swarm_captain = SwarmCaptain()
    rclpy.spin(swarm_captain)
    swarm_captain.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()