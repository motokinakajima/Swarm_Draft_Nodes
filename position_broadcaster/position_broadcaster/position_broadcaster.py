import math

import rclpy
from rclpy.node import Node

import utm

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped, PolygonStamped, Point32


def utm_epsg_for(lat, lon):
    zone = int((lon + 180.0) / 6.0) + 1
    return (32600 if lat >= 0 else 32700) + zone


class PositionBroadcaster(Node):
    def __init__(self):
        super().__init__(
            'position_broadcaster',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )

        self.self_navsat_topic = self.get_parameter('self_navsat_topic').value
        self.self_heading_topic = self.get_parameter('self_heading_topic').value
        self.self_velocity_topic = self.get_parameter('self_velocity_topic').value
        publish_frequency = self.get_parameter('publish_frequency').value  # Hz

        peer_params = self.get_parameters_by_prefix('peers')
        self.peer_topics = {}  # name -> topic
        for key, param in peer_params.items():
            peer_name, prop = key.split('.', 1)
            if prop == 'topic':
                self.peer_topics[peer_name] = param.value

        self.self_utm = None  # (easting, northing)
        self.self_yaw_rad = None  # radians, clockwise from North (compass convention)
        self.peer_utm = {}  # name -> (easting, northing)

        self.velocity_publisher = self.create_publisher(TwistStamped, 'velocity', 10)
        self.peer_publisher = self.create_publisher(PolygonStamped, 'peer_detection', 10)

        self.self_navsat_subscriber = self.create_subscription(
            NavSatFix,
            self.self_navsat_topic,
            self.self_navsat_callback,
            10
        )
        self.self_navsat_subscriber  # prevent unused variable warning

        self.self_heading_subscriber = self.create_subscription(
            Float64,
            self.self_heading_topic,
            self.self_heading_callback,
            10
        )
        self.self_heading_subscriber  # prevent unused variable warning

        self.self_velocity_subscriber = self.create_subscription(
            TwistStamped,
            self.self_velocity_topic,
            self.self_velocity_callback,
            10
        )
        self.self_velocity_subscriber  # prevent unused variable warning

        self.peer_subscribers = {}
        for name, topic in self.peer_topics.items():
            subscriber = self.create_subscription(
                NavSatFix,
                topic,
                lambda msg, name=name: self.peer_navsat_callback(msg, name),
                10
            )
            self.peer_subscribers[name] = subscriber
            self.get_logger().info(f"Subscribed to peer [{name}] on topic: {topic}")

        self.timer = self.create_timer(1.0 / publish_frequency, self.timer_callback)

    def self_navsat_callback(self, msg):
        easting, northing, zone_number, zone_letter = utm.from_latlon(msg.latitude, msg.longitude)
        self.self_utm = (easting, northing)

    def self_heading_callback(self, msg):
        self.self_yaw_rad = math.radians(msg.data)

    def self_velocity_callback(self, msg):
        # mavros/local_position/velocity_body is already body-relative
        # (forward/left speed, body angular rates) -- relay only the
        # forward/lateral speed and yaw rate the directional derivative
        # actually needs, dropping the rest.
        relay = TwistStamped()
        relay.header = msg.header
        relay.twist.linear.x = msg.twist.linear.x
        relay.twist.linear.y = msg.twist.linear.y
        relay.twist.angular.z = msg.twist.angular.z
        self.velocity_publisher.publish(relay)

    def peer_navsat_callback(self, msg, name):
        easting, northing, zone_number, zone_letter = utm.from_latlon(msg.latitude, msg.longitude)
        self.peer_utm[name] = (easting, northing)

    def timer_callback(self):
        now = self.get_clock().now().to_msg()

        polygon_msg = PolygonStamped()
        polygon_msg.header.stamp = now
        polygon_msg.header.frame_id = 'base_link'

        if self.self_utm is not None and self.self_yaw_rad is not None:
            yaw = self.self_yaw_rad
            for peer_e, peer_n in self.peer_utm.values():
                delta_e = peer_e - self.self_utm[0]
                delta_n = peer_n - self.self_utm[1]

                # Compass yaw (0 = North, clockwise) rotated into the robot's
                # own x-forward/y-left body frame, so this lines up with the
                # frame the camera-based peer_detector used to publish in.
                x_body = delta_e * math.sin(yaw) + delta_n * math.cos(yaw)
                y_body = -delta_e * math.cos(yaw) + delta_n * math.sin(yaw)

                point = Point32()
                point.x = float(x_body)
                point.y = float(y_body)
                point.z = 0.0
                polygon_msg.polygon.points.append(point)

        self.peer_publisher.publish(polygon_msg)


def main(args=None):
    rclpy.init(args=args)
    position_broadcaster = PositionBroadcaster()
    rclpy.spin(position_broadcaster)
    position_broadcaster.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
