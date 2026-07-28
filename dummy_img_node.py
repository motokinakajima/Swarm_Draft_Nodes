import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class DummyGradientPublisher(Node):
    def __init__(self):
        super().__init__('dummy_gradient_publisher')
        
        # app.py と同じデフォルトトピック名、QoS (履歴サイズ10) で設定
        self.publisher_ = self.create_publisher(Image, '/camera1/image_raw', 10)
        
        # 30 FPSでパブリッシュ
        self.timer_period = 1.0 / 30.0  
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.bridge = CvBridge()
        self.start_time = time.time()
        
        # 解像度設定
        self.width = 320
        self.height = 240
        
        # 波打つグラデーション用の座標グリッドを事前計算（処理負荷軽減のため）
        x = np.linspace(0, 4 * np.pi, self.width)
        y = np.linspace(0, 4 * np.pi, self.height)
        self.X, self.Y = np.meshgrid(x, y)
        
        self.get_logger().info("Dummy Gradient Publisher started at /camera1/image_raw")

    def timer_callback(self):
        # 経過時間
        t = time.time() - self.start_time
        
        # --- 1. 背景の動的グラデーション生成 ---
        # サイン波・コサイン波を使って時間(t)とともに波打つ色を作る
        # R: X軸方向に波打つ, G: Y軸方向に波打つ, B: 斜め方向に波打つ
        r = (np.sin(self.X - t * 2.0) * 127 + 128).astype(np.uint8)
        g = (np.cos(self.Y + t * 1.5) * 127 + 128).astype(np.uint8)
        b = (np.sin(self.X + self.Y - t * 3.0) * 127 + 128).astype(np.uint8)
        
        # BGR画像として合成
        cv_image = cv2.merge([b, g, r])
        
        # --- 2. ターゲット(動く赤い円)の描画 ---
        # 画面内をリサージュ図形のように動き回る軌道
        target_x = int(self.width / 2 + np.sin(t * 1.2) * 100)
        target_y = int(self.height / 2 + np.cos(t * 0.9) * 80)
        
        # ターゲットとして純粋な赤(BGR: 0, 0, 255)の円を描く
        cv2.circle(cv_image, (target_x, target_y), 15, (0, 0, 255), -1)
        
        # (おまけ) ターゲットの中心に少しノイズ（ハイライト）を入れる
        cv2.circle(cv_image, (target_x - 3, target_y - 3), 4, (200, 200, 255), -1)

        # --- 3. ROS 2 Imageメッセージへの変換とPublish ---
        msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "dummy_camera_frame"
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DummyGradientPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping Dummy Publisher...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()