#!/usr/bin/env python3
#test
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import time

_COLORMAPS = {
    'JET': cv2.COLORMAP_JET,
    'INFERNO': cv2.COLORMAP_INFERNO,
    'TURBO': cv2.COLORMAP_TURBO,
    'VIRIDIS': cv2.COLORMAP_VIRIDIS,
}

class ThermalColour(Node):
    def __init__(self):
        super().__init__('thermal_colour')
        self.bridge = CvBridge()

        # ----- Parameters you can override in launch -----
        self.declare_parameter('scale_factor', 0.01)     # counts -> Kelvin multiplier
        self.declare_parameter('min_k', 285.0)           # fallback lo (≈12 °C)
        self.declare_parameter('max_k', 305.0)           # fallback hi (≈32 °C)
        self.declare_parameter('auto_range', True)       # use percentiles by default
        self.declare_parameter('auto_low_pct', 2.0)
        self.declare_parameter('auto_high_pct', 98.0)
        self.declare_parameter('colormap', 'JET')
        self.declare_parameter('log_stats', True)
        self.declare_parameter('log_period_s', 1.0)

        self.scale_factor  = float(self.get_parameter('scale_factor').value)
        self.min_k         = float(self.get_parameter('min_k').value)
        self.max_k         = float(self.get_parameter('max_k').value)
        self.auto_range    = bool(self.get_parameter('auto_range').value)
        self.auto_low_pct  = float(self.get_parameter('auto_low_pct').value)
        self.auto_high_pct = float(self.get_parameter('auto_high_pct').value)
        self.colormap_name = str(self.get_parameter('colormap').value).upper()
        self.log_stats     = bool(self.get_parameter('log_stats').value)
        self.log_period_s  = float(self.get_parameter('log_period_s').value)

        if self.colormap_name not in _COLORMAPS:
            self.get_logger().warn(f"Unknown colormap '{self.colormap_name}', defaulting to JET")
            self.colormap_name = 'JET'
        self.colormap = _COLORMAPS[self.colormap_name]

        self.sub = self.create_subscription(Image, '/camera/thermal/image', self._cb, 10)
        self.pub = self.create_publisher(Image, '/camera/thermal/image_colour', 10)

        self._last_log = 0.0
        self.get_logger().info(
            f"ThermalColour started | cmap={self.colormap_name} | "
            f"scale={self.scale_factor} | fallback=[{self.min_k:.2f},{self.max_k:.2f}] K | "
            f"auto_range={self.auto_range} ({self.auto_low_pct}-{self.auto_high_pct} pct)"
        )

    def _cb(self, msg: Image):
        try:
            enc = msg.encoding.lower()
            img_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # counts -> Kelvin
            img_k = img_raw.astype(np.float32) * self.scale_factor

            # Decide mapping range
            if self.auto_range:
                # ---- ignore zeros so they don't crush the histogram ----
                valid = img_k[img_k > 0.0]
                if valid.size >= 16:
                    lo = float(np.percentile(valid, self.auto_low_pct))
                    hi = float(np.percentile(valid, self.auto_high_pct))
                    if hi - lo < 1e-6:
                        lo, hi = self.min_k, self.max_k
                else:
                    lo, hi = self.min_k, self.max_k
            else:
                lo, hi = self.min_k, self.max_k

            # Normalise -> 0..1 -> 8-bit
            img_norm = (img_k - lo) / max(hi - lo, 1e-6)
            img_norm = np.clip(img_norm, 0.0, 1.0)
            img_u8 = (img_norm * 255.0).astype(np.uint8)
            coloured = cv2.applyColorMap(img_u8, self.colormap)

            # Periodic logs
            now = time.time()
            if self.log_stats and (now - self._last_log) >= self.log_period_s:
                kmin, kmax = float(img_k.min()), float(img_k.max())
                self.get_logger().info(
                    f"enc={enc}, K[min,max]=({kmin:.2f},{kmax:.2f}) "
                    f"({kmin-273.15:.2f}°C, {kmax-273.15:.2f}°C) | map_range=[{lo:.2f},{hi:.2f}] K"
                )
                self._last_log = now

            out = self.bridge.cv2_to_imgmsg(coloured, encoding='bgr8')
            out.header = msg.header
            self.pub.publish(out)

        except Exception as e:
            self.get_logger().warn(f'Colourise error: {e}')

def main():
    rclpy.init()
    node = ThermalColour()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
