import rclpy
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image
from ntcore import NetworkTableInstance
from cscore import CameraServer
from cv_bridge import CvBridge

def main():
    rsvideo_cs = CameraServer.putVideo("Camera", 640, 480)
    bridge = CvBridge()
    try:
        rclpy.init()
        node = rclpy.create_node("robot_bridge_node")
        rsvideo_sub = node.create_subscription(
            Image, "video",
            lambda img: rsvideo_cs.putFrame(bridge.imgmsg_to_cv2(img, "bgr8")), 10
        )
        node.get_logger().info("started listening for video")
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info("shutdown requested")
    finally:
        node.get_logger().info("shutdown cleanly")

if __name__ == '__main__':
    main()
