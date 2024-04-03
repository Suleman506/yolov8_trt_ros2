#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile

class YOLOV8Subscriber(Node):
    def __init__(self):
        super().__init__('yolov8_subscriber')
        qos_profile = QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT
        )
        self.subscription = self.create_subscription(
            String,
            '/yolov8/result',
            self.callback,
            qos_profile
        )
        self.subscription

    def callback(self, msg):
        try:
            data = msg.data.split(';')
            for item in data:
                parts = item.split(':')
                class_label = parts[0]
                score = float(parts[1])
                bbox_str = parts[2]
                bbox = [float(coord) for coord in bbox_str.strip('[]').split()]
                bbox = [round(coord, 2) for coord in bbox]
                print("Class Label:", class_label)
                print("Score:", score)
                print("Bounding Box:", bbox)
                print()
        except IndexError:
            print("No objects found.")

def main(args=None):
    rclpy.init(args=args)
    yolov8_subscriber = YOLOV8Subscriber()
    rclpy.spin(yolov8_subscriber)
    yolov8_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
