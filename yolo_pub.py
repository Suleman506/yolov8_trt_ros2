#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.node import QoSProfile
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
from std_msgs.msg import String

from yolov8_det_vid import YoLov8TRT

def plot_one_box(x, img, color=None, label=None, line_thickness=None):
    tl = (
            line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1
    )  # line/font thickness
    color = (255, 0 ,0)#color or [random.randint(0, 255) for _ in range(3)]
    c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
    cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
    if label:
        tf = max(tl - 1, 1)  # font thickness
        t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
        cv2.putText(
            img,
            label,
            (c1[0], c1[1] - 2),
            0,
            tl / 3,
            [225, 255, 255],
            thickness=tf,
            lineType=cv2.LINE_AA,
        )

class YOLOv8DetectorNode(Node):
    def __init__(self, engine_file_path, library):
        super().__init__('yolov8_detector')

        # QoS profile for reliable, high-throughput image data
        qos_profile = QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT # Add reliability policy                      
        )

        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            qos_profile
        )


        self.result_boxes_publisher = self.create_publisher(String, '/yolov8/result', qos_profile)

        self.image = np.empty(shape=[1])
        self.model = YoLov8TRT(library, engine_file_path)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        self.image= self.bridge.imgmsg_to_cv2(msg, "bgr8")
        result_boxes, result_scores, result_classid = self.model.Inference(self.image)



        result_data = []
        for j in range(len(result_boxes)):
            category = self.model.categories[int(result_classid[j])]
            box = result_boxes[j]
            score = "{:.2f}".format(result_scores[j])
            result_data.append(f"{category}:{score}:{box}")

            plot_one_box(box, self.image, label=f"{category}:{score}",)

        result_msg = String()
        result_msg.data = ";".join(result_data)
        self.result_boxes_publisher.publish(result_msg)
        
        cv2.imshow('Yolov8', self.image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    # Provide paths to the YOLOv8 engine and library
    engine_file_path = "/home/wego/yolov8_trt_ros2/build/yolov8n.engine"
    library = "/home/wego/yolov8_trt_ros2/build/libmyplugins.so"

    yolov8_detector_node = YOLOv8DetectorNode(engine_file_path, library)

    rclpy.spin(yolov8_detector_node)

    yolov8_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()