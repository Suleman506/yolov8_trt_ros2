#!/usr/bin/env python3

import cv2
import numpy as np
from tensorflow import keras
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile

class Tracker(Node):
    def __init__(self):
        super().__init__('tracker_publisher')
        self.ros_pub_twist = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ros_pub_image = self.create_publisher(Image, '/tracking', 10)

        self.bridge = CvBridge()
        self.initModel()

        qos_profile = QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/camera/color/image_raw', self.camera_CB, qos_profile)

    def initModel(self):
        input1 = keras.layers.Input(
            shape=(
                110,
                300,
                3,
            )
        )

        conv1 = keras.layers.Conv2D(filters=16, kernel_size=(3, 3), strides=(2, 2), padding="same", activation="swish")(input1)
        norm1 = keras.layers.BatchNormalization()(conv1)
        pool1 = keras.layers.MaxPooling2D(pool_size=(3, 3), strides=(2, 2))(norm1)
        conv2 = keras.layers.Conv2D(filters=32, kernel_size=(3, 3), strides=(2, 2), padding="same", activation="swish")(pool1)
        norm2 = keras.layers.BatchNormalization()(conv2)
        conv3 = keras.layers.Conv2D(filters=32, kernel_size=(3, 3), strides=(1, 1), padding="same", activation="swish")(norm2)
        norm3 = keras.layers.BatchNormalization()(conv3)
        add1 = keras.layers.Add()([norm2, norm3])
        conv4 = keras.layers.Conv2D(filters=64, kernel_size=(3, 3), strides=(2, 2), padding="same", activation="swish")(add1)
        norm4 = keras.layers.BatchNormalization()(conv4)
        conv5 = keras.layers.Conv2D(filters=64, kernel_size=(3, 3), strides=(1, 1), padding="same", activation="swish")(norm4)
        norm5 = keras.layers.BatchNormalization()(conv5)
        add2 = keras.layers.Add()([norm4, norm5])
        conv6 = keras.layers.Conv2D(filters=128, kernel_size=(3, 3), strides=(2, 2), padding="same", activation="swish")(add2)
        norm6 = keras.layers.BatchNormalization()(conv6)
        conv7 = keras.layers.Conv2D(filters=128, kernel_size=(3, 3), strides=(1, 1), padding="same", activation="swish")(norm6)
        norm7 = keras.layers.BatchNormalization()(conv7)
        add3 = keras.layers.Add()([norm6, norm7])
        conv8 = keras.layers.Conv2D(filters=256, kernel_size=(3, 3), strides=(2, 2), padding="same", activation="swish")(add3)
        norm7 = keras.layers.BatchNormalization()(conv8)
        conv9 = keras.layers.Conv2D(filters=512, kernel_size=(3, 3), strides=(2, 2), padding="same", activation="swish")(norm7)
        norm8 = keras.layers.BatchNormalization()(conv9)
        flat1 = keras.layers.Flatten()(norm8)
        dense1 = keras.layers.Dense(128, activation="swish")(flat1)
        norm9 = keras.layers.BatchNormalization()(dense1)
        dense2 = keras.layers.Dense(64, activation="swish")(norm9)
        norm10 = keras.layers.BatchNormalization()(dense2)
        dense3 = keras.layers.Dense(64, activation="swish")(norm10)
        norm11 = keras.layers.BatchNormalization()(dense3)
        dense4 = keras.layers.Dense(2, activation="tanh")(norm11)
        self.model = keras.models.Model(inputs=input1, outputs=dense4)
        self.model.load_weights("/home/wego/tensorrtx/yolov8/CHS_Track.h5")

    def publish_to_ros(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.ros_pub_twist.publish(twist)

    def camera_CB(self, data):
        origin_img = self.bridge.imgmsg_to_cv2(data)
        resize_img = cv2.resize(origin_img, (300, 300), cv2.INTER_LINEAR)
        crop_img = resize_img[190:300, :]
        
        x, y = self.model(np.array([crop_img]).astype(np.float32)).numpy()[0]

        linear_x = 0.25
        angular_z = -(x - 0.55) * 1.04
        self.publish_to_ros(linear_x, angular_z)
        # Example code to publish the processed image
        cX = int(300 * x)

        # Draw a circle on the resized frame
        out_img = resize_img.copy()
        out_img = cv2.circle(out_img, (cX, 190), 6, (255, 0, 0), 2)
        out_img_rgb = cv2.cvtColor(out_img, cv2.COLOR_BGR2RGB)
        resize_img_msg = self.bridge.cv2_to_imgmsg(out_img_rgb)
        self.ros_pub_image.publish(resize_img_msg)


def main(args=None):
    rclpy.init(args=args)
    tracker = Tracker()
    rclpy.spin(tracker)
    tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
