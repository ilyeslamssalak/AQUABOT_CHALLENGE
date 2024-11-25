#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String  # Import String message type

import cv2

# Package to convert between ROS and OpenCV Images
from cv_bridge import CvBridge 

class OpenCvDecoder(Node):

    def __init__(self):
        super().__init__('opencv_decoder_node')
        
        # Subscriber: listens to the topic with camera images
        self.subscriber = self.create_subscription(
            Image, 
            '/aquabot/sensors/cameras/main_camera_sensor/image_raw', 
            self.image_callback, 
            10
        )

        # Publisher: publishes decoded data to a topic
        self.publisher = self.create_publisher(String, '/navigator/qr_code_data', 10)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        # Used to decode QR codes
        self.qr_decoder = cv2.QRCodeDetector()

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(msg)

        # Decode 
        try :
            data, bbox, rectifiedImage = self.qr_decoder.detectAndDecode(current_frame)
        except :
            data = ''

        if len(data) > 0:
            # Publish the decoded data to the topic
            decoded_message = String()
            decoded_message.data = data
            self.publisher.publish(decoded_message)


def main(args=None):
    rclpy.init(args=args)

    opencv_decoder_node = OpenCvDecoder()

    rclpy.spin(opencv_decoder_node)

    # Destroy the node explicitly
    opencv_decoder_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
