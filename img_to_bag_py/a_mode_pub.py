"""
Publisher for A-mode OCT .txt images.

Author: 
    - John Dallas Cast, Apr 12, 2023
    - johndallascast.com

Adapted from: https://automaticaddison.com/getting-started-with-opencv-in-ros-2-foxy-fitzroy-python/ 
"""

import os
import rclpy
import yaml
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images


class ImagePublisher(Node):

    def __init__(self):
        super().__init__('image_publisher')

        package_share_directory = get_package_share_directory('img_to_bag_py')
        self.config_file = None
        with open(os.path.join(package_share_directory, 'config', 'config.yaml'), 'r') as f:
            self.config_file = yaml.safe_load(f) 
        self.input_dir = self.config_file['a_mode_publisher']['input_dir']
        self.topic = self.config_file['a_mode_publisher']['topic']

        print("Input Directory: {}".format(self.input_dir))

        self.publisher_ = self.create_publisher(Image, self.topic, 10)
        self.i = 0
        
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        self.publish()

    def publish(self):
        # iterate over files in that input_dir
        idx = 0
        for filename in os.listdir(self.input_dir):
            f = os.path.join(self.input_dir, filename)

            # checking if it is a file
            if os.path.isfile(f):
                print(f)
                with open(f) as img_f:
                    img = []
                    for line in img_f:
                        nums = [int(i) for i in line.split()]
                        if (len(nums) > 0): # some text files have empty lines
                            img.append(nums)
                    img = np.vstack(img).astype(np.uint8) / 255.0
                    
                    print("image shape: {}".format(img.shape))
                    print("image max: {}".format(np.max(img)))
                    print("image min: {}".format(np.min(img)))
                    
                    # The 'cv2_to_imgmsg' method converts an OpenCV
                    # image to a ROS 2 image message
                    img_msg = None
                    try:
                        img_msg = self.br.cv2_to_imgmsg(img, encoding="passthrough")
                        #print(ros_img)
                    except CvBridgeError as e:
                        print(e)
        
                    self.publisher_.publish(img_msg)
                idx+=1


def main(args=None):
    rclpy.init(args=args)

    image_publisher = ImagePublisher()

    rclpy.spin(image_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
