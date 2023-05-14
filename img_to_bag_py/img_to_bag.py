"""
Convert directory of .png/.jpg images into bag file.

Author: 
    - John Dallas Cast, Apr 12, 2023
    - johndallascast.com
"""

import os
import cv2
import yaml
import rosbag2_py
import numpy as np

from rclpy.clock import Clock
from rclpy.duration import Duration
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images


def main(args=None):
    package_share_directory = get_package_share_directory('img_to_bag_py')
    config_file = None
    with open(os.path.join(package_share_directory, 'config', 'config.yaml'), 'r') as f:
        config_file = yaml.safe_load(f) 

    input_dir = config_file['img_to_bag']['input_dir']
    
    topic = config_file['img_to_bag']['topic']

    bag_name = config_file['img_to_bag']['bag']['name']
    bag_path = config_file['img_to_bag']['bag']['path']
    bag_file = os.path.join(bag_path, bag_name)

    print("Input Directory: {}".format(input_dir))
    print("Output File: {}".format(bag_path))

    # bag file writer
    writer = rosbag2_py.SequentialWriter()

    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    storage_options = rosbag2_py._storage.StorageOptions(
        uri=bag_file,
        storage_id='sqlite3')
    converter_options = rosbag2_py._storage.ConverterOptions('', '')
    writer.open(storage_options, converter_options)

    topic_info = rosbag2_py._storage.TopicMetadata(
        name=topic,
        type='sensor_msgs/msg/Image',
        serialization_format='cdr')
    writer.create_topic(topic_info)

    # iterate over files in that input_dir
    for filename in os.listdir(input_dir):
        f = os.path.join(input_dir, filename)

        # checking if it is a file
        if os.path.isfile(f):
            print(f)
            try:
                img = cv2.imread(f)

                print("image shape: {}".format(img.shape))
                print("image max: {}".format(np.max(img)))
                print("image min: {}".format(np.min(img)))
                
                # The 'cv2_to_imgmsg' method converts an OpenCV
                # image to a ROS 2 image message
                img_msg = br.cv2_to_imgmsg(img)

                time_stamp = Clock().now()
                writer.write(
                    topic,
                    serialize_message(img_msg),
                    time_stamp.nanoseconds)
                time_stamp += Duration(seconds=1)
            except cv2.error as e:
                print("CV2 error opening: {}".format(f))

if __name__ == '__main__':
    main()
