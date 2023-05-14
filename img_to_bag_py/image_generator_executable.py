from rclpy.clock import Clock
from rclpy.duration import Duration
from rclpy.serialization import serialize_message
from example_interfaces.msg import Int32
from sensor_msgs.msg import Image

import rosbag2_py

import os
import cv2
import yaml
import numpy as np
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

def main(args=None):
    writer = rosbag2_py.SequentialWriter()

    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    storage_options = rosbag2_py._storage.StorageOptions(
        uri='img_synthetic_bag',
        storage_id='sqlite3')
    converter_options = rosbag2_py._storage.ConverterOptions('', '')
    writer.open(storage_options, converter_options)

    topic_info = rosbag2_py._storage.TopicMetadata(
        name='synthetic_image',
        type='sensor_msgs/msg/Image',
        serialization_format='cdr')
    writer.create_topic(topic_info)

    #directory = ament_index_python.get_resource('package', 'img_to_bag_py')#"share/ament_index/resource_index/packages/install/"
    package_share_directory = get_package_share_directory('img_to_bag_py')
    config_file = None
    with open(os.path.join(package_share_directory, 'config', 'config.yaml'), 'r') as f:
        config_file = yaml.safe_load(f) 
    directory = config_file['directory']
    img_h = config_file['image']['height']
    img_w = config_file['image']['width']

    print("Directory: {}".format(directory))

    # iterate over files in that directory
    idx = 0
    for filename in os.listdir(directory):
        f = os.path.join(directory, filename)
        # checking if it is a file
        if os.path.isfile(f):
            print(f)
            with open(f) as img_f:
                #img =np.zeros((img_h, img_w, 1), np.uint8)
                img = []
                for line in img_f:
                    nums = [int(i) for i in line.split()]
                    if (len(nums) > 0): # some text files have empty lines
                        img.append(nums)
                    #print(len(nums))
                    #nums = np.asarray(nums)/255.0
                    #if (idx == 0):
                    #    print(nums)
                #img = np.vstack(img).astype(np.uint8) / 255.0
                img = np.vstack(img).astype(np.uint8)
                print(img.shape)
                print(np.max(img))
                print(np.min(img))
                #print(type(img.tolist()[0][0]))
                
                #cv2.imshow("camera", img)
                #cv2.waitKey(1)
                
                # The 'cv2_to_imgmsg' method converts an OpenCV
                # image to a ROS 2 image message
                img_msg = br.cv2_to_imgmsg(img)
                #print(ros_img)

                time_stamp = Clock().now()
                Img = Image()
                Img.header.stamp = time_stamp.to_msg()
                Img.width = img.shape[1]
                Img.height = img.shape[0]
                Img.encoding = "rgb8"
                Img.header.frame_id = "camera"
                Img.data = img_msg.data#img.flatten().tolist() #img.tolist() #TODO: how to do this?

                writer.write(
                    'synthetic_image',
                    serialize_message(img_msg),
                    #serialize_message(Img),
                    time_stamp.nanoseconds)
                time_stamp += Duration(seconds=1)
            idx+=1

if __name__ == '__main__':
    main()
