import rclpy
import os
import yaml
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images


class ImagePublisher(Node):

    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'image', 10)
        self.i = 0
        
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        self.publish()

    def publish(self):
        package_share_directory = get_package_share_directory('img_to_bag_py')
        config_file = None
        with open(os.path.join(package_share_directory, 'config', 'config.yaml'), 'r') as f:
            config_file = yaml.safe_load(f) 
        directory = config_file['directory']

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
                    img = np.vstack(img).astype(np.uint8) / 255.0
                    #img = np.vstack(img).astype(np.uint8)
                    print(img.shape)
                    print(np.max(img))
                    print(np.min(img))
                    
                    #cv2.imshow("camera", img)
                    #cv2.waitKey(1)
                    
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

    #image_publisher.publish()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
