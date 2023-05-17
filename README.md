# ros2_img_to_bag_py
ROS2 utilities for creating and viewing bag files from images

# Installation
1. `mkdir -p ros2_ws/src`
2. `cd ros2_ws/src`
3. `git clone https://github.com/jdcast/ros2_img_to_bag_py.git`
4. `cd ..`
5. `colcon build && source install/setup.bash`

# Usage
## img_to_bag
### Convert directory of .png/.jpg images to ros2 bag file
1. configure the desired topic, input/output directories and filename under the key `img_to_bag` within *config/config.yaml* 
2. `cd ros2_ws`
3. `colcon build && source install/setup.bash`
4. `ros2 run img_to_bag_py img_to_bag`

## a_mode_to_bag
### Convert directory of A-mode OCT image .txt files formatted like this file: [a_mode_oct_example_image.txt](https://github.com/jdcast/img_to_bag_py/files/11473319/1.txt)
1. configure the desired topic, input/output directories and filename under the key `a_mode_to_bag` within *config/config.yaml*
2. `cd ros2_ws`
3. `colcon build && source install/setup.bash`
3. `ros2 run img_to_bag_py a_mode_to_bag`

## a_mode_pub
### Publish directory of A-mode OCT image .txt files formatted like this file: [a_mode_oct_example_image.txt](https://github.com/jdcast/img_to_bag_py/files/11473319/1.txt)
1. configure the desired topic and input directory under the key `a_mode_pub` within *config/config.yaml*
2. `cd ros2_ws`
3. `colcon build && source install/setup.bash`
3. `ros2 run img_to_bag_py a_mode_pub --ros-args --remap /image_a_mode:=/image` (the topic remapping is only needed if using `img_sub` to view the published images)

## img_viewer
### Subscribes to `/image` topic and displays the published images using opencv
1. rerun an image bag file (e.g. `ros2 bag play /home/nightrider/Downloads/img_bag_a_mode/ --loop --remap /image_a_mode:=/image`) or start the `a_mode_pub` publisher
2. `cd ros2_ws`
3. `colcon build && source install/setup.bash`
3. `ros2 run img_to_bag_py img_sub`

# Resources
1. https://docs.ros.org/en/galactic/Tutorials/Advanced/Recording-A-Bag-From-Your-Own-Node-Py.html#write-the-python-node
2. https://docs.ros.org/en/foxy/Releases/Release-Galactic-Geochelone.html#rosbag2-new-features
3. https://answers.ros.org/question/355764/ros2how-to-write-data-to-rosbag2-in-ros2-foxy/
4. https://answers.ros.org/question/11537/creating-a-bag-file-out-of-a-image-sequence/
5. https://gist.github.com/Sarath18/7c48b6f2e667bf6dab1b12f419cab397
6. https://github.com/ros2/rosbag2/blob/galactic/rosbag2_tests/test/rosbag2_tests/test_rosbag2_cpp_api.cpp
7. https://ternaris.gitlab.io/rosbags/topics/serde.html
8. https://en.wikipedia.org/wiki/Common_Data_Representation
9. https://automaticaddison.com/getting-started-with-opencv-in-ros-2-foxy-fitzroy-python/
10. http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
11. https://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
12. https://automaticaddison.com/getting-started-with-opencv-in-ros-2-foxy-fitzroy-python/
