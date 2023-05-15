# img_to_bag_py
ROS2 utilities for creating and viewing bag files from images

# Installation
1. `mkdir -p ros2_ws/src`
2. `cd ros2_ws/src`
3. `git clone https://github.com/jdcast/img_to_bag_py.git`
4. `cd ..`
5. `colcon build && source install/setup.bash`

# Usage
## img_to_bag
### Convert directory of .png/.jpg images to ros2 bag file
1. configure the desired topic, input/output directories and filename under the key `img_to_bag` within *config/config.yaml* 
2. `ros2 run img_to_bag_py img_to_bag`

## a_mode_to_bag
### Convert directory of A-mode OCT image .txt files formatted like this file: [a_mode_oct_example_image.txt](https://github.com/jdcast/img_to_bag_py/files/11473319/1.txt)
1. configure the desired topic, input/output directories and filename under the key `a_mode_to_bag` within *config/config.yaml*
2. `ros2 run img_to_bag_py a_mode_to_bag`

## a_mode_pub
### Publish directory of A-mode OCT image .txt files formatted like this file: [a_mode_oct_example_image.txt](https://github.com/jdcast/img_to_bag_py/files/11473319/1.txt)
1. configure the desired topic and input directory under the key `a_mode_pub` within *config/config.yaml*
2. `ros2 run img_to_bag_py a_mode_pub --ros-args --remap /image_a_mode:=/image` (the topic remapping is only needed if using `img_sub` to view the published images)

## img_viewer
### Subscribes to `/image` topic and displays the published images using opencv
1. rerun an image bag file (e.g. `ros2 bag play /home/nightrider/Downloads/img_bag_a_mode/ --loop --remap /image_a_mode:=/image`) or start the `a_mode_pub` publisher
2. `ros2 run img_to_bag_py img_sub`
