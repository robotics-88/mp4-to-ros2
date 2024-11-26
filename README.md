# mp4-to-ros2

Simple ROS2 node to read in an mp4 with OpenCV and publish frames as image messages. If the mp4 filename is a date-time, it will publish images with timestamps beginning from this timestamp using FPS to estimate each frame stamp. Otherwise, it starts at the current time.