# mp4-to-ros2

Simple ROS2 node to read in an mp4 with OpenCV and publish frames as image messages. If the mp4 filename is a date-time, it will publish images with timestamps beginning from this timestamp using FPS to estimate each frame stamp. Otherwise, it starts at the timestamp from the latest IMU message.

## Data cleaning

If the video was recorded automatically at arming, great! Otherwise, trim the video to match the moment the props arm. To reduce the framerate, downsample the video prior to launch with:
```
ffmpeg -i in.mp4 -vf fps=5 out.mp4
```