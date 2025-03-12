#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <sstream>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mp4_to_ros2");

    std::string video_file, frame_id, camera_info_file;
    bool bag_sync = true;
    node->declare_parameter("video_file", video_file);
    node->declare_parameter("frame_id", frame_id);
    node->declare_parameter("bag_sync", bag_sync);
    node->declare_parameter("camera_info_file", camera_info_file);

    node->get_parameter("video_file", video_file);
    node->get_parameter("frame_id", frame_id);
    node->get_parameter("bag_sync", bag_sync);
    node->get_parameter("camera_info_file", camera_info_file);

    if (bag_sync) {
        RCLCPP_INFO(node->get_logger(), "Waiting for IMU message to start video publishing");
        bool imu_received = false;
        auto sensor_qos = rclcpp::SensorDataQoS();
        auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>("/mavros/imu/data", sensor_qos, [&imu_received](sensor_msgs::msg::Imu::SharedPtr) {
            RCLCPP_INFO(rclcpp::get_logger("simple_wait_for_message"), "Received imu");
            imu_received = true;
        });

        // Block execution until a message is received
        rclcpp::Rate rate(10);  // 10 Hz loop rate
        while (rclcpp::ok() && !imu_received) {
            rclcpp::spin_some(node);
            rate.sleep();
        }
        imu_sub.reset();
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher =
        node->create_publisher<sensor_msgs::msg::Image>("/mp4/video_frames", 10);
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher =
        node->create_publisher<sensor_msgs::msg::CameraInfo>("/mp4/camera_info", 10);

    cv::VideoCapture cap(video_file); 
    if (!cap.isOpened())
    {
        RCLCPP_ERROR(node->get_logger(), "Error opening video file");
        return 1;
    }
    double fps = cap.get(cv::CAP_PROP_FPS);
    int frame_count = 0;
    rclcpp::Time start_time = node->get_clock()->now();

    // Get time from video filename if possible
    std::string base_filename = video_file.substr(video_file.find_last_of("//") + 1);
    std::string::size_type const p(base_filename.find_last_of('.'));
    base_filename = base_filename.substr(0, p);
    std::tm tm = {}; 
    std::istringstream ss(base_filename);
    // Parse the string into the tm struct
    ss >> std::get_time(&tm, "%Y-%m-%d_%H-%M-%S"); 
    if (ss.fail()) {
        std::cerr << "Error parsing date/time string, frame timestamps will start at ros time now()" << std::endl;
    }
    else {
        // Convert tm to time_t
        std::time_t time = std::mktime(&tm);
        start_time = rclcpp::Time(time, 0); // Need to get fractional seconds storing
        std::cout << "starting time was " << start_time.seconds() << " sec" << std::endl;
    }

    // Load camera info
    std::string camera_info_url = "file://" + camera_info_file;
    camera_info_manager::CameraInfoManager cam_info_manager(node.get(), "mp4", camera_info_url);
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>(cam_info_manager.getCameraInfo());

    cv::Mat frame;
    sensor_msgs::msg::Image::SharedPtr image_msg;
    rclcpp::WallRate loop_rate(fps); // Adjust the rate as needed

    while (rclcpp::ok() && cap.read(frame))
    {
        try 
        {
            std_msgs::msg::Header header;
            header.frame_id = frame_id;
            double timestamp_seconds = frame_count / fps;
            double timestamp_ns = 1e9 * timestamp_seconds;
            int nsec = floor(timestamp_ns);
            int sec = floor(timestamp_seconds);
            rclcpp::Duration t(sec, nsec);
            header.stamp = start_time + t;
            image_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
            camera_info_msg->header = header;
        } 
        catch (cv_bridge::Exception& e) 
        {
            RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
            return 1;
        }
        frame_count++;

        image_publisher->publish(*image_msg);
        camera_info_publisher->publish(*camera_info_msg);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}