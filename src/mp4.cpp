#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <camera_calibration_parsers/parse.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <fstream>
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
        node->create_publisher<sensor_msgs::msg::Image>("/mp4/image_raw", 10);
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
    int index = video_file.find_last_of("//");
    std::cout << "video_file: " << video_file << std::endl;
    std::string base_path = video_file.substr(0, index);
    std::string base_filename = video_file.substr(index + 1);
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
    std::string camera_name = "mp4";
    sensor_msgs::msg::CameraInfo camera_info_;
    camera_calibration_parsers::readCalibration( camera_info_url, camera_name, camera_info_);

    // TF2 buffer and listener
    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // Open file to write camera positions
    std::ofstream pose_file_(base_path + "/camera_positions.txt");
    if (!pose_file_.is_open()) {
        RCLCPP_ERROR(node->get_logger(), "Error opening camera_positions.txt");
    }
    else {
        RCLCPP_INFO(node->get_logger(), "Camera positions will be written to %s", (base_path + "camera_positions.txt").c_str());
    }

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
            camera_info_.header = header;

            // Get the transform from camera to map
            geometry_msgs::msg::TransformStamped transform_stamped;
            std::string image_name = "image_" + std::to_string(frame_count) + ".png";
            try {
                transform_stamped = tf_buffer.lookupTransform("map", frame_id, tf2::TimePointZero);
                pose_file_  << frame_count << " ";
                pose_file_  << transform_stamped.transform.rotation.w << " " 
                            << transform_stamped.transform.rotation.x << " "
                            << transform_stamped.transform.rotation.y << " " 
                            << transform_stamped.transform.rotation.z << " ";
                pose_file_  << transform_stamped.transform.translation.x << " " 
                            << transform_stamped.transform.translation.y << " " 
                            << transform_stamped.transform.translation.z << " ";
                pose_file_  << "mp4" << " " << image_name << "\n";
                pose_file_  << "\n"; // Leave blank line between frames
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(node->get_logger(), "Could not transform %s to map: %s", frame_id.c_str(), ex.what());
            }
        } 
        catch (cv_bridge::Exception& e) 
        {
            RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
            return 1;
        }
        frame_count++;

        image_publisher->publish(*image_msg);
        camera_info_publisher->publish(camera_info_);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    pose_file_.close();
    rclcpp::shutdown();
    return 0;
}