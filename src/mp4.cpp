#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sstream>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mp4_to_ros2");

    std::string video_file, frame_id;
    node->declare_parameter("video_file", video_file);
    node->declare_parameter("frame_id", frame_id);

    node->get_parameter("video_file", video_file);
    node->get_parameter("frame_id", frame_id);

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher =
        node->create_publisher<sensor_msgs::msg::Image>("/mp4/video_frames", 10);

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

    cv::Mat frame;
    sensor_msgs::msg::Image::SharedPtr msg;
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
            msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        } 
        catch (cv_bridge::Exception& e) 
        {
            RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
            return 1;
        }
        frame_count ++;

        publisher->publish(*msg);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}