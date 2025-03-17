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
#include "messages_88/srv/geopoint.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <exiv2/exiv2.hpp>

using namespace std::chrono_literals;

// Function to embed GPS EXIF data
void embedGPSData(const std::string& imagePath, double latitude, double longitude, double altitude, rclcpp::Time timestamp) {
    try {
        Exiv2::Image::AutoPtr image = Exiv2::ImageFactory::open(imagePath);
        if (!image.get()) {
            std::cerr << "Failed to open image for EXIF writing: " << imagePath << std::endl;
            return;
        }
        image->readMetadata();
        Exiv2::ExifData &exifData = image->exifData();

        // Convert ROS2 Time to human-readable format
        std::time_t raw_time = timestamp.seconds();
        struct tm *time_info = std::localtime(&raw_time);
        char time_str[20]; // "YYYY:MM:DD HH:MM:SS"
        strftime(time_str, sizeof(time_str), "%Y:%m:%d %H:%M:%S", time_info);

        // Write Date/Time to EXIF
        exifData["Exif.Photo.DateTimeOriginal"] = std::string(time_str);

        // Write GPS data
        exifData["Exif.GPSInfo.GPSLatitudeRef"] = latitude >= 0 ? "N" : "S";
        exifData["Exif.GPSInfo.GPSLatitude"] = Exiv2::Rational(abs(latitude), 1);
        exifData["Exif.GPSInfo.GPSLongitudeRef"] = longitude >= 0 ? "E" : "W";
        exifData["Exif.GPSInfo.GPSLongitude"] = Exiv2::Rational(abs(longitude), 1);
        exifData["Exif.GPSInfo.GPSAltitudeRef"] = 0; // 0 = above sea level, 1 = below sea level
        exifData["Exif.GPSInfo.GPSAltitude"] = Exiv2::Rational(static_cast<int>(altitude * 100), 100);

        // Write DOP (Dilution of Precision)
        double dop = 5.0; // Default value
        exifData["Exif.GPSInfo.GPSDOP"] = Exiv2::Rational(static_cast<int>(dop * 100), 100);

        image->setExifData(exifData);
        image->writeMetadata();
    } catch (const Exiv2::AnyError& e) {
        std::cerr << "EXIF writing error: " << e.what() << std::endl;
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mp4_to_ros2");

    std::string video_file, frame_id, camera_info_file;
    bool bag_sync = true, save_splat_images = false;
    int splat_fps;
    node->declare_parameter("video_file", video_file);
    node->declare_parameter("frame_id", frame_id);
    node->declare_parameter("bag_sync", bag_sync);
    node->declare_parameter("camera_info_file", camera_info_file);
    node->declare_parameter("save_splat_images", save_splat_images);
    node->declare_parameter("splat_fps", splat_fps);

    node->get_parameter("video_file", video_file);
    node->get_parameter("frame_id", frame_id);
    node->get_parameter("bag_sync", bag_sync);
    node->get_parameter("camera_info_file", camera_info_file);
    node->get_parameter("save_splat_images", save_splat_images);
    node->get_parameter("splat_fps", splat_fps);

    rclcpp::Client<messages_88::srv::Geopoint>::SharedPtr geo_client_ = node->create_client<messages_88::srv::Geopoint>("/task_manager/slam2geo");
    // Ensure service client is available
    if (!geo_client_->wait_for_service(std::chrono::seconds(10))) {
        RCLCPP_ERROR(node->get_logger(), "Geo service unavailable, skipping EXIF.");
        return 1;
    }

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
    }

    // Load camera info
    // std::string camera_info_url = camera_info_file;
    std::string camera_name = "mp4";
    sensor_msgs::msg::CameraInfo camera_info_;
    camera_calibration_parsers::readCalibration(camera_info_file, camera_name, camera_info_);

    // TF2 buffer and listener
    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    std::string outputDir = base_path + "/camera";
    if (!std::filesystem::exists(outputDir)) {
        std::filesystem::create_directory(outputDir);
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

            if (!save_splat_images || frame_count % splat_fps != 0) {
                frame_count++;

                image_publisher->publish(*image_msg);
                camera_info_publisher->publish(camera_info_);
                rclcpp::spin_some(node);
                loop_rate.sleep();
                continue;
            }

            // Get the transform from camera to map
            geometry_msgs::msg::TransformStamped transform_stamped;
            try {
                transform_stamped = tf_buffer.lookupTransform("map", frame_id, tf2::TimePointZero);
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(node->get_logger(), "Could not transform %s to map: %s", frame_id.c_str(), ex.what());
                continue;
            }

            // Get lat/long for image
            auto geo_request = std::make_shared<messages_88::srv::Geopoint::Request>(); 
            geo_request->slam_position.x = transform_stamped.transform.translation.x;
            geo_request->slam_position.y = transform_stamped.transform.translation.y;
            geo_request->slam_position.z = transform_stamped.transform.translation.z;
            auto geo_res = geo_client_->async_send_request(geo_request);
            double lat, lon, home_alt;
            if (rclcpp::spin_until_future_complete(node, geo_res, 1s) == rclcpp::FutureReturnCode::SUCCESS)
            {
                try
                {
                    auto response = geo_res.get();
                    lat = response->latitude;
                    lon = response->longitude;
                    home_alt = response->home_altitude;
                }
                catch (const std::exception &e)
                {
                    RCLCPP_INFO(node->get_logger(), "Geo service call failed, no lat/long available for EXIF data.");
                }
            
            } else {
                RCLCPP_ERROR(node->get_logger(), "Failed to get lat/long, no EXIF.");
                continue;
            }

            // Save image
            std::string image_name = outputDir + "/image_" + std::to_string(frame_count) + ".png";
            cv::imwrite(image_name, frame); // Save the frame

            double altitude = transform_stamped.transform.translation.z + home_alt;
            embedGPSData(image_name, lat, lon, altitude, start_time + t); // Embed GPS data
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

    rclcpp::shutdown();
    return 0;
}