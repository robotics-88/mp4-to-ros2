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
#include <vector>
#include <exiv2/exiv2.hpp>

using namespace std::chrono_literals;

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

        // Write GPS Latitude
        // Write latitude and latitude reference
        // Convert decimal latitude to DMS as a vector of Exiv2::Rational values
        std::vector<Exiv2::Rational> latValues = {
            Exiv2::Rational(static_cast<int>(std::fabs(latitude)), 1),
            Exiv2::Rational(static_cast<int>((std::fabs(latitude) - static_cast<int>(std::fabs(latitude))) * 60), 1),
            Exiv2::Rational(
                static_cast<int>(
                    (
                        ((std::fabs(latitude) - static_cast<int>(std::fabs(latitude))) * 60
                         - static_cast<int>((std::fabs(latitude) - static_cast<int>(std::fabs(latitude))) * 60))
                        * 60 * 1000
                    )
                ),
                1000
            )
        };
        // Create an Exiv2::Value object of type unsignedRational
        Exiv2::Value::AutoPtr latValue = Exiv2::Value::create(Exiv2::unsignedRational);
        // Load the raw bytes from the vector into the value.
        // Note: reinterpret_cast to Exiv2::byte* is required.
        latValue->read(reinterpret_cast<const Exiv2::byte*>(latValues.data()),
                    latValues.size() * sizeof(Exiv2::Rational),
                    Exiv2::littleEndian);
        // Add the value to the ExifData
        exifData.add(Exiv2::ExifKey("Exif.GPSInfo.GPSLatitude"), latValue.get());
        exifData["Exif.GPSInfo.GPSLatitudeRef"] = latitude >= 0 ? "N" : "S";

        // Write GPS Longitude
        // Convert decimal longitude to DMS as a vector of Exiv2::Rational values
        std::vector<Exiv2::Rational> lonValues = {
            Exiv2::Rational(static_cast<int>(std::fabs(longitude)), 1),
            Exiv2::Rational(static_cast<int>((std::fabs(longitude) - static_cast<int>(std::fabs(longitude))) * 60), 1),
            Exiv2::Rational(
                static_cast<int>(
                    (
                        ((std::fabs(longitude) - static_cast<int>(std::fabs(longitude))) * 60
                         - static_cast<int>((std::fabs(longitude) - static_cast<int>(std::fabs(longitude))) * 60))
                        * 60 * 1000
                    )
                ),
                1000
            )
        };
        // Create an Exiv2::Value object of type unsignedRational
        Exiv2::Value::AutoPtr lonValue = Exiv2::Value::create(Exiv2::unsignedRational);
        // Load the raw bytes from the vector into the value.
        // Note: reinterpret_cast to Exiv2::byte* is required.
        lonValue->read(reinterpret_cast<const Exiv2::byte*>(lonValues.data()),
                    latValues.size() * sizeof(Exiv2::Rational),
                    Exiv2::littleEndian);
        // Add the value to the ExifData
        exifData.add(Exiv2::ExifKey("Exif.GPSInfo.GPSLongitude"), lonValue.get());
        exifData["Exif.GPSInfo.GPSLongitudeRef"] = longitude >= 0 ? "E" : "W";

        // Write altitude data
        Exiv2::Value::AutoPtr altRef = Exiv2::Value::create(Exiv2::unsignedByte);
        altRef->read("0");  // "0" indicates above sea level
        exifData["Exif.GPSInfo.GPSAltitudeRef"] = *altRef;  // Dereference here
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

    std::string video_file, frame_id, camera_info_file, camera_name;
    bool bag_sync = true, save_splat_images = false;
    int splat_fps;
    node->declare_parameter("video_file", video_file);
    node->declare_parameter("frame_id", frame_id);
    node->declare_parameter("camera_name", camera_name);
    node->declare_parameter("bag_sync", bag_sync);
    node->declare_parameter("camera_info_file", camera_info_file);
    node->declare_parameter("save_splat_images", save_splat_images);
    node->declare_parameter("splat_fps", splat_fps);

    node->get_parameter("video_file", video_file);
    node->get_parameter("frame_id", frame_id);
    node->get_parameter("camera_name", camera_name);
    node->get_parameter("bag_sync", bag_sync);
    node->get_parameter("camera_info_file", camera_info_file);
    node->get_parameter("save_splat_images", save_splat_images);
    node->get_parameter("splat_fps", splat_fps);

    rclcpp::Client<messages_88::srv::Geopoint>::SharedPtr geo_client_ = node->create_client<messages_88::srv::Geopoint>("/task_manager/slam2geo");
    
    rclcpp::Time start_time;
    if (bag_sync) {
        // Ensure service client is available
        if (save_splat_images && !geo_client_->wait_for_service(std::chrono::seconds(10))) {
            RCLCPP_ERROR(node->get_logger(), "Geo service unavailable, skipping EXIF.");
            return 1;
        }

        RCLCPP_INFO(node->get_logger(), "Waiting for IMU message to start video publishing");
        bool imu_received = false;
        auto sensor_qos = rclcpp::SensorDataQoS();
        auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
            "/mavros/imu/data", sensor_qos,
            [&imu_received, &start_time](sensor_msgs::msg::Imu::SharedPtr msg) {
                RCLCPP_INFO(rclcpp::get_logger("simple_wait_for_message"), "Received imu");
                imu_received = true;
                start_time = msg->header.stamp; // Corrected access
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
        node->create_publisher<sensor_msgs::msg::Image>(camera_name + "/image_raw", 10);
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher =
        node->create_publisher<sensor_msgs::msg::CameraInfo>(camera_name + "/camera_info", 10);

    cv::VideoCapture cap(video_file); 
    if (!cap.isOpened())
    {
        RCLCPP_ERROR(node->get_logger(), "Error opening video file");
        return 1;
    }

    double fps = cap.get(cv::CAP_PROP_FPS);
    int frame_count = 0;

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
        // start_time = node->get_clock()->now();
        std::cerr << "Error parsing date/time string, frame timestamps will start at ros bag time" << std::endl;
    }
    else {
        // Convert tm to time_t
        std::time_t time = std::mktime(&tm);
        start_time = rclcpp::Time(time, 0); // Need to get fractional seconds storing
    }

    // Load camera info
    // std::string camera_info_url = camera_info_file;
    sensor_msgs::msg::CameraInfo camera_info_;
    camera_calibration_parsers::readCalibration(camera_info_file, camera_name, camera_info_);

    // TF2 buffer and listener
    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    std::string outputDir = base_path + "/" + camera_name + "_images";
    if (!std::filesystem::exists(outputDir)) {
        std::filesystem::create_directory(outputDir);
    }

    cv::Mat frame;
    sensor_msgs::msg::Image::SharedPtr image_msg;
    rclcpp::WallRate loop_rate(fps);

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

            if (save_splat_images) {
                // Get the transform from camera to map
                geometry_msgs::msg::TransformStamped transform_stamped;
                try {
                    transform_stamped = tf_buffer.lookupTransform("map", frame_id, tf2::TimePointZero);
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
                            // RCLCPP_INFO(node->get_logger(), "Got lat/long for image: %f, %f. Home: %f", lat, lon, home_alt);

                            // Save image
                            std::string image_name = outputDir + "/image_" + std::to_string(frame_count) + ".jpg";
                            cv::imwrite(image_name, frame); // Save the frame
            
                            double altitude = transform_stamped.transform.translation.z + home_alt;
                            embedGPSData(image_name, lat, lon, altitude, start_time + t); // Embed GPS data
                        }
                        catch (const std::exception &e)
                        {
                            RCLCPP_INFO(node->get_logger(), "Geo service call failed, no lat/long available for EXIF data.");
                        }
                    
                    } else {
                        RCLCPP_ERROR(node->get_logger(), "Failed to get lat/long, no EXIF.");
                    }
    
                } catch (tf2::TransformException &ex) {
                    RCLCPP_WARN(node->get_logger(), "Could not transform %s to map: %s", frame_id.c_str(), ex.what());
                }

            }
        } 
        catch (cv_bridge::Exception& e) 
        {
            RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
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