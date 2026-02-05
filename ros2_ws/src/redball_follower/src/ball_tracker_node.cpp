#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/pinhole_camera_model.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

class BallTrackerNode : public rclcpp::Node
{
private:
    // Define the synchronization policy for only Image and Depth
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;

public:
    BallTrackerNode() : Node("ball_tracker_node")
    {
        // Publisher for the 3D position of the ball (will not be used in this mode)
        publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/ball_position", 10);
        debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/ball_tracker/debug_image", 10);
        target_pixel_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/target_pixel_coords", 10);

        min_area_ = this->declare_parameter<int>("min_area", 150);
        ema_alpha_ = this->declare_parameter<double>("ema_alpha", 0.35);
        depth_median_window_ = this->declare_parameter<int>("depth_median_window", 5);
        depth_default_ = this->declare_parameter<double>("depth_default", 0.8);
        depth_scale_ = this->declare_parameter<double>("depth_scale", 0.001); // if 16UC1 in millimeters

        // Subscribe to D455 camera topics from Isaac Sim
        image_sub_.subscribe(this, "/d455/color/image_raw");
        depth_sub_.subscribe(this, "/d455/depth/image_raw");
        // NOTE: cam_info_sub_ is disabled as the topic is not available

        // Create a synchronizer for just image and depth
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), image_sub_, depth_sub_);

        sync_->setAgePenalty(0.2);

        // Register the new callback that only takes two messages
        sync_->registerCallback(std::bind(&BallTrackerNode::synced_callback, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Ball tracker node started in 2D-ONLY mode (camera_info not found).");
    }

private:
    // Modified callback for only Image and Depth
    void synced_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                         const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
    {
        RCLCPP_DEBUG(this->get_logger(), "Synced callback triggered (2D)!");
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
            // We don't need to copy depth_ptr unless we are doing 3D calculations
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // --- Ball detection in color image ---
        cv::Mat hsv_image;
        cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

        // Red color can wrap around in HSV, so we check two ranges
        cv::Scalar lower_red1(0, 120, 70);
        cv::Scalar upper_red1(10, 255, 255);
        cv::Scalar lower_red2(170, 120, 70);
        cv::Scalar upper_red2(180, 255, 255);
        cv::Mat mask1, mask2, mask;
        cv::inRange(hsv_image, lower_red1, upper_red1, mask1);
        cv::inRange(hsv_image, lower_red2, upper_red2, mask2);
        mask = mask1 | mask2;

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty())
        {
            auto largest_contour = std::max_element(contours.begin(), contours.end(),
                [](const auto& a, const auto& b) {
                    return cv::contourArea(a) < cv::contourArea(b);
                });

            double area = cv::contourArea(*largest_contour);
            if (area >= min_area_)
            {
                cv::Moments M = cv::moments(*largest_contour);
                if (M.m00 > 0)
                {
                    cv::Point2f center(M.m10 / M.m00, M.m01 / M.m00);
                    cv::Rect bounding_box = cv::boundingRect(*largest_contour);

                    // Depth extraction (robust: median over small ROI)
                    double depth = extractDepth(depth_msg, center);
                    if (std::isnan(depth) || depth <= 0.01) depth = depth_default_;

                    // EMA smoothing
                    if (!have_prev_)
                    {
                        u_ema_ = center.x;
                        v_ema_ = center.y;
                        z_ema_ = depth;
                        have_prev_ = true;
                    }
                    else
                    {
                        u_ema_ = ema_alpha_ * center.x + (1.0 - ema_alpha_) * u_ema_;
                        v_ema_ = ema_alpha_ * center.y + (1.0 - ema_alpha_) * v_ema_;
                        z_ema_ = ema_alpha_ * depth     + (1.0 - ema_alpha_) * z_ema_;
                    }

                    geometry_msgs::msg::Point pt;
                    pt.x = u_ema_;
                    pt.y = v_ema_;
                    pt.z = z_ema_;
                    target_pixel_pub_->publish(pt);

                    cv::rectangle(cv_ptr->image, bounding_box, cv::Scalar(0, 255, 0), 2);
                    cv::circle(cv_ptr->image, center, 5, cv::Scalar(0, 0, 255), -1);
                    cv::putText(cv_ptr->image,
                                "u=" + std::to_string(int(pt.x)) +
                                " v=" + std::to_string(int(pt.y)) +
                                " z=" + cv::format("%.2f", pt.z),
                                center + cv::Point2f(10, -10),
                                cv::FONT_HERSHEY_SIMPLEX, 0.4, {0,255,0}, 1);
                }
            }
        }
        debug_image_pub_->publish(*cv_ptr->toImageMsg());
    }

    double extractDepth(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
                        const cv::Point2f& center)
    {
        // Convert depth
        cv_bridge::CvImageConstPtr depth_ptr;
        try
        {
            if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
                depth_ptr = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
            else
                depth_ptr = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch(...)
        {
            return std::numeric_limits<double>::quiet_NaN();
        }

        int u = std::clamp<int>(int(std::round(center.x)), 0, depth_msg->width  - 1);
        int v = std::clamp<int>(int(std::round(center.y)), 0, depth_msg->height - 1);

        // ROI window
        int w = std::min(5, int(depth_msg->width));
        int h = std::min(5, int(depth_msg->height));
        int u0 = std::max(0, u - w/2);
        int v0 = std::max(0, v - h/2);
        int u1 = std::min<int>(depth_msg->width -1, u0 + w -1);
        int v1 = std::min<int>(depth_msg->height-1, v0 + h -1);

        std::vector<double> samples;
        samples.reserve(w*h);

        if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
        {
            for (int y = v0; y <= v1; ++y)
            {
                const uint16_t* row = depth_ptr->image.ptr<uint16_t>(y);
                for (int x = u0; x <= u1; ++x)
                {
                    uint16_t raw = row[x];
                    if (raw > 0) samples.push_back(raw * depth_scale_);
                }
            }
        }
        else
        {
            for (int y = v0; y <= v1; ++y)
            {
                const float* row = depth_ptr->image.ptr<float>(y);
                for (int x = u0; x <= u1; ++x)
                {
                    float val = row[x];
                    if (std::isfinite(val) && val > 0.0f) samples.push_back(double(val));
                }
            }
        }

        if (samples.empty()) return std::numeric_limits<double>::quiet_NaN();
        std::nth_element(samples.begin(),
                         samples.begin() + samples.size()/2,
                         samples.end());
        return samples[samples.size()/2];
    }

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_pixel_pub_;
    int min_area_;
    double ema_alpha_;
    int depth_median_window_;
    double depth_default_;
    double depth_scale_;
    bool have_prev_{false};
    double u_ema_{0}, v_ema_{0}, z_ema_{0};

    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    // message_filters::Subscriber<sensor_msgs::msg::CameraInfo> cam_info_sub_; // Disabled

    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    image_geometry::PinholeCameraModel cam_model_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = nullptr;
    try {
        node = std::make_shared<BallTrackerNode>();
        rclcpp::spin(node);
    } catch (const std::exception & e) {
        RCLCPP_FATAL(rclcpp::get_logger("ball_tracker_node_main"), "FATAL ERROR during node initialization: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
