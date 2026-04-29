/**
 * @file object_tracking_node.cpp
 * @brief Object tracking node: subscribes to image, runs detector + tracker,
 *        publishes boxes, optional Detection2DArray, debug image.
 *
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#ifdef HAVE_VISION_MSGS
#include "vision_msgs/msg/detection2_d_array.hpp"
#endif

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "object_tracking/detection_utils.h"
#include "object_tracking/image_utils.h"
#include "vision_service.h"

class ObjectTrackingNode : public rclcpp::Node {
    public:
    ObjectTrackingNode() : Node("object_tracking_node") {
        std::string tracker_type = declare_parameter<std::string>("tracker_type", "bytetrack");
        std::string config_path = declare_parameter<std::string>("config_path", "");
        const bool lazy_load = declare_parameter<bool>("lazy_load", true);
        score_threshold_ = declare_parameter<double>("score_threshold", 0.25);
        labels_ = declare_parameter<std::vector<std::string>>("labels", std::vector<std::string>{});
        image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image_raw");
        debug_image_topic_ =
            declare_parameter<std::string>("debug_image_topic", "/object_tracking/debug_image");
        boxes_topic_ = declare_parameter<std::string>("boxes_topic", "/object_tracking/boxes");
        use_camera_ = declare_parameter<bool>("use_camera", true);
        camera_id_ = declare_parameter<int>("camera_id", 0);
        camera_fps_ = declare_parameter<double>("camera_fps", 30.0);
#ifdef HAVE_VISION_MSGS
        detections_topic_ = declare_parameter<std::string>("detections_topic", "/perception/tracks");
#endif

        if (config_path.empty()) {
            if (tracker_type != "bytetrack" && tracker_type != "ocsort") {
                throw std::runtime_error(
                    "tracker_type must be 'bytetrack' or 'ocsort', got: " + tracker_type);
            }
            config_path = GetDefaultConfigPath(tracker_type);
        }
        if (config_path.empty()) {
            throw std::runtime_error(
                "config_path is empty and default for tracker_type='" + tracker_type +
                "' not found. Set config_path to vision tracker yaml.");
        }

        service_ = VisionService::Create(config_path, "", lazy_load);
        if (!service_) {
            throw std::runtime_error(
                "VisionService::Create failed: " + VisionService::LastCreateError());
        }

        boxes_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(boxes_topic_, 10);
#ifdef HAVE_VISION_MSGS
        detections_pub_ =
            create_publisher<vision_msgs::msg::Detection2DArray>(detections_topic_, 10);
#endif
        debug_pub_ =
            create_publisher<sensor_msgs::msg::Image>(debug_image_topic_, rclcpp::SensorDataQoS());

        if (use_camera_) {
            image_pub_ =
                create_publisher<sensor_msgs::msg::Image>(image_topic_, rclcpp::SensorDataQoS());
            cap_.open(camera_id_);
            if (!cap_.isOpened()) {
                throw std::runtime_error(
                    "object_tracking: cannot open camera id=" + std::to_string(camera_id_));
            }
            const int period_ms =
                (camera_fps_ > 1.0) ? static_cast<int>(1000.0 / camera_fps_) : 33;
            camera_timer_ = create_wall_timer(
                std::chrono::milliseconds(period_ms),
                std::bind(&ObjectTrackingNode::OnCameraTimer, this));
            RCLCPP_INFO(
                get_logger(),
                "object_tracking_node: use_camera=true, publishing to %s, camera_id=%d",
                image_topic_.c_str(),
                camera_id_);
        } else {
            image_sub_ = create_subscription<sensor_msgs::msg::Image>(
                image_topic_,
                rclcpp::SensorDataQoS(),
                std::bind(&ObjectTrackingNode::OnImage, this, std::placeholders::_1));
            RCLCPP_INFO(
                get_logger(),
                "object_tracking_node: use_camera=false, subscribing to %s",
                image_topic_.c_str());
        }

        RCLCPP_INFO(
            get_logger(),
            "object_tracking_node started, config=%s, image_topic=%s",
            config_path.c_str(),
            image_topic_.c_str());
    }

    private:
    static std::string GetDefaultConfigPath(const std::string& tracker_type) {
        try {
            std::string share = ament_index_cpp::get_package_share_directory("object_tracking");
            return share + "/config/" + tracker_type + ".yaml";
        } catch (...) {
            return "";
        }
    }

    void OnCameraTimer() {
        std_msgs::msg::Header header;
        header.stamp = now();
        header.frame_id = "camera";
        sensor_msgs::msg::Image img_msg;
        if (!object_tracking::CaptureCameraFrame(cap_, img_msg, header)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "camera read empty frame");
            return;
        }
        image_pub_->publish(img_msg);
        cv::Mat bgr = object_tracking::ImageMsgToBgr(img_msg);
        if (bgr.empty()) return;
        ProcessFrame(img_msg.header, bgr);
    }

    void OnImage(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat bgr = object_tracking::ImageMsgToBgr(*msg);
        if (bgr.empty()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                3000,
                "invalid or unsupported image (encoding=%s)",
                msg->encoding.c_str());
            return;
        }
        ProcessFrame(msg->header, bgr);
    }

    void ProcessFrame(const std_msgs::msg::Header& header, const cv::Mat& bgr) {
        std::vector<VisionServiceResult> results;
        if (service_->InferImage(bgr, &results) != VISION_SERVICE_OK) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "infer failed: %s", service_->LastError().c_str());
            return;
        }

        std::vector<object_tracking::DetectionBox> boxes;
        boxes.reserve(results.size());
        for (const auto& r : results) {
            if (r.score < static_cast<float>(score_threshold_)) continue;
            object_tracking::DetectionBox b;
            b.x1 = r.x1;
            b.y1 = r.y1;
            b.x2 = r.x2;
            b.y2 = r.y2;
            b.score = r.score;
            b.label = r.label;
            b.track_id = r.track_id;
            b.class_name = LabelName(r.label);
            boxes.push_back(b);
        }

        boxes_pub_->publish(object_tracking::EncodeBoxes(boxes));
#ifdef HAVE_VISION_MSGS
        detections_pub_->publish(object_tracking::EncodeDetection2DArray(boxes, header));
#endif
        PublishDebugImage(header, bgr);
    }

    std::string LabelName(int label_id) const {
        if (label_id >= 0 && static_cast<size_t>(label_id) < labels_.size()) {
            return labels_[static_cast<size_t>(label_id)];
        }
        return "class_" + std::to_string(label_id);
    }

    void PublishDebugImage(const std_msgs::msg::Header& header, const cv::Mat& bgr) {
        cv::Mat out_image;
        if (service_->Draw(bgr, &out_image) != VISION_SERVICE_OK) return;
        if (out_image.empty() || out_image.rows != bgr.rows || out_image.cols != bgr.cols) return;
        debug_pub_->publish(object_tracking::BgrToImageMsg(out_image, header, "bgr8"));
    }

    std::unique_ptr<VisionService> service_;
    double score_threshold_{0.25};
    std::vector<std::string> labels_;
    std::string image_topic_;
    std::string debug_image_topic_;
    std::string boxes_topic_;
    bool use_camera_{true};
    int camera_id_{0};
    double camera_fps_{30.0};
    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr camera_timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
#ifdef HAVE_VISION_MSGS
    std::string detections_topic_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;
#endif
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr boxes_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<ObjectTrackingNode>());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("object_tracking_node"), "Exception: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
