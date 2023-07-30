#ifndef DATALOADER_H_
#define DATALOADER_H_

#include <fstream>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rtabmap_msgs/msg/map_data.hpp>
#include <rtabmap/core/Compression.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

class dataLoader : public rclcpp::Node
{
private:
    rclcpp::Subscription<rtabmap_msgs::msg::MapData>::SharedPtr mapDataSub_;
    void mapDataCallback_(const rtabmap_msgs::msg::MapData & mapDataMsg);
    std::string imagePath_;
    std::string transformsName_;
    bool isEmpty_(std::string& file);
    nlohmann::ordered_json createJson_(const rtabmap_msgs::msg::MapData & mapDataMsg);
    nlohmann::ordered_json updateJson_(const rtabmap_msgs::msg::MapData & mapDataMsg);
    double varianceOfLaplacian_(const std::string& imagePath);
    nlohmann::ordered_json geoMsgToJson_(const geometry_msgs::msg::Pose& geoMsg);
public:
    dataLoader();
    ~dataLoader();
};

#endif