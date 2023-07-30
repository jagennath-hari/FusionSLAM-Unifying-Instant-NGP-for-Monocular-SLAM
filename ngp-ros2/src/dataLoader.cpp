#include "dataLoader.h"

dataLoader::dataLoader() : Node("ngp_data_loader")
{
    this->declare_parameter("image_path", "images");
    this->declare_parameter("transform_path", "transfrom.json");
    this->imagePath_ = this->get_parameter("image_path").as_string();
    this->transformsName_ = this->get_parameter("transform_path").as_string();
    this->mapDataSub_ = this->create_subscription<rtabmap_msgs::msg::MapData>("/rtabmap/mapData", 10, std::bind(&dataLoader::mapDataCallback_, this, std::placeholders::_1));
}

dataLoader::~dataLoader()
{
}

void dataLoader::mapDataCallback_(const rtabmap_msgs::msg::MapData & mapDataMsg)
{
    std::pair<cv::Mat, cv::Mat> rgbd = std::make_pair(rtabmap::uncompressImage(mapDataMsg.nodes.back().image), rtabmap::uncompressImage(mapDataMsg.nodes.back().depth));
    if (!rgbd.first.empty() && !rgbd.second.empty() && this->isEmpty_(this->transformsName_)) 
    {
        cv::imwrite(this->imagePath_ + "rgb" + std::to_string(mapDataMsg.nodes.back().id) + ".png", rgbd.first);
        cv::imwrite(this->imagePath_ + "depth" + std::to_string(mapDataMsg.nodes.back().id) +".png", rgbd.second);
        std::ofstream file(this->transformsName_);
        file << std::setw(2) << this->createJson_(mapDataMsg) << std::endl;
    }
    else if (!rgbd.first.empty() && !rgbd.second.empty() && !this->isEmpty_(this->transformsName_))
    {
        cv::imwrite(this->imagePath_ + "rgb" + std::to_string(mapDataMsg.nodes.back().id) + ".png", rgbd.first);
        cv::imwrite(this->imagePath_ + "depth" + std::to_string(mapDataMsg.nodes.back().id) +".png", rgbd.second);
        nlohmann::ordered_json newJson = this->updateJson_(mapDataMsg);
        std::remove(this->transformsName_.c_str());
        std::ofstream file(this->transformsName_);
        file << std::setw(2) << newJson << std::endl;
    }
}

nlohmann::ordered_json dataLoader::createJson_(const rtabmap_msgs::msg::MapData & mapDataMsg)
{
    nlohmann::ordered_json transformsJson;
    // transformsJson["camera_angle_x"] = std::atan(mapDataMsg.nodes.back().width.back() / mapDataMsg.nodes.back().fx.back() * 2) * 2;
    // transformsJson["camera_angle_y"] = std::atan(mapDataMsg.nodes.back().height.back() / mapDataMsg.nodes.back().fy.back() * 2) * 2;
    transformsJson["fl_x"] = mapDataMsg.nodes.back().fx.back();
    transformsJson["fl_y"] = mapDataMsg.nodes.back().fy.back();
    transformsJson["p2"] = transformsJson["p1"] = transformsJson["k2"] = transformsJson["k1"] = 0.0f;
    transformsJson["cx"] = mapDataMsg.nodes.back().cx.back();
    transformsJson["cy"] = mapDataMsg.nodes.back().cy.back();
    transformsJson["w"] = mapDataMsg.nodes.back().width.back();
    transformsJson["h"] = mapDataMsg.nodes.back().height.back();
    transformsJson["aabb_scale"] = 16;
    
    //Populate frame 1
    nlohmann::ordered_json frameJson;
    frameJson["file_path"] = this->imagePath_ + "rgb" + std::to_string(mapDataMsg.nodes.back().id) + ".png";
    frameJson["sharpness"] = this->varianceOfLaplacian_(frameJson["file_path"]);
    frameJson["transform_matrix"] = this->geoMsgToJson_(mapDataMsg.nodes.back().pose);
    frameJson["depth_file_path"] = this->imagePath_ + "depth" + std::to_string(mapDataMsg.nodes.back().id) + ".png";

    //Push back frame 1 to frames
    std::vector<nlohmann::ordered_json> frames;
    frames.push_back(frameJson);
    transformsJson["frames"] = frames;

    return transformsJson;
}

nlohmann::ordered_json dataLoader::updateJson_(const rtabmap_msgs::msg::MapData & mapDataMsg)
{
    //Read the Json file and get the data
    std::ifstream file(this->transformsName_);
    nlohmann::ordered_json transformsJson = nlohmann::ordered_json::parse(file);
    file.close();

    //Update the file
    nlohmann::ordered_json newFrame;
    newFrame["file_path"] = this->imagePath_ + "rgb" + std::to_string(mapDataMsg.nodes.back().id) + ".png";
    newFrame["sharpness"] = this->varianceOfLaplacian_(newFrame["file_path"]);
    newFrame["transform_matrix"] = this->geoMsgToJson_(mapDataMsg.nodes.back().pose);
    newFrame["depth_file_path"] = this->imagePath_ + "depth" + std::to_string(mapDataMsg.nodes.back().id) + ".png";
    transformsJson["frames"].push_back(newFrame);

    return transformsJson;
}

bool dataLoader::isEmpty_(std::string& file)
{
    std::ifstream pFile(file);
    return pFile.peek() == std::ifstream::traits_type::eof();
}

nlohmann::ordered_json dataLoader::geoMsgToJson_(const geometry_msgs::msg::Pose& geoMsg)
{
    Eigen::Matrix3d rot = Eigen::Quaterniond(geoMsg.orientation.w, geoMsg.orientation.x, geoMsg.orientation.y, geoMsg.orientation.z).toRotationMatrix();
    double yaw = std::atan2(rot(1, 0), rot(0, 0));
    Eigen::Matrix4d transform;
    transform << std::cos(yaw) ,     0   ,  std::sin(yaw),  geoMsg.position.y,
                    0         ,     1   ,      0        ,  geoMsg.position.z,
                -std::sin(yaw),     0   ,  std::cos(yaw), geoMsg.position.x,
                    0         ,     0   ,        0      ,            1      ;

    nlohmann::ordered_json tfJson;
    
    for (int i = 0; i < 4; ++i)
    {
        nlohmann::ordered_json row;
        for (int j = 0; j < 4; ++j) row.push_back(transform(i, j));
        tfJson.push_back(row);
    }

    return tfJson;
}

double dataLoader::varianceOfLaplacian_(const std::string& imagePath)
{
    cv::Mat laplacian;
    cv::Laplacian(cv::imread(imagePath, cv::IMREAD_GRAYSCALE), laplacian, CV_64F);
    cv::Scalar mean, stddev;
    cv::meanStdDev(laplacian, mean, stddev);
    double laplacianVariance = stddev.val[0] * stddev.val[0];

    return laplacianVariance;
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dataLoader>());
    rclcpp::shutdown();

    return 0;
}