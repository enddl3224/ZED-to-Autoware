#include "zed_convert/zed_convert.hpp"

#include <string>

namespace zed_convert
{
ZedConvert::ZedConvert(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("zed_convert", node_options)
{
    zed_objects_ = this->create_subscription<zed_msgs::msg::ObjectsStamped>(
        "input/zed_objects", rclcpp::SensorDataQoS{}.keep_last(1), std::bind(&ZedConvert::convertCallback, this, std::placeholders::_1));
    
    objects_pub_ = this->create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>(
        "output/object", rclcpp::QoS{1});
}


void ZedConvert::convertCallback(
  const zed_msgs::msg::ObjectsStamped::ConstSharedPtr input_objects)
{
    autoware_auto_perception_msgs::msg::DetectedObjects output_objects;
    output_objects.header = input_objects->header;

    output_objects.objects.resize(input_objects->objects.size());
    for (size_t i = 0; i < input_objects->objects.size(); ++i){
        output_objects.objects[i].kinematics.pose_with_covariance.pose.position.x = input_objects->objects[i].position[0];
        output_objects.objects[i].kinematics.pose_with_covariance.pose.position.y = input_objects->objects[i].position[1];
        output_objects.objects[i].kinematics.pose_with_covariance.pose.position.z = input_objects->objects[i].position[2];

        output_objects.objects[i].shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
        output_objects.objects[i].shape.dimensions.x = input_objects->objects[i].dimensions_3d[0];
        output_objects.objects[i].shape.dimensions.y = input_objects->objects[i].dimensions_3d[1];
        output_objects.objects[i].shape.dimensions.z = input_objects->objects[i].dimensions_3d[2];

        output_objects.objects[i].existence_probability = input_objects->objects[i].confidence;

        output_objects.objects[i].classification.resize(1);
        if(input_objects->objects[i].label == "PERSON"){
            output_objects.objects[i].classification[0].label = autoware_auto_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
        }
        else if(input_objects->objects[i].label == "VEHICLE"){
            output_objects.objects[i].classification[0].label = autoware_auto_perception_msgs::msg::ObjectClassification::CAR;
        }
        else {
            output_objects.objects[i].classification[0].label = autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN;
        }

    }
    objects_pub_->publish(output_objects);
}
} //namespace zed_convert

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<zed_convert::ZedConvert>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
