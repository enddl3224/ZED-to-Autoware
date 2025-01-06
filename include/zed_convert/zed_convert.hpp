#include "rclcpp/rclcpp.hpp"

#include <zed_msgs/msg/objects_stamped.hpp>
#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <autoware_auto_perception_msgs/msg/shape.hpp>

namespace zed_convert
{
    class ZedConvert : public rclcpp::Node
    {
    public:
        explicit ZedConvert(const rclcpp::NodeOptions & node_options);
    private:
        void convertCallback(const zed_msgs::msg::ObjectsStamped::ConstSharedPtr input_objects);

        rclcpp::Subscription<zed_msgs::msg::ObjectsStamped>::SharedPtr zed_objects_;
        rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr objects_pub_;
    };
} // namespace zed_convert