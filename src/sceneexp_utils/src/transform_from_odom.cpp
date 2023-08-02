#include "rclcpp/rclcpp.hpp"
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>

namespace transform_from_odom
{
  class TransformFromOdom : public rclcpp::Node {
    private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    tf2_ros::TransformBroadcaster tf_pub_;
    tf2::Transform tf2_;

    bool invert_ = false;

    void publish_transform_from_odom(nav_msgs::msg::Odometry::SharedPtr msg)
    {
      geometry_msgs::msg::Pose &pose = msg->pose.pose;
      
      tf2_.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
      tf2_.setRotation(tf2::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));
      
      geometry_msgs::msg::TransformStamped tfs;
      tfs.transform = tf2::toMsg(invert_ ? tf2_.inverse() : tf2_);

      const std::string parent_frame = msg->header.frame_id;
      const std::string child_frame = msg->child_frame_id;

      tfs.header.frame_id = invert_ ? child_frame : parent_frame;
      tfs.child_frame_id = invert_ ? parent_frame : child_frame;
      tfs.header.stamp = msg->header.stamp;
      tf_pub_.sendTransform(tfs);
    }

    public:
    TransformFromOdom(const rclcpp::NodeOptions &options) 
    : rclcpp::Node("ground_truth_node", options)
    , tf_pub_(this)
    , odom_sub_(create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            rclcpp::QoS(1),
            std::bind(&TransformFromOdom::publish_transform_from_odom, this, std::placeholders::_1)))
    {
      invert_ = declare_parameter<bool>("invert", false);
    }
};
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(transform_from_odom::TransformFromOdom)


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<transform_from_odom::TransformFromOdom>(rclcpp::NodeOptions{}));
  return 0;
}
