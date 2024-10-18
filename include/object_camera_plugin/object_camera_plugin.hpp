#ifndef OBJECT_CAMERA_PLUGIN__OBJECTCAMERAPLUGIN_HPP_
#define OBJECT_CAMERA_PLUGIN__OBJECTCAMERAPLUGIN_HPP_

#include <string>
#include <sstream>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <ignition/math.hh>
#include <gazebo/plugins/CameraPlugin.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <regex>
#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/msgs.hh>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cv_bridge/cv_bridge.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <opencv2/calib3d.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace gazebo
{
  class ObjectCameraPlugin : public SensorPlugin
  {
    public: 
      ObjectCameraPlugin();
      virtual ~ObjectCameraPlugin();

      void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

    private:
      void OnNewFrame(const unsigned char *image, unsigned int width, unsigned int height, 
                      unsigned int depth, const std::string &format);
      
      void OnNewDepthFrame(const float *image, unsigned int width, unsigned int height, 
                           unsigned int depth, const std::string &format);
      

      cv::Point2f lineIntersection(cv::Point2f line1[2], cv::Point2f line2[2]);

      // Method for transforming image points to robot coordinates
      std::vector<cv::Point3f> ImageToRobotCoords(const std::vector<cv::Point2f>& image_points, int img_width, int img_height);

      // Add further points to build the polygon
      void AddFurtherPoints(std::vector<cv::Point3f>& points);

      void PrintSDF(sdf::ElementPtr _sdf, const std::string &prefix)
      {
        // Print the current element's name and value
        std::cout << prefix << _sdf->GetName();
        if (_sdf->GetValue())
        {
          std::cout << ": " << _sdf->GetValue()->GetAsString();
        }
        std::cout << std::endl;
        // Recurse over child elements
        sdf::ElementPtr child = _sdf->GetFirstElement();
        while (child)
        {
          PrintSDF(child, prefix + "  ");
          child = child->GetNextElement();
        }
      }

      // Gazebo-specific members
      sensors::CameraSensorPtr camera_sensor_;
      sensors::DepthCameraSensorPtr depth_camera_sensor_;
      event::ConnectionPtr connection_;
      event::ConnectionPtr depth_connection_;
      physics::WorldPtr world_; 

      // ROS2-related members
      rclcpp::Node::SharedPtr ros_node_;
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
      rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_stamped_pub_;

      // Polygon extraction members
      double extension_length_;
      int num_segments_ = 30;

      // Camera members
      cv::Mat depth_image_;
      double horizontal_fov_;
      double clip_near_ = 0.05;
      double clip_far_ = 8.0;
      ignition::math::Pose3d camera_to_base_pose_;
  };
}

#endif  // OBJECT_CAMERA_PLUGIN__OBJECTCAMERAPLUGIN_HPP_
