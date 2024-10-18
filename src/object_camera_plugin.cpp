#include <object_camera_plugin/object_camera_plugin.hpp>

namespace gazebo
{
  ObjectCameraPlugin::ObjectCameraPlugin() 
  {
    if(!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
    this->ros_node_ = rclcpp::Node::make_shared("object_camera_plugin");
    RCLCPP_INFO(this->ros_node_->get_logger(), "Object Camera Plugin node created.");
  }

  ObjectCameraPlugin::~ObjectCameraPlugin() 
  {
    RCLCPP_INFO(this->ros_node_->get_logger(), "Object Camera Plugin shutting down.");
  }

  void ObjectCameraPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
  {
    sdf::ElementPtr parent_sdf = _sdf->GetParent();
    // this->PrintSDF(parent_sdf, "");

    if (parent_sdf->HasElement("extension_length"))
    {
      this->extension_length_ = parent_sdf->Get<double>("extension_length");
    }
    else
    {
      RCLCPP_INFO(this->ros_node_->get_logger(), "Extension length not set. Defaulting to 3m");
      this->extension_length_ = 3.0;
    }

    if (parent_sdf-> HasElement("camera")&& parent_sdf -> GetElement("camera") -> HasElement("horizontal_fov"))
    {
      sdf::ElementPtr camera_sdf = parent_sdf -> GetElement("camera");
      this -> horizontal_fov_= camera_sdf -> Get<double>("horizontal_fov");
      RCLCPP_INFO(this->ros_node_->get_logger(), "Horizontal FOV: %f", this->horizontal_fov_);
    }

    if(parent_sdf-> HasElement("camera")&& parent_sdf -> GetElement("camera") -> HasElement("clip") &&
        parent_sdf -> GetElement("camera") -> GetElement("clip") -> HasElement("near") &&
        parent_sdf -> GetElement("camera") -> GetElement("clip") -> HasElement("far"))
    {
      sdf::ElementPtr clip_sdf = parent_sdf -> GetElement("camera")->GetElement("clip");
      this -> clip_far_ = clip_sdf -> Get<double>("far");
      this -> clip_near_ = clip_sdf -> Get<double>("near");
      RCLCPP_INFO(this->ros_node_->get_logger(), "Clip Far: %f, Clip Near: %f", this->clip_far_, this->clip_near_);
    }

    if(parent_sdf-> HasElement("pose"))
    {
      this -> camera_to_base_pose_ = parent_sdf -> Get<ignition::math::Pose3d>("pose");
      RCLCPP_INFO(this->ros_node_->get_logger(), "Camera Pose: %f, %f, %f", this->camera_to_base_pose_.Pos().X(), this->camera_to_base_pose_.Pos().Y(), this->camera_to_base_pose_.Pos().Z());
    }

    // Get the camera sensor
    this->camera_sensor_ = std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);
    this->depth_camera_sensor_ = std::dynamic_pointer_cast<sensors::DepthCameraSensor>(_sensor);
    if (!this->depth_camera_sensor_)
    {
      RCLCPP_ERROR(this->ros_node_->get_logger(), "Depth Camera sensor not found");
      return;
    }
    if (!this->camera_sensor_)
    {
      RCLCPP_ERROR(this->ros_node_->get_logger(), "Camera sensor not found");
      return;
    }
    
    // Activate the camera sensor
    this->camera_sensor_->SetActive(true);
    this->depth_camera_sensor_->SetActive(true);

    this->depth_connection_ = this->depth_camera_sensor_->DepthCamera()->ConnectNewDepthFrame(
      std::bind(&ObjectCameraPlugin::OnNewDepthFrame, this, std::placeholders::_1, 
                std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, 
                std::placeholders::_5)
    );
    this->connection_ = this->camera_sensor_->Camera()->ConnectNewImageFrame(
      std::bind(&ObjectCameraPlugin::OnNewFrame, this, std::placeholders::_1, 
                std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, 
                std::placeholders::_5)
    );

    this->image_pub_ = this->ros_node_->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
    this->polygon_stamped_pub_ = this->ros_node_->create_publisher<geometry_msgs::msg::PolygonStamped>("/camera/polygon_stamped", 10);
    RCLCPP_INFO(this->ros_node_->get_logger(), "Object Camera Plugin loaded.");
  }


  void ObjectCameraPlugin::OnNewDepthFrame(const float *depth_image, unsigned int width, unsigned int height, 
                                         unsigned int depth, const std::string &format)
  {
    // Store the depth image for later use in the RGB image processing
    this->depth_image_ = cv::Mat(height, width, CV_32FC1, const_cast<float*>(depth_image));
  }

  void ObjectCameraPlugin::OnNewFrame(const unsigned char *image, unsigned int width, unsigned int height, 
                                      unsigned int depth, const std::string &format)
  {
    // Convert the image to an OpenCV format
    cv::Mat cv_image(height, width, CV_8UC3, const_cast<unsigned char*>(image));

    //------------------------Functions for object detection or segmentation, can integrate neuron network here--------------------------------------------------------

    // convert the image to HSV format
    cv::Mat hsv_image;
    cv::cvtColor(cv_image, hsv_image, cv::COLOR_RGB2HSV); 
    // define the color range for the grass
    cv::Scalar lower_green(35, 40, 40);
    cv::Scalar upper_green(102, 255, 255);  
    // Threshold the HSV image to get only the green colors
    cv::Mat mask;
    cv::inRange(hsv_image, lower_green, upper_green, mask);

    // Find contours in the mask
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Check if contours are found
    if (contours.empty())
    {
      RCLCPP_INFO(this->ros_node_->get_logger(), "No contours found.");
      return;
    }

    // Find the largest contour and assume it is the grass
    std::vector<cv::Point> largest_contour;
    double max_area = 0;
    for (const auto& contour : contours)
    {
      double area = cv::contourArea(contour);
      if (area > max_area)
      {
        max_area = area;
        largest_contour = contour;
      }
    }

    //------------------------Functions for object detection or segmentation, can integrate neuron network here--------------------------------------------------------

    // ------------------------Functions for extract points from contour, can be replaced by other methods -------------------------------------------------//

    int segment_width = width/num_segments_;
    std::vector<cv::Point2f> intersection_points;

    for (int i = 0; i < num_segments_; ++i)
    {
      int x1 = i * segment_width;
      cv::Point2f line[2] = { cv::Point2f(x1, 0), cv::Point2f(x1, static_cast<float>(height)) };
      // Check for intersections with the contour
      for (size_t j = 0; j < largest_contour.size(); ++j)
      {
        cv::Point p1 = largest_contour[j];
        cv::Point p2 = largest_contour[(j + 1) % largest_contour.size()];
        cv::Point2f contour_segment[2] = { p1, p2 };
        cv::Point2f intersection = lineIntersection(line, contour_segment);
        // Store valid intersection points
        if (intersection.x >= 0 && intersection.y >= 0)
        {
            intersection_points.push_back(intersection);
        }
      }
    }

    // Filter out points near image edges
    int edge_distance = 40;  // Distance(pixel) from the edge of the image to consider for intersection
    std::vector<cv::Point2f> filtered_points;
    for (const auto& point : intersection_points)
    {
      if (point.x >= edge_distance && point.x < width - edge_distance &&
          point.y >= edge_distance && point.y < height - edge_distance)
      {
          filtered_points.push_back(point);
      }
    }

    if(filtered_points.size() == 0)
    {
      RCLCPP_INFO(this->ros_node_->get_logger(), "No intersection points found.");
    }
    // draw the intersection points on the image
    for (const auto& point : filtered_points)
    {
      cv::circle(cv_image, point, 5, cv::Scalar(0,0,255), -1);
    }
  
    // Transform image points to robot coordinates
    std::vector<cv::Point3f> polygon_points = ImageToRobotCoords(filtered_points, width, height);

    // Add furthuer points to build the polygon
    AddFurtherPoints(polygon_points);

    // Publish the polygon and image
    geometry_msgs::msg::PolygonStamped polygon_stamped_msg;
    polygon_stamped_msg.header.stamp = this -> ros_node_ -> now(); 
    polygon_stamped_msg.header.frame_id = "base_link"; 

    for (const auto& point : polygon_points)
    {
      geometry_msgs::msg::Point32 ros_point;
      ros_point.x = point.x ;
      ros_point.y = point.y;
      ros_point.z = point.z;
      polygon_stamped_msg.polygon.points.push_back(ros_point);
    }

    this->polygon_stamped_pub_->publish(polygon_stamped_msg);
    auto ros_image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", cv_image).toImageMsg();
    this->image_pub_->publish(*ros_image_msg);
  }

  void ObjectCameraPlugin::AddFurtherPoints(std::vector<cv::Point3f>& points)
  {
    if(points.size() == 0)
    {
        RCLCPP_INFO(this->ros_node_->get_logger(), "No Polygon to publish.");
    }
    else if (points.size() < 2)
    {
        RCLCPP_INFO(this->ros_node_->get_logger(), "Not enough points to calculate further points.");
        // Assuming the ground is flat, add two further points for the polygon
        cv::Point3f p1(points.back().x + 1.0, points.back().y, points.back().z);
        cv::Point3f p2(points[0].x + 1.0, points[0].y, points[0].z); 

        points.push_back(p1);
        points.push_back(p2);
    }
    else
    {
      const cv::Point3f& first_point = points.front();
      const cv::Point3f& last_point = points.back();
      cv::Point2f v1_xy(last_point.x - first_point.x, last_point.y - first_point.y);
      cv::Point2f v1_normalized_xy= v1_xy/ cv::norm(v1_xy);
      cv::Point2f normal_xy(-v1_normalized_xy.y, v1_normalized_xy.x);
      std::vector<cv::Point3f> outer_points;
      for (size_t i = 0; i < points.size(); i++)
      {
        cv::Point3f current_point = points[i];
        cv::Point3f outer_point(current_point.x + abs(normal_xy.x * this->extension_length_),
                                current_point.y + normal_xy.y * this->extension_length_,
                                current_point.z);
        outer_points.push_back(outer_point);
      }                       
      auto outer_size = outer_points.size();
      while(outer_size > 0)
      {
        points.push_back(outer_points[outer_size - 1]);
        outer_size--;
      }
    }
  }


  std::vector<cv::Point3f> ObjectCameraPlugin::ImageToRobotCoords(const std::vector<cv::Point2f>& image_points, int img_width, int img_height)
  {
    std::vector<cv::Point3f> camera_points;

    // Assuming pinhole camera model
    double fx =  img_width / (2 * std::tan(this -> horizontal_fov_ / 2.0));  // Horizontal FOV to focal length
    double fy =  img_height / (2 * std::tan(this -> horizontal_fov_ / 2.0)); // Assuming square pixels
    double cx = img_width / 2.0;
    double cy = img_height / 2.0;

    for (const auto& pt : image_points)
    {
      double x = (pt.x - cx) / fx;
      double y = (pt.y - cy) / fy;
      float z = this -> depth_image_.at<float>(pt.y, pt.x);  // Get the depth value from the depth image

      if (z<=this->clip_far_ - this->clip_near_ * 2)
      {
        camera_points.emplace_back(x * z, y * z, z);
      }
    }

    std::vector<cv::Point3f> robot_points;
    // Define the camera position and orientation (relative to base_link)
    tf2::Vector3 camera_position(this->camera_to_base_pose_.Pos().X(), this->camera_to_base_pose_.Pos().Y(), this->camera_to_base_pose_.Pos().Z());

    // (0 0 1)
    // (-1 0 0)
    // (0 -1 0)
    tf2::Matrix3x3 camera_orientation(0, 0, 1, -1, 0, 0, 0, -1, 0);

    // Define the transform for the camera relative to base_link
    tf2::Transform camera_to_base(camera_orientation, camera_position);
  
    // Apply the transformation to each point in the camera frame
    for (const auto& point : camera_points)
    {
      tf2::Vector3 point_in_camera_frame(point.x, point.y, point.z);
      // Transform the point to the base_link frame
      tf2::Vector3 point_in_base_frame = camera_to_base * point_in_camera_frame;
      robot_points.emplace_back(point_in_base_frame.x(), point_in_base_frame.y(), point_in_base_frame.z());
    }

    return robot_points;
  }

  cv::Point2f ObjectCameraPlugin::lineIntersection(cv::Point2f line1[2], cv::Point2f line2[2])
  {
    float x1 = line1[0].x, y1 = line1[0].y, x2 = line1[1].x, y2 = line1[1].y;
    float x3 = line2[0].x, y3 = line2[0].y, x4 = line2[1].x, y4 = line2[1].y;

    float den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (den == 0) return cv::Point2f(-1, -1);  // Lines are parallel, no intersection

    float t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den;
    float u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / den;

    if (0 <= t && t <= 1 && 0 <= u && u <= 1)
    {
      float ix = x1 + t * (x2 - x1);
      float iy = y1 + t * (y2 - y1);
      return cv::Point2f(ix, iy);
    }
    else
    {
      return cv::Point2f(-1, -1);  // No valid intersection
    }
  }

  GZ_REGISTER_SENSOR_PLUGIN(ObjectCameraPlugin)
}
