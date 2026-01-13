#include <visp_tracker_common/TrackerGUI.hpp>

namespace visp_tracker_common
{
TrackerGUI::TrackerGUI(const std::string &node_name)
  : rclcpp::Node(node_name)
  , m_it(std::shared_ptr<rclcpp::Node>(this))
  , m_hints(std::shared_ptr<rclcpp::Node>(this).get())
{
  //////////////////////////////////////////////////////////////////////
  //                        ROS2 PARAMETERS                           //
  //////////////////////////////////////////////////////////////////////

  // // ---- Parameters related to the services ----

  // // ---- Parameters related to the publishers / subscribers ----
  this->declare_parameter<std::string>("image_transport", "compressed");
  this->declare_parameter<std::string>("camera_topic", "");
  this->declare_parameter<std::string>("color_topic", "");
  this->declare_parameter("color_qos_queue_depth", 1);
  this->declare_parameter("color_qos_durability", "volatile");
  this->declare_parameter("color_qos_reliability", "reliable");
  this->declare_parameter("features_topic", "");
  this->declare_parameter("poses_topic", "");
  m_client_node_name = this->declare_parameter<std::string>("client_node", "");

  // // ---- Other parameters ----
  auto features_thick_desc = rcl_interfaces::msg::ParameterDescriptor {};
  features_thick_desc.description = "Thickness to use to display the 2D features.";
  this->declare_parameter("features_thickness", 1, features_thick_desc);
  m_features_thickness = this->get_parameter("features_thickness").as_int();

  auto features_type_desc = rcl_interfaces::msg::ParameterDescriptor {};
  features_type_desc.description = "Available types of 2D visualization for the potential 2D features we are listening to: " + getAvailableFeaturesType();
  this->declare_parameter("features_type", "point", features_type_desc);
  m_features_type = featuresTypeFromString(this->get_parameter("features_type").as_string());


  //////////////////////////////////////////////////////////////////////
  //                        ROS2 SERVICES                             //
  //////////////////////////////////////////////////////////////////////
  m_switch_request = std::make_shared<std_srvs::srv::Trigger::Request>();
  m_quit_request = std::make_shared<std_srvs::srv::Trigger::Request>();
  rclcpp::NodeOptions options;
  m_service_node = rclcpp::Node::make_shared("tracker_gui_service_manager", options);

  //////////////////////////////////////////////////////////////////////
  //                        ROS2 PUB/SUB                              //
  //////////////////////////////////////////////////////////////////////

}

bool TrackerGUI::init()
{
  if (m_client_node_name.empty()) {
    RCLCPP_ERROR(this->get_logger(), "client_node parameter was not set");
    return false;
  }

  //////////////////////////////////////////////////////////////////////
  //                        ROS2 PUB/SUB                              //
  //////////////////////////////////////////////////////////////////////
  rmw_qos_profile_t compressed_color_qos = rmw_qos_profile_default;
  compressed_color_qos.depth = this->get_parameter("color_qos_queue_depth").as_int();
  visp_bridge::STREAM_QOS_DURABILITY color_qos_durability = visp_bridge::durabilityFromString(this->get_parameter("color_qos_durability").as_string());
  visp_bridge::STREAM_QOS_RELIABILITY color_qos_reliability = visp_bridge::reliabilityFromString(this->get_parameter("color_qos_reliability").as_string());
  switch (color_qos_durability) {
  case visp_bridge::QOS_DURABILITY_TRANSIENT:
    compressed_color_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    break;
  case visp_bridge::QOS_DURABILITY_VOLATILE:
    compressed_color_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    break;
  default:
  {
    RCLCPP_ERROR(this->get_logger(), "Durability '%s' unknown, allowed values are %s", this->get_parameter("color_qos_durability").as_string().c_str(), visp_bridge::durabilityList().c_str());
    return false;
  }
  }

  switch (color_qos_reliability) {
  case visp_bridge::QOS_RELIABILITY_BEST_EFFORT:
    compressed_color_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    break;
  case visp_bridge::QOS_RELIABILITY_RELIABLE:
    compressed_color_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    break;
  default:
  {
    RCLCPP_ERROR(this->get_logger(), "Reliability '%s' unknown, allowed values are %s", this->get_parameter("color_qos_reliability").as_string().c_str(), visp_bridge::reliabilityList().c_str());
    return false;
  }
  }

  image_transport::Subscriber subColor = m_it.subscribe(this->get_parameter("color_topic").as_string(), compressed_color_qos, std::bind(&TrackerGUI::image_callback, this, std::placeholders::_1), image_transport::ImageTransport::VoidPtr(), &m_hints, rclcpp::SubscriptionOptions());

  std::string rgb_cam_topic = this->get_parameter("camera_topic").as_string();
  if (rgb_cam_topic.empty()) {
    RCLCPP_ERROR(this->get_logger(), "'camera_topic' has not been set");
    return false;
  }
  auto qos_cam_params = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
  RCLCPP_INFO(this->get_logger(), "Subscribing to camera topic '%s'", rgb_cam_topic.c_str());
  m_rgb_cam_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(rgb_cam_topic, qos_cam_params, std::bind(&TrackerGUI::camera_info_callback, this, std::placeholders::_1));
  return true;
}

//////////////////////////////////////////////////////////////////////
//                        ROS2 SUBCRIPTIONS                         //
//////////////////////////////////////////////////////////////////////
void TrackerGUI::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (m_opt_rgb_cam) {
    return;
  }

  double fx = msg->k[0]; // fx
  double fy = msg->k[4]; // fy
  double cx = msg->k[2]; // cx
  double cy = msg->k[5]; // cy

  m_opt_rgb_cam = vpCameraParameters();
  m_opt_rgb_cam->initPersProjWithoutDistortion(fx, fy, cx, cy);
}

void TrackerGUI::depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
  static const unsigned int height = msg->height;
  static const unsigned int width = msg->width;
  static const unsigned int size = height * width;
  static vpImage<uint16_t> Iuint16(msg->height, msg->width);

  Iuint16 = std::move(visp_bridge::toVispImageUint16(*msg));

  uint16_t min = std::numeric_limits<uint16_t>::max();
  uint16_t max = 0;
  if (!(m_opt_min_depth && m_opt_max_depth)) {
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
    for (unsigned int idx = 0; idx < size; ++idx) {
      if ((Iuint16.bitmap[idx]!= std::numeric_limits<uint16_t>::max()) && (Iuint16.bitmap[idx] != 0)) {
        min = std::min(Iuint16.bitmap[idx], min);
        max = std::max(Iuint16.bitmap[idx], max);
      }
    }
  }

  if (m_opt_min_depth) {
    min = *m_opt_min_depth;
  }

  if (m_opt_max_depth) {
    max = *m_opt_max_depth;
  }

  float a = 255.f / static_cast<float>(max - min);
  float b = 255.f - a * static_cast<float>(max);

  vpImage<vpRGBa> Id(height, width, vpRGBa(0, 0, 0));
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
  for (unsigned int idx = 0; idx < size; ++idx) {
    if (Iuint16.bitmap[idx] <= min) {
      Id.bitmap[idx].B = Id.bitmap[idx].R = Id.bitmap[idx].G = 255;
    }
    else if (Iuint16.bitmap[idx] >= max) {
      Id.bitmap[idx].B = Id.bitmap[idx].R = Id.bitmap[idx].G = 0;
    }
    else {
      Id.bitmap[idx].B = static_cast<unsigned char>(a * static_cast<float>(Iuint16.bitmap[idx]) + b);
      Id.bitmap[idx].R = 255 - Id.bitmap[idx].B;
    }
  }

  {
    std::scoped_lock sl(m_mutex_Id);
    m_opt_Id = Id;
  }
}

void TrackerGUI::features_callback(visp_tracker_common::msg::NamedFeatureArray::SharedPtr msg)
{
  std::scoped_lock sl(m_mutex_features);
  m_feature_array = std::move(*msg);
}

void TrackerGUI::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
  static const unsigned int left_hor_offset = 20;
  static const unsigned int v_offset = 20;

  bool run = false;
  {
    std::scoped_lock sl(m_mutex_run);
    run = m_run;
  }
  if (!run) {
    return;
  }

  m_I = std::move(visp_bridge::toVispImageRGBa(*msg));

  if (!m_display_color) {
    m_display_color = vpDisplayFactory::createDisplay(m_I, -1, -1, "Tracker GUI: color image");
  }
  static std::optional<vpImage<vpRGBa>> opt_Id = std::nullopt;
  if (m_opt_Id) {
    {
      std::scoped_lock sl(m_mutex_Id);
      opt_Id = m_opt_Id.value();
    }
  }
  if (opt_Id) {
    if (!m_display_depth) {
      m_display_depth = vpDisplayFactory::createDisplay(opt_Id.value(), m_display_color->getWidth() + 20, -1, "Remote GUI: depth image");
    }
  }
  bool display_frame = (m_display_nb_frames_skipped <= 0) || ((m_frame_cnt % m_display_nb_frames_skipped) == 0);
  ++m_frame_cnt;
  if (display_frame) {
    vpDisplay::display(m_I);
    if (opt_Id) {
      vpDisplay::display(opt_Id.value());
    }

    {
      std::scoped_lock sl(m_mutex_info);
      for (unsigned int r = 0; r < m_vec_info.info_strings.size(); ++r) {
        vpDisplay::displayText(m_I, v_offset * (r + 1), m_I.getWidth() - m_vec_info.hor_offset_right_border[r], m_vec_info.info_strings[r], vpColor::red);
      }
    }

    {
      std::scoped_lock sl(m_mutex_poses);
      if (m_opt_rgb_cam) {
        for (const auto &named_pose: m_pose_array.poses) {
          vpHomogeneousMatrix H = visp_bridge::toVispHomogeneousMatrix(named_pose.pose);
          vpDisplay::displayFrame(m_I, H, m_opt_rgb_cam.value(), 0.03, vpColor::none, 2, vpImagePoint(0, 0), named_pose.name, vpColor::red);
        }
      }
    }

    {
      std::scoped_lock sl(m_mutex_features);
      int idx = 0;
      for (const auto &named_feature: m_feature_array.features) {
        vpColor color = vpColor::allColors[idx % vpColor::nbColors];
        vpDisplay::displayText(m_I, m_display_color->getHeight() -  v_offset * (idx + 1), left_hor_offset, named_feature.name, color);
        switch (m_features_type) {
        case FeaturesType::CROSS:
        {
          for (const auto &ip: named_feature.image_points) {
            vpDisplay::displayCross(m_I, ip.y, ip.x, 10, color, m_features_thickness);
          }
          break;
        }
        case FeaturesType::POINT:
        {
          for (const auto &ip: named_feature.image_points) {
            vpDisplay::displayPoint(m_I, ip.y, ip.x, color, m_features_thickness);
          }
          break;
        }
        default:
          RCLCPP_WARN_STREAM(this->get_logger(), "Visualization type for the 2D features is unknown, available types are: " << getAvailableFeaturesType());
        }
        for (const auto &ellipse: named_feature.ellipses) {
          vpDisplay::displayEllipse(m_I, vpImagePoint(ellipse.center.y, ellipse.center.x), ellipse.n20, ellipse.n11, ellipse.n02, true, color, m_features_thickness);
        }
        for (const auto &line: named_feature.lines) {
          vpDisplay::displayLine(m_I, vpImagePoint(line.start.y, line.start.x), vpImagePoint(line.end.y, line.end.x), color, m_features_thickness);
        }
        for (const auto &poly: named_feature.polygons) {
          std::vector<vpImagePoint> poly_points;
          poly_points.push_back(vpImagePoint(poly.lines[0].start.y, poly.lines[0].start.x));
          for (const auto &line: poly.lines) {
            poly_points.push_back(vpImagePoint(line.end.y, line.end.x));
          }
          vpDisplay::displayPolygon(m_I, poly_points, color, m_features_thickness);
        }
        for (const auto &rect: named_feature.rectangles) {
          vpDisplay::displayRectangle(m_I, vpImagePoint(rect.start.y, rect.start.x), vpImagePoint(rect.end.y, rect.end.x), color, false, m_features_thickness);
        }

        ++idx;
      }
    }

    vpDisplay::flush(m_I);
    if (opt_Id) {
      vpDisplay::flush(opt_Id.value());
    }
  }

  // Getting user interaction feedback
  vpMouseButton::vpMouseButtonType button;
  if (vpDisplay::getClick(m_I, button, false)) {
    switch (button) {
    case vpMouseButton::button1:
    {
      auto result = m_client_switch_tracking->async_send_request(m_switch_request);
      RCLCPP_INFO(this->get_logger(), "Sent a switch tracking request...");
      // Wait for the result.
      if (rclcpp::spin_until_future_complete(m_service_node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Got a response !");
        auto response = result.get();
        RCLCPP_INFO(this->get_logger(), "Message : '%s'", response->message.c_str());
        // m_tracker_active = response->is_tracker_active;
      }
      else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call switch service");
      }
      break;
    }
    case vpMouseButton::button2:
    {
      auto result = m_client_switch_visualization->async_send_request(m_switch_request);
      RCLCPP_INFO(this->get_logger(), "Sent a switch visualization request...");
      // Wait for the result.
      if (rclcpp::spin_until_future_complete(m_service_node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Got a response !");
        auto response = result.get();
        RCLCPP_INFO(this->get_logger(), "Message : '%s'", response->message.c_str());
      }
      else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call switch service");
      }
      break;
    }
    case vpMouseButton::button3:
    {
      std::scoped_lock sl(m_mutex_run);
      auto result = m_client_quit->async_send_request(m_quit_request);
      RCLCPP_INFO(this->get_logger(), "Sent a quit request...");
      // Wait for the result.
      if (rclcpp::spin_until_future_complete(m_service_node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Got a response !");
        RCLCPP_INFO(this->get_logger(), "Message : '%s'", result.get()->message.c_str());
        m_run = false;
      }
      else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call quit service");
      }
      break;
    }
    default:
      break;
    }
  }
}

void TrackerGUI::info_callback(visp_tracker_common::msg::InfoStrings::SharedPtr msg)
{
  std::scoped_lock sl(m_mutex_info);
  m_vec_info = std::move(*msg);
}

void TrackerGUI::poses_callback(visp_tracker_common::msg::NamedPoseArray::SharedPtr msg)
{
  std::scoped_lock sl(m_mutex_poses);
  m_pose_array = std::move(*msg);
}

//////////////////////////////////////////////////////////////////////
//                        OTHERS                                    //
//////////////////////////////////////////////////////////////////////
std::string TrackerGUI::featuresTypeToString(const TrackerGUI::FeaturesType &mode)
{
  switch (mode) {
  case POINT:
    return "point";
  case CROSS:
    return "cross";
  default:
    break;
  }
  return "unknown";
}

TrackerGUI::FeaturesType TrackerGUI::featuresTypeFromString(const std::string &name)
{
  FeaturesType res = FeaturesType::TYPE_COUNT;
  bool wasFound = false;
  std::string lowerCaseName = vpIoTools::toLowerCase(name);
  unsigned int i = 0;
  while ((i < FeaturesType::TYPE_COUNT) && (!wasFound)) {
    FeaturesType candidate = (FeaturesType)i;
    if (lowerCaseName == featuresTypeToString(candidate)) {
      res = candidate;
      wasFound = true;
    }
    ++i;
  }
  return res;
}

std::string TrackerGUI::getAvailableFeaturesType(const std::string &prefix, const std::string &sep, const std::string &suffix)
{
  std::string modes(prefix);
  for (unsigned int i = 0; i < FeaturesType::TYPE_COUNT - 1; ++i) {
    FeaturesType candidate = (FeaturesType)i;
    modes += featuresTypeToString(candidate) + sep;
  }
  FeaturesType candidate = (FeaturesType)(FeaturesType::TYPE_COUNT - 1);
  modes += featuresTypeToString(candidate) + suffix;
  return modes;
}
}
