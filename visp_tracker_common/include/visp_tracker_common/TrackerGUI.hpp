#ifndef TRACKER_GUI_HPP
#define TRACKER_GUI_HPP

// ---- Base ROS ----
#include <rclcpp/rclcpp.hpp>
// ---- ROS Images ----
#include <image_transport/image_transport.hpp>
// ---- ROS Messages ----
#include <sensor_msgs/msg/image.hpp>
// ---- ROS Services ----
#include <std_srvs/srv/trigger.hpp>
// ---- ViSP ROS packages ----
#include <visp_bridge/camera.h>
#include <visp_tracker_common/names.hpp>
#include <visp_tracker_common/msg/named_feature_array.hpp>
#include <visp_tracker_common/msg/named_pose_array.hpp>
// ---- ViSP includes ----
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpQuaternionVector.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpTime.h>
#include <visp3/core/vpTranslationVector.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/gui/vpDisplayFactory.h>

// ---- System includes ----
#include <memory>
#include <mutex>
#include <optional>
#ifdef VISP_HAVE_OPENMP
#include <omp.h>
#endif

namespace visp_tracker_common
{
class TrackerGUI : public rclcpp::Node
{
public:
  TrackerGUI(const std::string &node_name);
protected:
  /**
   * @brief Enumeration permitting to choose the type of object to represent the 2D feature points
   * published by the client node.
   */
  typedef enum FeaturesType
  {
    POINT = 0, /*!< Use points to represent the 2D features published by the client node.*/
    CROSS = 1, /*!< Use crosses to represent the 2D features published by the client node.*/
    TYPE_COUNT = 2
  } FeaturesType;

  /**
   * @brief Cast a \b FeaturesType enum value into a \b std::stirng.
   *
   * @param mode The type of 2D features we want to cast into a string.
   * @return std::string The name of the \b FeaturesType enum value.
   */
  static std::string featuresTypeToString(const FeaturesType &mode);

  /**
   * @brief Cast a string into a \b FeaturesType enum value.
   * If \b name is not found, return \b FeaturesType::TYPE_COUNT .
   *
   * @param name The name of the display mode.
   * @return DisplayMode The corresponding \b FeaturesType enum value, or \b FeaturesType::TYPE_COUNT if not found.
   */
  static FeaturesType featuresTypeFromString(const std::string &name);

  /**
   * @brief Create a string that lists the different \b FeaturesType available.
   *
   * @param prefix The string that must prefix the list of modes.
   * @param sep The separator between the different modes.
   * @param suffix The string that must suffix the list of modes.
   * @return std::string The list containing the different modes.
   */
  static std::string getAvailableFeaturesType(const std::string &prefix = "< ", const std::string &sep = " , ", const std::string &suffix = " >");
};
}

#endif
