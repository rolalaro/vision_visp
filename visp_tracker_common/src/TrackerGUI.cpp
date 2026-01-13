#include <visp_tracker_common/TrackerGUI.hpp>

namespace visp_tracker_common
{
TrackerGUI::TrackerGUI(const std::string &node_name) : rclcpp::Node(node_name)
{

}

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
