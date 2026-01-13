#include <rclcpp/rclcpp.hpp>

#include <visp_tracker_common/TrackerGUI.hpp>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<visp_tracker_common::TrackerGUI> gui = std::make_shared<visp_tracker_common::TrackerGUI>("tracekr_gui");
  bool status = gui->init();
  if (status) {

  }
  rclcpp::shutdown();
  return (status ? EXIT_SUCCESS : EXIT_FAILURE);
}
