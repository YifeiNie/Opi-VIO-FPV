#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  auto node=rclcpp::Node::make_shared("helloworld_node_cpp");
  RCLCPP_INFO(node->get_logger(),"hello world");


  rclcpp::shutdown();
  return 0;
}
