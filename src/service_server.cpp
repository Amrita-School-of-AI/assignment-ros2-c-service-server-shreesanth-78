#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;

class ServiceServer : public rclcpp::Node
{
public:
  ServiceServer() : Node("service_server")
  {
    service_ = this->create_service<AddTwoInts>(
      "add_two_ints",
      std::bind(
        &ServiceServer::handle_service,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );
  }

private:
  void handle_service(
    const std::shared_ptr<AddTwoInts::Request> request,
    std::shared_ptr<AddTwoInts::Response> response)
  {
    response->sum = request->a + request->b;
  }

  rclcpp::Service<AddTwoInts>::SharedPtr service_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServiceServer>());
  rclcpp::shutdown();
  return 0;
}

