#include "rclcpp/rclcpp.hpp"
#include "cashier_system/srv/get_status.hpp"

using namespace std::chrono_literals;

class StatusViewer : public rclcpp::Node {
public:
    StatusViewer() : Node("status_viewer") {
        client_ = this->create_client<cashier_system::srv::GetStatus>("get_status");

        timer_ = this->create_wall_timer(
            5s,
            std::bind(&StatusViewer::request_status, this));
    }

private:
    void request_status() {
        if (!client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Service not available");
            return;
        }

        auto request = std::make_shared<cashier_system::srv::GetStatus::Request>();

        auto future = client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future)
            == rclcpp::FutureReturnCode::SUCCESS) {

            auto response = future.get();

            RCLCPP_INFO(this->get_logger(), "---- STATUS ----");

            for (size_t i = 0; i < response->item_names.size(); i++) {
                RCLCPP_INFO(this->get_logger(), "%s : %d",
                    response->item_names[i].c_str(),
                    response->quantities[i]);
            }

            RCLCPP_INFO(this->get_logger(), "Total Income: %.2f", response->total_income);
        }
    }

    rclcpp::Client<cashier_system::srv::GetStatus>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatusViewer>());
    rclcpp::shutdown();
    return 0;
}
