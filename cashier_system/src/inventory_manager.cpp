#include "rclcpp/rclcpp.hpp"
#include "cashier_system/msg/bill.hpp"
#include "cashier_system/srv/get_status.hpp"
#include <map>

using std::placeholders::_1;
using std::placeholders::_2;

class InventoryManager : public rclcpp::Node {
public:
    InventoryManager() : Node("inventory_manager"), total_income_(0.0) {

        subscription_ = this->create_subscription<cashier_system::msg::Bill>(
            "bill", 10,
            std::bind(&InventoryManager::bill_callback, this, _1));

        service_ = this->create_service<cashier_system::srv::GetStatus>(
            "get_status",
            std::bind(&InventoryManager::handle_status, this, _1, _2));
    }

private:
    void bill_callback(const cashier_system::msg::Bill::SharedPtr msg) {
        inventory_[msg->item_name] += msg->quantity;
        total_income_ += msg->quantity * msg->price_per_item;

        RCLCPP_INFO(this->get_logger(), "Updated inventory for %s", msg->item_name.c_str());
    }

    void handle_status(
        const std::shared_ptr<cashier_system::srv::GetStatus::Request>,
        std::shared_ptr<cashier_system::srv::GetStatus::Response> response) {

        for (auto &item : inventory_) {
            response->item_names.push_back(item.first);
            response->quantities.push_back(item.second);
        }

        response->total_income = total_income_;
    }

    std::map<std::string, int> inventory_;
    float total_income_;

    rclcpp::Subscription<cashier_system::msg::Bill>::SharedPtr subscription_;
    rclcpp::Service<cashier_system::srv::GetStatus>::SharedPtr service_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InventoryManager>());
    rclcpp::shutdown();
    return 0;
}
