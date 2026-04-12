#include "rclcpp/rclcpp.hpp"
#include "cashier_system/msg/bill.hpp"
#include <iostream>

class BillGenerator : public rclcpp::Node {
public:
    BillGenerator() : Node("bill_generator") {
        publisher_ = this->create_publisher<cashier_system::msg::Bill>("bill", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(3),
            std::bind(&BillGenerator::generate_bill, this));
    }

private:
    void generate_bill() {
        cashier_system::msg::Bill bill;

        std::string item;
        int qty;
        float price;

        std::cout << "Enter item: ";
        std::cin >> item;
        std::cout << "Enter quantity: ";
        std::cin >> qty;
        std::cout << "Enter price: ";
        std::cin >> price;

        bill.item_name = item;
        bill.quantity = qty;
        bill.price_per_item = price;

        publisher_->publish(bill);

        RCLCPP_INFO(this->get_logger(), "Published bill for %s", item.c_str());
    }

    rclcpp::Publisher<cashier_system::msg::Bill>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BillGenerator>());
    rclcpp::shutdown();
    return 0;
}
