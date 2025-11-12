#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <random>
#include <string>
#include <sstream>

using namespace std::chrono_literals;


class AuctionParticipant : public rclcpp::Node
{
public:
    AuctionParticipant(int uav_id)
    : Node("auction_participant_" + std::to_string(uav_id)), uav_id_(uav_id)
    {
        task_sub_ = this->create_subscription<std_msgs::msg::String>(
            "auction_task", 10,
            std::bind(&AuctionParticipant::task_callback, this, std::placeholders::_1));
        bid_pub_ = this->create_publisher<std_msgs::msg::String>("auction_bid", 10);
        winner_sub_ = this->create_subscription<std_msgs::msg::String>(
            "auction_winner", 10,
            std::bind(&AuctionParticipant::winner_callback, this, std::placeholders::_1));
    }

private:
    // Para evitar pujar dos veces por la misma tarea
    std::set<std::string> tasks_bid_;
    // Para evitar anunciar varias veces el mismo ganador
    std::set<std::string> tasks_won_;

    void task_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string task_id = msg->data;
        if (tasks_bid_.count(task_id)) return; // Ya pujé por esta tarea
        tasks_bid_.insert(task_id);
        float cost = random_cost();
        std_msgs::msg::String bid_msg;
        std::stringstream ss;
        ss << uav_id_ << ":" << task_id << ":" << cost;
        bid_msg.data = ss.str();
        RCLCPP_INFO(this->get_logger(), "UAV %d puja %.2f por tarea %s", uav_id_, cost, task_id.c_str());
        bid_pub_->publish(bid_msg);
    }

    void winner_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        // winner_msg: "uav_id:task_id"
        std::stringstream ss(msg->data);
        std::string winner_id_str, task_id;
        std::getline(ss, winner_id_str, ':');
        std::getline(ss, task_id, ':');
        int winner_id = std::stoi(winner_id_str);
        if (winner_id == uav_id_ && !tasks_won_.count(task_id)) {
            RCLCPP_INFO(this->get_logger(), "¡UAV %d ha ganado la tarea %s!", uav_id_, task_id.c_str());
            tasks_won_.insert(task_id);
        }
    }

    float random_cost()
    {
        static std::default_random_engine e(std::random_device{}());
        static std::uniform_real_distribution<float> dist(1.0, 10.0);
        return dist(e);
    }

    int uav_id_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr bid_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr winner_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    int uav_id = 1;
    if (argc > 1) {
        uav_id = std::stoi(argv[1]);
    }
    auto node = std::make_shared<AuctionParticipant>(uav_id);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}