#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <string>
#include <sstream>
#include <limits>


class DistributedAuctioneer : public rclcpp::Node
{
public:
    DistributedAuctioneer(int uav_id, int total_uavs)
    : Node("distributed_auctioneer_" + std::to_string(uav_id)), uav_id_(uav_id), total_uavs_(total_uavs)
    {
        bid_sub_ = this->create_subscription<std_msgs::msg::String>(
            "auction_bid", 10,
            std::bind(&DistributedAuctioneer::bid_callback, this, std::placeholders::_1));
        winner_pub_ = this->create_publisher<std_msgs::msg::String>("auction_winner", 10);
    }

private:
    struct Bid {
        int uav_id;
        std::string task_id;
        float cost;
    };

    // bids_by_task[task_id] = vector<Bid>
    std::map<std::string, std::vector<Bid>> bids_by_task_;
    // Para evitar publicar varias veces el ganador de la misma tarea
    std::set<std::string> tasks_announced_;

    void bid_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        // msg->data: "uav_id:task_id:cost"
        std::stringstream ss(msg->data);
        std::string uav_id_str, task_id, cost_str;
        std::getline(ss, uav_id_str, ':');
        std::getline(ss, task_id, ':');
        std::getline(ss, cost_str, ':');
        int uav_id = std::stoi(uav_id_str);
        float cost = std::stof(cost_str);

        // Si ya se anunció el ganador de esta tarea, ignora
        if (tasks_announced_.count(task_id)) return;

        // Añade la puja solo si no existe ya para ese UAV y tarea
        auto &bids = bids_by_task_[task_id];
        bool already_bid = false;
        for (const auto &b : bids) {
            if (b.uav_id == uav_id) {
                already_bid = true;
                break;
            }
        }
        if (!already_bid) {
            bids.push_back({uav_id, task_id, cost});
        }

        if (bids.size() >= static_cast<size_t>(total_uavs_)) {
            // Selecciona el menor coste
            int winner_id = -1;
            float min_cost = std::numeric_limits<float>::max();
            for (const auto &bid : bids) {
                if (bid.cost < min_cost) {
                    min_cost = bid.cost;
                    winner_id = bid.uav_id;
                }
            }
            std_msgs::msg::String winner_msg;
            winner_msg.data = std::to_string(winner_id) + ":" + task_id;
            RCLCPP_INFO(this->get_logger(), "Ganador: UAV %d para tarea %s", winner_id, task_id.c_str());
            winner_pub_->publish(winner_msg);
            tasks_announced_.insert(task_id);
            // Limpia las pujas de esta tarea
            bids_by_task_.erase(task_id);
        }
    }

    int uav_id_;
    int total_uavs_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr bid_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr winner_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    int uav_id = 1;
    int total_uavs = 3; // Cambia según tu sistema
    if (argc > 1) {
        uav_id = std::stoi(argv[1]);
    }
    if (argc > 2) {
        total_uavs = std::stoi(argv[2]);
    }
    auto node = std::make_shared<DistributedAuctioneer>(uav_id, total_uavs);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}