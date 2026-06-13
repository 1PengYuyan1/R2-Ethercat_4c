#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

class UpRangeSubscriber : public rclcpp::Node {
public:
    UpRangeSubscriber()
        : rclcpp::Node("up_range_subscriber")
    {
        ranges_[0].name = "up_front";
        ranges_[0].topic = declare_parameter<std::string>(
            "topic_up_front", "/high/up_front/range");

        ranges_[1].name = "up_back";
        ranges_[1].topic = declare_parameter<std::string>(
            "topic_up_back", "/high/up_back/range");

        ranges_[2].name = "down_front";
        ranges_[2].topic = declare_parameter<std::string>(
            "topic_down_front", "/high/down_front/range");

        ranges_[3].name = "down_back";
        ranges_[3].topic = declare_parameter<std::string>(
            "topic_down_back", "/high/down_back/range");

        const double log_rate_hz = declare_parameter<double>("log_rate_hz", 5.0);

        for (size_t i = 0; i < ranges_.size(); ++i) {
            subs_[i] = create_subscription<sensor_msgs::msg::Range>(
                ranges_[i].topic,
                rclcpp::SensorDataQoS(),
                [this, i](const sensor_msgs::msg::Range::SharedPtr msg) {
                    handle_range(i, msg);
                });

            RCLCPP_INFO(get_logger(), "Subscribed %s -> %s",
                        ranges_[i].topic.c_str(), ranges_[i].name.c_str());
        }

        if (log_rate_hz > 0.0) {
            const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / log_rate_hz));
            summary_timer_ = create_wall_timer(period, [this]() { log_summary(); });
        }
    }

private:
    struct RangeState {
        std::string name;
        std::string topic;
        std::string frame_id;
        rclcpp::Time stamp;
        float range_m = 0.0f;
        bool received = false;
    };

    void handle_range(size_t index, const sensor_msgs::msg::Range::SharedPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto& state = ranges_[index];
        state.frame_id = msg->header.frame_id;
        state.stamp = msg->header.stamp;
        state.range_m = msg->range;
        state.received = true;
    }

    void log_summary()
    {
        std::array<RangeState, 4> snapshot;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            snapshot = ranges_;
        }

        std::ostringstream line;
        line << std::fixed << std::setprecision(3) << "[up_ranges]";
        for (const auto& state : snapshot) {
            line << ' ' << state.name << '=';
            if (!state.received) {
                line << "no_data";
            } else if (std::isfinite(state.range_m)) {
                line << state.range_m << "m";
            } else if (std::isinf(state.range_m)) {
                line << (std::signbit(state.range_m) ? "-inf" : "+inf");
            } else {
                line << "nan";
            }
        }

        RCLCPP_INFO(get_logger(), "%s", line.str().c_str());
    }

    std::array<RangeState, 4> ranges_;
    std::array<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr, 4> subs_;
    rclcpp::TimerBase::SharedPtr summary_timer_;
    std::mutex mutex_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UpRangeSubscriber>());
    rclcpp::shutdown();
    return 0;
}
