#include <chrono>
#include <exception>
#include <functional>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <memory>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/logging.hpp>
#include <thread>

/*#define SIM*/

#include <ebot_docking/srv/dock_sw.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#ifdef SIM

#include <sensor_msgs/msg/range.hpp>

#else

#include <std_msgs/msg/float32_multi_array.hpp>
#include <usb_servo/srv/servo_sw.hpp>

#endif  // SIM

class Dock : public rclcpp::Node {
  public:
    Dock() : Node("ebot_docking") {
        docking_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

#ifdef SIM
        usonicL_sub = this->create_subscription<sensor_msgs::msg::Range>(
            "/ultrasonic_rl/scan", 10,
            std::bind(&Dock::callback_usonicL_sub, this, std::placeholders::_1));

        usonicR_sub = this->create_subscription<sensor_msgs::msg::Range>(
            "/ultrasonic_rr/scan", 10,
            std::bind(&Dock::callback_usonicR_sub, this, std::placeholders::_1));
#else
        usonic_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "ultrasonic_sensor_std_float", 10,
            std::bind(&Dock::callback_usonic_sub, this, std::placeholders::_1));
#endif  // SIM

        vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        docking_server = this->create_service<ebot_docking::srv::DockSw>(
            "/docking",
            std::bind(&Dock::callback_docking_server, this, std::placeholders::_1,
                      std::placeholders::_2),
            rmw_qos_profile_services_default, docking_group);
    }

  private:
    bool b_usonicOn{false};

    float f_usonicLeft{}, f_usonicRight{};

    void callback_docking_server(const ebot_docking::srv::DockSw::Request::SharedPtr request,
                                 const ebot_docking::srv::DockSw::Response::SharedPtr response) {
        RCLCPP_INFO(this->get_logger(), "Docking Request Received");
        uint8_t target = request->target;

        auto vel_msg = geometry_msgs::msg::Twist();
        vel_pub->publish(vel_msg);

        while (!b_usonicOn) {
            RCLCPP_WARN(this->get_logger(), "Ultrasonic not working...");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }  // Waiting for ultrasonic node to start publishing

        static float f_stopDist = 28.0f;
        static bool b_isFirstDock{true};
        float f_rangeDiff{};

        while (1) {
            if ((f_usonicLeft > 80.0f) && (f_usonicRight > 80.0f)) {
                vel_msg.linear.x = -0.3;
                RCLCPP_INFO(this->get_logger(), "Getting in Range");
            } else if ((f_usonicLeft > f_stopDist) && (f_usonicRight > f_stopDist)) {
                f_rangeDiff = f_usonicLeft - f_usonicRight;
                vel_msg.angular.z = -0.03 * f_rangeDiff;

                if (abs(f_rangeDiff) < 5.0f) {
                    vel_msg.linear.x = -0.003 * f_usonicLeft;
                    vel_msg.angular.z = 0.0;
                } else {
                    vel_msg.linear.x = 0.0;
                }
                RCLCPP_INFO(this->get_logger(), "%f", f_rangeDiff);
            } else {
                vel_msg.linear.x = 0.0;
                vel_msg.angular.z = 0.0;
                vel_pub->publish(vel_msg);
#ifndef SIM
                if (!b_isFirstDock) call_servo(true);

#endif
                std::this_thread::sleep_for(std::chrono::seconds(2));
                b_isFirstDock = false;
                RCLCPP_INFO(this->get_logger(), "Stopped, %d", b_isFirstDock);
                break;
            }

            vel_pub->publish(vel_msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }

        RCLCPP_INFO(this->get_logger(), "%d", target);
        response->success = true;
    }

#ifdef SIM
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr usonicR_sub;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr usonicL_sub;

    void callback_usonicL_sub(sensor_msgs::msg::Range::SharedPtr msg) {
        f_usonicLeft = static_cast<int16_t>(msg->range * 100.0f);
        b_usonicOn = true;
        // RCLCPP_INFO(this->get_logger(), "%f", f_usonicLeft);
    }
    void callback_usonicR_sub(sensor_msgs::msg::Range::SharedPtr msg) {
        f_usonicRight = static_cast<int16_t>(msg->range * 100.0f);
        b_usonicOn = true;
        // RCLCPP_INFO(this->get_logger(), "%f", f_usonicRight);
    }
#else

    std::vector<std::thread> threads_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr usonic_sub;
    void callback_usonic_sub(std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        f_usonicLeft = msg->data[4];
        f_usonicRight = msg->data[5];

        b_usonicOn = true;
    }

    inline void call_servo(bool state) {
        threads_.push_back(std::thread(std::bind(&Dock::callback_call_servo, this, state)));
    }

    void callback_call_servo(bool state) {
        auto servoClient = this->create_client<usb_servo::srv::ServoSw>("toggle_usb_servo");

        while (!servoClient->wait_for_service(std::chrono::milliseconds(1000))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for servo server...");
        }

        auto request = std::make_shared<usb_servo::srv::ServoSw::Request>();
        request->servostate = state;

        auto future = servoClient->async_send_request(request);

        try {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "angle = %d", response->num);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Servo Call Failed");
        }
    }
#endif  // SIM

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;

    rclcpp::Service<ebot_docking::srv::DockSw>::SharedPtr docking_server;

    rclcpp::CallbackGroup::SharedPtr docking_group;
    rclcpp::CallbackGroup::SharedPtr main_group;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Dock>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
