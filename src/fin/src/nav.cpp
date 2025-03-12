#include <chrono>
#include <ebot_docking/srv/detail/dock_sw__struct.hpp>
#include <ebot_docking/srv/detail/passing_service__struct.hpp>
#include <ebot_docking/srv/passing_service.hpp>
#include <functional>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <ratio>
#include <rclcpp/client.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/detail/bool__struct.hpp>
#include <thread>
#include <vector>

class Nav : public rclcpp::Node {
  public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    Nav() : Node("waypoint_follower"), waypoint_index_(0) {
        nav_client = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Nav::send_goal, this));
    }

  private:
    struct Quaternion {
        double w, x, y, z;
    };

    float waypoints_[3][3] = {{0.4, -2.4, 3.14}, {-4.0, 2.89, -1.57}, {2.32, 2.55, -1.57}};

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client;
    rclcpp::Client<ebot_docking::srv::DockSw>::SharedPtr dockClient;
    int waypoint_index_ = 0;
    int current_goal = 10;
    bool inside_dock = false;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::thread> threads_, threads_2;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;

    void send_goal() {
        if (current_goal == waypoint_index_) {
            return;
        }

        if (inside_dock) {
            auto vel_msg = geometry_msgs::msg::Twist();
            vel_msg.linear.x = 0.5;
            vel_pub->publish(vel_msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(2500));
            inside_dock = false;
            vel_msg.linear.x = 0.0;
            vel_pub->publish(vel_msg);
        }

        if (!nav_client->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = waypoints_[waypoint_index_][0];
        goal_msg.pose.pose.position.y = waypoints_[waypoint_index_][1];
        goal_msg.pose.pose.orientation = to_quaternion(0.0, 0.0, waypoints_[waypoint_index_][2]);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback =
            std::bind(&Nav::goal_result_callback, this, std::placeholders::_1);

        current_goal = waypoint_index_;

        nav_client->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_result_callback(const GoalHandleNavigate::WrappedResult &result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Reached waypoint %d", waypoint_index_);
            call_dock();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to reach waypoint %d", waypoint_index_);
            current_goal = 10;
        }
    }

    inline void call_passing_service() {
        threads_2.push_back(std::thread(std::bind(&Nav::callback_call_passing_service, this)));
    }

    void callback_call_passing_service() {
        auto passClient =
            this->create_client<ebot_docking::srv::PassingService>("/passing_service");

        while (!passClient->wait_for_service(std::chrono::milliseconds(1000))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for passing_service...");
        }

        auto request = std::make_shared<ebot_docking::srv::PassingService::Request>();

        auto future = passClient->async_send_request(request);
        try {
            auto result = future.get();
            RCLCPP_INFO(this->get_logger(), "Drop at : %d", result->conveyer);
            keep_safe = result->conveyer;

        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Passing Service call failed: %s", e.what());
        }
    }

    void callback_call_dock() {
        dockClient = this->create_client<ebot_docking::srv::DockSw>("/docking");

        while (!dockClient->wait_for_service(std::chrono::milliseconds(1000))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for docking server...");
        }

        auto request = std::make_shared<ebot_docking::srv::DockSw::Request>();
        request->target = waypoint_index_;

        auto future = dockClient->async_send_request(request);

        try {
            auto result = future.get();
            RCLCPP_INFO(this->get_logger(), "Docking %d", result->success);
            if (waypoint_index_ == 0) {
                call_passing_service();
            } else {
                inside_dock = true;
                waypoint_index_ = 0;
                RCLCPP_INFO(this->get_logger(), "current_index %d", waypoint_index_);
            }

        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Docking Service Call Failed: %s", e.what());
        }
    }

    inline void call_dock() {
        threads_.push_back(std::thread(std::bind(&Nav::callback_call_dock, this)));
    }

    geometry_msgs::msg::Quaternion to_quaternion(double roll, double pitch, double yaw) {
        geometry_msgs::msg::Quaternion q;
        Quaternion tf_q;
        tf_q = rpyToQuaternion(roll, pitch, yaw);
        q.x = tf_q.x;
        q.y = tf_q.y;
        q.z = tf_q.z;
        q.w = tf_q.w;
        return q;
    }

    Quaternion rpyToQuaternion(double roll, double pitch, double yaw) {
        // Convert angles to radians
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);

        Quaternion q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;

        return q;
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pass_done_sub =
        this->create_subscription<std_msgs::msg::Bool>(
            "/pass_done", 10, std::bind(&Nav::callback_pass_done_sub, this, std::placeholders::_1));

    bool pass_done = false;
    int keep_safe = 0;
    void callback_pass_done_sub(std_msgs::msg::Bool::SharedPtr msg) {
        pass_done = msg->data;
        if (pass_done) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            inside_dock = true;
            waypoint_index_ = keep_safe;
            RCLCPP_INFO(this->get_logger(), "current_index %d", waypoint_index_);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Nav>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
