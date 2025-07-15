#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <thread>


using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;



class Control : public rclcpp::Node
{

public:
    Control() : Node("control")
    {
        qos_profile = rmw_qos_profile_sensor_data; // might break in the future
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        // publisher instantiations : gives the publisher actual value
        actuator_motors_publisher = this->create_publisher<ActuatorMotors>("/fmu/in/actuator_motors", 10);

        // subscriber
        vehicle_control_mode_subscriber = this->create_subscription<VehicleControlMode>(
            "/fmu/out/vehicle_control_mode", 
            qos, 
            std::bind(&Control::subscribe_control_mode, this, std::placeholders::_1)
        );



    }

private:
    // Declarations
    bool is_running = false;
    rmw_qos_profile_t qos_profile;

    // Publishers : defines the publisher 
    rclcpp::Publisher<ActuatorMotors>::SharedPtr actuator_motors_publisher;
    // MAKE /DEMO_STATUS PUBLISHER AND TOPIC !!!
    

    // Subscribers : defines the subscriber
    rclcpp::Subscription<VehicleControlMode>::SharedPtr vehicle_control_mode_subscriber;


    // method declarations
    void subscribe_control_mode(const VehicleControlMode & msg);
    void demo();
    void go_straight();
    void stop();

};



void Control::subscribe_control_mode(const VehicleControlMode & msg)
{
    RCLCPP_INFO(this->get_logger(), "Flag_armed: %d", msg.flag_armed);

    if (msg.flag_armed && !is_running)      // if msg.flag_armed is true AND is_running is false
    {
        RCLCPP_INFO(this->get_logger(), "Starting Demo");
        is_running = true;
        demo();   
    }

}



void Control::demo()
{
    int delay = 5; //seconds

    go_straight();
    RCLCPP_INFO(this->get_logger(), "Moving Forward");

    std::this_thread::sleep_for(std::chrono::seconds(delay)); // waits 5 seconds

    stop();
    RCLCPP_INFO(this->get_logger(), "Stopping");

    // send demo status over to /demo_status0,


}



void Control::go_straight()
{
    ActuatorMotors msg{};
    
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    
                // right motor %   left motor %
    msg.control = {     0.1f,           0.1f,      0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};    

    actuator_motors_publisher->publish(msg);
}



void Control::stop()
{
    ActuatorMotors msg{};
    
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.control = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};    

    actuator_motors_publisher->publish(msg);
}



int main(int argc, char *argv[])
{
    std::cout << "Starting Demo_Control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Control>());

    rclcpp::shutdown();
    return 0;
}
