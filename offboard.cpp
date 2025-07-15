#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class Offboard : public rclcpp::Node
{
public:
    Offboard() : Node("offboard")
    {
        //isArmed = false;
        qos_profile = rmw_qos_profile_sensor_data; // might break in the future
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        // publisher instantiations
        attitude_setpoint_publisher = this->create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", 10);
        offboard_control_mode_publisher = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        vehicle_command_publisher = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        // subscriber instantiations
        vehicle_control_mode_subscriber = this->create_subscription<VehicleControlMode>(
            "/fmu/out/vehicle_control_mode", 
            qos, 
            std::bind(&Offboard::subscribe_control_mode, this, std::placeholders::_1)
        );

        tick = 0;

        auto timer_callback = [this]() -> void
        {
            if (tick % 10 == 0)
            {
                RCLCPP_INFO(this->get_logger(), "Tick: %ld", tick);
            }
            /* We check isArmed first inorder to take advantage of
             * short-circuit evaluation. Thereby skipping the 
             * tick % 10 operation if isArmed is true.
            */
		    if (!isArmed && tick % 10 == 0)
		    {
			
		       isArmed = true;  // change !`			

		 	   // Change to Offboard mode
		  	   this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
			   
		 	   // Arm the vehicle
		 	   this->arm();
		    }

            // offboard control mode needs to be paired with the specified setpoint
            publish_offboard_control_mode();
            publish_attitude_setpoint(); // in our case, this is the attitude setpoint
			
			tick++;            
        };

        timer = this->create_wall_timer(100ms, timer_callback);
    }

    void arm();
    void disarm();

private:
    // Declarations
    
    // Publishers
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher;
    rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_publisher;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher;
    
    // Subscribers
    rclcpp::Subscription<VehicleControlMode>::SharedPtr vehicle_control_mode_subscriber;
    
    // Misc
    std::atomic<uint64_t> timestamp;
    
    rclcpp::TimerBase::SharedPtr timer;
    
    uint64_t tick;
    
    bool isArmed;

    rmw_qos_profile_t qos_profile;

    // Methods
    void publish_attitude_setpoint();
    void publish_offboard_control_mode();
    void publish_vehicle_command(uint16_t command, float enabled = 0.0, float mode_id = 0.0);

    void subscribe_control_mode(const VehicleControlMode & msg);
};

void Offboard::arm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void Offboard::publish_attitude_setpoint()
{
    VehicleAttitudeSetpoint msg{};
    msg.yaw_sp_move_rate = 0.0;
    msg.q_d = {0.0, 0.0, 0.0, 0.0};
    msg.thrust_body = {0.0, 0.0, 0.0};
    
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    attitude_setpoint_publisher->publish(msg);
}

void Offboard::publish_offboard_control_mode()
{
    OffboardControlMode msg{};
    /*
    *   Boolean message values are listed in their respective heirarchy. For each true value,
    *   values listed bellow it are irrelevant. For more informations see:
    *   https://docs.px4.io/main/en/flight_modes/offboard.html#ros-2-messages
    */
    msg.position = false;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = true;
    msg.body_rate = false;

    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher->publish(msg);
}

void Offboard::publish_vehicle_command(uint16_t command, float enabled, float mode_id)
{
    VehicleCommand msg{};
    msg.param1 = enabled;
    msg.param2 = mode_id;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;

    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher->publish(msg);
}

void Offboard::subscribe_control_mode(const VehicleControlMode & msg)
{
    isArmed = msg.flag_armed;
}

int main(int argc, char *argv[])
{
    std::cout << "Starting offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Offboard>());

    rclcpp::shutdown();
    return 0;
}
