#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <thread>



using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;



class key_control : public rclcpp::Node
{

public:
    key_control() : Node("control")
    {


        // publisher instantiations : gives the publisher actual value
        actuator_motors_publisher = this->create_publisher<ActuatorMotors>("/fmu/in/actuator_motors", 10);



		this->keyboard_loop();

        // test
       // int delay = 5;
       // 
       // this->go();
       // 
       // std::this_thread::sleep_for(std::chrono::seconds(delay)); // waits 5 seconds
       // 
       // this->stop();
    }

private:

// Declarations
    bool is_running = false;

    // Publishers : defines the publisher 
    rclcpp::Publisher<ActuatorMotors>::SharedPtr actuator_motors_publisher;
    // MAKE /DEMO_STATUS PUBLISHER AND TOPIC !!!
    

    // Subscribers : defines the subscriber
   // rclcpp::Subscription<VehicleControlMode>::SharedPtr vehicle_control_mode_subscriber;


    // method declarations
    void go();
    void stop();
    void right_turn();
    void left_turn();

    int getKeyInput();
    void keyboard_loop();

	void disarm(); // add code for dis need vehicle cmomman dpublisher stuff
};



void key_control::go()
{
    ActuatorMotors msg{};
    
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    
                // right motor %   left motor %
    msg.control = {     0.1f,           0.1f,      0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};    

    actuator_motors_publisher->publish(msg);
}



void key_control::stop()
{
    ActuatorMotors msg{};
    
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.control = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};    

    actuator_motors_publisher->publish(msg);
}



void key_control::right_turn()
{
    ActuatorMotors msg{};
    
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.control = {0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};    

    actuator_motors_publisher->publish(msg);
}



void key_control::left_turn()
{
    ActuatorMotors msg{};
    
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.control = {0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};    

    actuator_motors_publisher->publish(msg);
}



int key_control::getKeyInput()
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);            
    newt = oldt;                               
    newt.c_lflag &= ~(ICANON | ECHO);          
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);   

    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);          
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK); 

    ch = getchar();                            

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);   
    fcntl(STDIN_FILENO, F_SETFL, oldf);        

    if (ch != EOF)
        return ch;                             

    return -1;         
}



void key_control::keyboard_loop()
{
    std::cout << "Press W/A/S/D to move. Press Q to quit.\n";

    while (true)
    {
        int key = getKeyInput();

        switch (key)
        {
            case 'w':
            case 'W':
                go();
                break;
            case 'a':
            case 'A':
                left_turn();
                break;
            case 'd':
            case 'D':
                right_turn();
                break;
            case 's':
            case 'S':
                stop();
				//disarm();
                break;
            case 'q':
            case 'Q':
                std::cout << "Quitting...\n";
                return;
        }

        usleep(100000); // sleep for 100ms to avoid CPU overload
    }

}






int main(int argc, char *argv[])
{
    std::cout << "Starting key_control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<key_control>());

    rclcpp::shutdown();
    return 0;
}
