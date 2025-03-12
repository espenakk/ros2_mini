#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <chrono>
#include <cmath>
#include "pid_controller_msgs/srv/set_reference.hpp"

using namespace std::chrono_literals;

class pidController
{
public:
    //Variabler
    double p;               //P-ledd
    double i;               //I-ledd
    double d;               //D-ledd
    double reference;       //Referanseverdi
    double voltage;         //Utgang

    pidController(double p_val, double i_val, double d_val)
    {
        p = p_val;
        i = i_val;
        d = d_val;
        reference = 0.0;
        voltage = 0.0;
        previousError = 0.0;
        sumError = 0.0;
    }
    //Henter spenning
    double getVoltage()
    {
        return voltage;
    }

    //Oppdaterer voltage
    void update(double currentMeasurement)
    {
        double error = reference - currentMeasurement;  //Beregner feil
        sumError += error;  //Sum av tidligere feil
        double derivative = error - previousError;  //Endring i feil
        voltage = (p * error) + (i * sumError) + (d * derivative);  //PID-regulator
        previousError = error;  //Lag ny forrige feil
    }
private:
    double previousError;   //Forrige feil (D)
    double sumError;        //Sum av feil (I)
};

class PIDControllerNode : public rclcpp::Node
{
public:
    PIDControllerNode() : Node("pid_controller_node"), pid_(5.0, 0.001, 0.5)
    {
        //Publisher som sender utgangsignal fra pid = voltage
        publish_voltage_ = this->create_publisher<std_msgs::msg::Float64>("voltage", 10);

        //Callback-funksjon som lytter på den innkommende målingen og oppdaterer PID-kontrolleren
        auto measurement_listener = [this](std_msgs::msg::Float64::UniquePtr msg) -> void 
        {
            auto message_fb = msg->data;
            pid_.update(message_fb);

            auto message_voltage = std_msgs::msg::Float64();
            message_voltage.data = pid_.getVoltage();
            this->publish_voltage_->publish(message_voltage);
        };

        //Subscriber som lytter på målt vinkel (angle)
        measured_angle_ = this->create_subscription<std_msgs::msg::Float64>("angle", 10, measurement_listener);

        //Deklarer parameter
        this->declare_parameter("kp", 5.0);
        this->declare_parameter("ki", 0.001);
        this->declare_parameter("kd", 0.5);
        this->declare_parameter("ref", 0.0);

        //Linker parameter og variabler
        this->get_parameter("kp", p_);
        this->get_parameter("ki", i_);
        this->get_parameter("kd", d_);
        this->get_parameter("ref", ref_);        

        //Setter variabler til verdier fra PID-kontroller
        pid_.p = p_;
        pid_.i = i_;
        pid_.d = d_;
        pid_.reference = ref_;

        //Fortell noden at parameterne er endret
        parameter_callback = this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &parameters)
            {
                for (const auto &param : parameters)
                {
                    if (param.get_name() == "kp")
                    {
                        pid_.p = param.as_double();
                    }
                    else if (param.get_name() == "ki")
                    {
                        pid_.i = param.as_double();
                    }
                    else if (param.get_name() == "kd")
                    {
                        pid_.d = param.as_double();
                    }
                    else if (param.get_name() == "ref")
                    {
                        pid_.reference = param.as_double();
                    }
                }
                rcl_interfaces::msg::SetParametersResult result;
                result.set__successful(true);
                return result;
            }
        );
        // Create service for setting reference
        service_ = this->create_service<pid_controller_msgs::srv::SetReference>(
            "pid_controller_node/set_reference", std::bind(&PIDControllerNode::setReference, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to set reference");
    }

private:
    pidController pid_;                      // Instans av PID-kontrolleren
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publish_voltage_; // Publisher for voltage
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr measured_angle_; // Subscriber for measured angle
    double p_;               
    double i_;               
    double d_;               
    double ref_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback;

    // SetReference service callback
    void setReference(const std::shared_ptr<pid_controller_msgs::srv::SetReference::Request> request,
        std::shared_ptr<pid_controller_msgs::srv::SetReference::Response> response)
        {
            if ((-M_PI)<(request->request.data) && (request->request.data)<(M_PI))
            {
            pid_.reference = request->request.data;
            response->success.data = true;
            }
            else 
            response->success.data = false;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request: [%f]", request->request.data);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%d]", response->success.data);
    }
    // Service for setting reference
    rclcpp::Service<pid_controller_msgs::srv::SetReference>::SharedPtr service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDControllerNode>());
    rclcpp::shutdown();
    return 0;
}
