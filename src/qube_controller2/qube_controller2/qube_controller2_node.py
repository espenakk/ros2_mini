#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from time import time


class QubeControllerNode(Node):
    def __init__(self):
        super().__init__('qube_pid_controller')
        
        # Parameters for the PID controller (these can be made configurable)
        self.declare_parameter('p_gain', 1.0)
        self.declare_parameter('i_gain', 0.0)
        self.declare_parameter('d_gain', 0.1)
        self.declare_parameter('setpoint', 0.0)  # Desired position
        
        # Get parameter values
        self.p_gain = self.get_parameter('p_gain').value
        self.i_gain = self.get_parameter('i_gain').value
        self.d_gain = self.get_parameter('d_gain').value
        self.setpoint = self.get_parameter('setpoint').value
        
        # PID controller state variables
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time()
        
        # Create subscriber to joint_states
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10)
            
        # Create publisher for velocity commands
        self.vel_pub = self.create_publisher(
            Float64MultiArray,
            '/velocity_controller/command',
            10)
            
        self.get_logger().info('Qube PID controller initialized')
        
    def joint_states_callback(self, msg):
        # Extract position and velocity from joint_states
        if len(msg.position) > 0 and len(msg.velocity) > 0:
            position = msg.position[0]  # Assuming first joint is our target
            velocity = msg.velocity[0]
            
            # Calculate control output using PID
            control_output = self.compute_pid(position)
            
            # Create and publish velocity command
            self.publish_velocity_command(control_output)
            
    def compute_pid(self, position):
        # Calculate time delta
        current_time = time()
        dt = current_time - self.prev_time
        
        if dt <= 0:
            return 0.0
            
        # Calculate error
        error = self.setpoint - position
        
        # Proportional term
        p_term = self.p_gain * error
        
        # Integral term with anti-windup (simple limiting)
        self.integral += error * dt
        i_term = self.i_gain * self.integral
        
        # Derivative term
        derivative = (error - self.prev_error) / dt
        d_term = self.d_gain * derivative
        
        # Calculate total output
        output = p_term + i_term + d_term
        
        # Update state for next iteration
        self.prev_error = error
        self.prev_time = current_time
        
        return output
    
    def publish_velocity_command(self, velocity):
        # Create Float64MultiArray message
        msg = Float64MultiArray()
        # Initialize the data field as a list with our velocity value
        msg.data = [velocity]
        # Publish the message
        self.vel_pub.publish(msg)
        self.get_logger().debug(f'Published velocity command: {velocity}')


def main(args=None):
    rclpy.init(args=args)
    node = QubeControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
