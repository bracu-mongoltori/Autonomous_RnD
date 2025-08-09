#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
import time


class DroneJoystickInterface(Node):
    def __init__(self):
        super().__init__('drone_joystick_interface')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.manual_enable_pub = self.create_publisher(Bool, '/manual_control_enable', 10)
        
        # Subscriber
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        # Joystick state
        self.manual_control_enabled = False
        self.last_button_state = [0] * 12  # Xbox controller has ~12 buttons
        
        # Xbox controller button mapping
        self.BUTTON_A = 0      # Enable/Disable manual control
        self.BUTTON_B = 1      # Emergency stop (disable manual control)
        self.BUTTON_X = 2      # Reserved
        self.BUTTON_Y = 3      # Reserved
        self.BUTTON_LB = 4     # Left bumper
        self.BUTTON_RB = 5     # Right bumper
        self.BUTTON_BACK = 6   # Back button
        self.BUTTON_START = 7  # Start button
        self.BUTTON_GUIDE = 8  # Xbox button
        self.BUTTON_LS = 9     # Left stick button
        self.BUTTON_RS = 10    # Right stick button
        
        # Xbox controller axis mapping
        self.AXIS_LEFT_X = 0   # Left stick X (roll)
        self.AXIS_LEFT_Y = 1   # Left stick Y (pitch)
        self.AXIS_LT = 2       # Left trigger
        self.AXIS_RIGHT_X = 3  # Right stick X (yaw)
        self.AXIS_RIGHT_Y = 4  # Right stick Y (throttle)
        self.AXIS_RT = 5       # Right trigger
        self.AXIS_DPAD_X = 6   # D-pad X
        self.AXIS_DPAD_Y = 7   # D-pad Y
        
        # Control parameters
        self.max_pitch_roll = 1.0  # Maximum pitch/roll input
        self.max_yaw_rate = 1.0    # Maximum yaw rate
        self.max_throttle = 1.0    # Maximum throttle
        self.deadzone = 0.1        # Joystick deadzone
        
        self.get_logger().info("Drone Joystick Interface Started")
        self.get_logger().info("Xbox Controller Mapping:")
        self.get_logger().info("  - A Button: Enable/Disable manual control")
        self.get_logger().info("  - B Button: Emergency stop (disable manual control)")
        self.get_logger().info("  - Left Stick: Pitch (Y) / Roll (X)")
        self.get_logger().info("  - Right Stick: Throttle (Y) / Yaw (X)")
        self.get_logger().info("  - Deadzone: ±0.1")
        
    def apply_deadzone(self, value):
        """Apply deadzone to joystick input"""
        if abs(value) < self.deadzone:
            return 0.0
        else:
            # Scale the remaining range to maintain smooth control
            sign = 1 if value > 0 else -1
            return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)
    
    def joy_callback(self, msg):
        """Handle joystick input"""
        if len(msg.buttons) < 11 or len(msg.axes) < 8:
            self.get_logger().warn("Invalid joystick message format")
            return
        
        # Handle button presses (only on button press, not hold)
        for i, button in enumerate(msg.buttons):
            if button and not self.last_button_state[i]:  # Button just pressed
                self.handle_button_press(i)
        
        self.last_button_state = list(msg.buttons)
        
        # Handle manual control input (only if enabled)
        if self.manual_control_enabled:
            self.send_manual_control(msg)
    
    def handle_button_press(self, button_id):
        """Handle button press events"""
        if button_id == self.BUTTON_A:
            # Toggle manual control
            self.toggle_manual_control()
        elif button_id == self.BUTTON_B:
            # Emergency stop - disable manual control
            if self.manual_control_enabled:
                self.disable_manual_control()
                self.get_logger().warn("EMERGENCY STOP - Manual control disabled!")
        elif button_id == self.BUTTON_START:
            self.get_logger().info("Start button pressed - Reserved for future use")
        elif button_id == self.BUTTON_BACK:
            self.get_logger().info("Back button pressed - Reserved for future use")
    
    def toggle_manual_control(self):
        """Toggle manual control on/off"""
        if self.manual_control_enabled:
            self.disable_manual_control()
        else:
            self.enable_manual_control()
    
    def enable_manual_control(self):
        """Enable manual control mode"""
        msg = Bool()
        msg.data = True
        self.manual_enable_pub.publish(msg)
        self.manual_control_enabled = True
        self.get_logger().info("Manual control ENABLED - Use joysticks to control drone")
    
    def disable_manual_control(self):
        """Disable manual control mode"""
        msg = Bool()
        msg.data = False
        self.manual_enable_pub.publish(msg)
        self.manual_control_enabled = False
        self.get_logger().info("Manual control DISABLED")
    
    def send_manual_control(self, joy_msg):
        """Send manual control commands based on joystick input"""
        try:
            # Get raw joystick values
            left_x = joy_msg.axes[self.AXIS_LEFT_X]    # Roll
            left_y = joy_msg.axes[self.AXIS_LEFT_Y]    # Pitch
            right_x = joy_msg.axes[self.AXIS_RIGHT_X]  # Yaw
            right_y = joy_msg.axes[self.AXIS_RIGHT_Y]  # Throttle
            
            # Apply deadzone and scaling
            roll = self.apply_deadzone(left_x) * self.max_pitch_roll
            pitch = self.apply_deadzone(-left_y) * self.max_pitch_roll  # Invert Y axis
            yaw = self.apply_deadzone(right_x) * self.max_yaw_rate
            throttle = self.apply_deadzone(-right_y) * self.max_throttle  # Invert Y axis, up is positive
            
            # Create Twist message
            twist = Twist()
            twist.linear.x = pitch      # Forward/backward
            twist.linear.y = roll       # Left/right
            twist.linear.z = throttle   # Up/down
            twist.angular.z = yaw       # Rotation
            
            # Publish control command
            self.cmd_vel_pub.publish(twist)
            
        except Exception as e:
            self.get_logger().error(f"Failed to send manual control: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        joystick_interface = DroneJoystickInterface()
        rclpy.spin(joystick_interface)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'joystick_interface' in locals():
            # Disable manual control on shutdown
            if joystick_interface.manual_control_enabled:
                joystick_interface.disable_manual_control()
                time.sleep(0.5)
            joystick_interface.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
