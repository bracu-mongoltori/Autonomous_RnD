#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from pymavlink import mavutil
import time
import threading


class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        
        # Initialize parameters
        self.declare_parameter('connection_string', '/dev/ttyUSB1')
        self.declare_parameter('baud_rate', 57600)
        
        connection_string = self.get_parameter('connection_string').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        
        # Initialize MAVLink connection
        try:
            self.connection = mavutil.mavlink_connection(connection_string, baud=baud_rate)
            self.get_logger().info(f"Connecting to drone at {connection_string}")
            self.connection.wait_heartbeat()
            self.get_logger().info("Heartbeat received from drone!")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to drone: {e}")
            self.connection = None
        
        # Drone state variables
        self.is_armed = False
        self.is_motors_running = False
        self.motor_speed_percent = 0.1  # 10% motor speed
        self.last_heartbeat_time = time.time()
        self.actual_armed_state = False  # Track actual drone armed state
        self.manual_control_enabled = False  # Track if manual control is active
        
        # Manual control variables (normalized -1.0 to 1.0)
        self.manual_pitch = 0.0    # Forward/Backward (left stick Y)
        self.manual_roll = 0.0     # Left/Right (left stick X)
        self.manual_throttle = 0.0 # Up/Down (right stick Y)
        self.manual_yaw = 0.0      # Rotate (right stick X)
        
        # Subscribers
        self.arm_disarm_sub = self.create_subscription(
            Bool,
            '/arm_disarm',
            self.arm_disarm_callback,
            10
        )
        
        self.takeoff_land_sub = self.create_subscription(
            String,
            '/takeoff_land',
            self.takeoff_land_callback,
            10
        )
        
        # Manual control subscriber (for joystick input)
        self.manual_control_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.manual_control_callback,
            10
        )
        
        # Manual control mode subscriber
        self.manual_mode_sub = self.create_subscription(
            Bool,
            '/manual_control_enable',
            self.manual_mode_callback,
            10
        )
        
        # Publishers for status
        self.status_pub = self.create_publisher(String, '/drone_status', 10)
        
        # Timer for periodic status updates
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # Timer for checking drone state
        self.state_check_timer = self.create_timer(0.5, self.check_drone_state)
        
        self.get_logger().info("Drone Controller Node Started")
        self.get_logger().info("Listening to:")
        self.get_logger().info("  - /arm_disarm (Bool): true to arm, false to disarm")
        self.get_logger().info("  - /takeoff_land (String): 'takeoff' to start motors, 'land' to stop motors")
        self.get_logger().info("  - /cmd_vel (Twist): Manual joystick control (when enabled)")
        self.get_logger().info("  - /manual_control_enable (Bool): Enable/disable manual control mode")
        self.get_logger().info("")
        self.get_logger().info("Xbox Controller Mapping:")
        self.get_logger().info("  - Left Stick: Pitch (Y) / Roll (X)")
        self.get_logger().info("  - Right Stick: Throttle (Y) / Yaw (X)")
        self.get_logger().info("  - Use /manual_control_enable topic to activate joystick control")
        
    def arm_disarm_callback(self, msg):
        """Handle arm/disarm commands"""
        if msg.data and not self.actual_armed_state:
            self.arm_drone()
        elif not msg.data and self.actual_armed_state:
            self.disarm_drone()
        elif msg.data and self.actual_armed_state:
            self.get_logger().info("Drone is already armed")
        elif not msg.data and not self.actual_armed_state:
            self.get_logger().info("Drone is already disarmed")
    
    def takeoff_land_callback(self, msg):
        """Handle takeoff/land commands"""
        command = msg.data.lower()
        
        if command == "takeoff":
            if self.actual_armed_state:
                self.start_motors()
            else:
                self.get_logger().warn("Cannot takeoff: Drone is not armed!")
        elif command == "land":
            self.stop_motors()
        else:
            self.get_logger().warn(f"Unknown command: {command}. Use 'takeoff' or 'land'")
    
    def manual_mode_callback(self, msg):
        """Handle manual control mode enable/disable"""
        if msg.data and not self.manual_control_enabled:
            if self.actual_armed_state:
                self.enable_manual_control()
            else:
                self.get_logger().warn("Cannot enable manual control: Drone is not armed!")
        elif not msg.data and self.manual_control_enabled:
            self.disable_manual_control()
        elif msg.data and self.manual_control_enabled:
            self.get_logger().info("Manual control is already enabled")
        elif not msg.data and not self.manual_control_enabled:
            self.get_logger().info("Manual control is already disabled")
    
    def manual_control_callback(self, msg):
        """Handle manual control input from joystick"""
        if not self.manual_control_enabled or not self.actual_armed_state:
            return
        
        # Map Twist message to drone control (Xbox controller layout)
        # Linear.x = forward/backward (left stick Y) -> pitch
        # Linear.y = left/right (left stick X) -> roll  
        # Linear.z = up/down (right stick Y) -> throttle
        # Angular.z = rotate (right stick X) -> yaw
        
        self.manual_pitch = max(-1.0, min(1.0, msg.linear.x))    # Limit to [-1, 1]
        self.manual_roll = max(-1.0, min(1.0, msg.linear.y))     # Limit to [-1, 1] 
        self.manual_throttle = max(-1.0, min(1.0, msg.linear.z)) # Limit to [-1, 1]
        self.manual_yaw = max(-1.0, min(1.0, msg.angular.z))     # Limit to [-1, 1]
        
        # Send manual control commands to drone
        self.send_manual_control()
    
    def enable_manual_control(self):
        """Enable manual joystick control"""
        if self.connection is None:
            self.get_logger().error("No connection to drone")
            return
        
        try:
            # Set to manual mode for full control
            self.connection.mav.set_mode_send(
                self.connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                19  # Manual mode for ArduCopter
            )
            
            time.sleep(0.2)  # Wait for mode change
            
            # Stop any existing motor timers
            if hasattr(self, 'throttle_timer'):
                self.throttle_timer.cancel()
            
            self.manual_control_enabled = True
            self.is_motors_running = True  # Consider manual control as motors running
            
            # Start manual control timer
            self.manual_control_timer = self.create_timer(0.05, self.send_manual_control)  # 20Hz
            
            self.get_logger().info("Manual control ENABLED - Use joystick to control drone")
            self.get_logger().info("Controls: Left stick=Pitch/Roll, Right stick=Throttle/Yaw")
            
        except Exception as e:
            self.get_logger().error(f"Failed to enable manual control: {e}")
    
    def disable_manual_control(self):
        """Disable manual joystick control"""
        try:
            # Stop manual control timer
            if hasattr(self, 'manual_control_timer'):
                self.manual_control_timer.cancel()
            
            # Reset control inputs
            self.manual_pitch = 0.0
            self.manual_roll = 0.0
            self.manual_throttle = 0.0
            self.manual_yaw = 0.0
            
            # Send neutral control commands
            for i in range(5):
                self.connection.mav.manual_control_send(
                    self.connection.target_system,
                    0,    # pitch
                    0,    # roll
                    1000, # throttle (minimum)
                    0,    # yaw
                    0     # buttons
                )
                time.sleep(0.05)
            
            # Switch back to stabilize mode
            self.connection.mav.set_mode_send(
                self.connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                0  # Stabilize mode
            )
            
            self.manual_control_enabled = False
            self.is_motors_running = False
            
            self.get_logger().info("Manual control DISABLED - Returned to stabilize mode")
            
        except Exception as e:
            self.get_logger().error(f"Failed to disable manual control: {e}")
    
    def send_manual_control(self):
        """Send manual control commands to drone"""
        if not self.manual_control_enabled or not self.actual_armed_state:
            return
        
        try:
            # Convert normalized inputs to PWM values (1000-2000)
            # Base throttle (1000) + offset based on input
            throttle_pwm = int(1000 + max(0, self.manual_throttle) * 1000)  # Only positive throttle
            
            # Pitch, roll, yaw: center at 1500, +/- 500 range
            pitch_pwm = int(1500 + self.manual_pitch * 500)
            roll_pwm = int(1500 + self.manual_roll * 500) 
            yaw_pwm = int(1500 + self.manual_yaw * 500)
            
            # Send manual control command
            self.connection.mav.manual_control_send(
                self.connection.target_system,
                pitch_pwm,    # x (pitch)
                roll_pwm,     # y (roll)
                throttle_pwm, # z (throttle)
                yaw_pwm,      # r (yaw)
                0             # buttons
            )
            
        except Exception as e:
            self.get_logger().error(f"Failed to send manual control: {e}")
    
    def check_drone_state(self):
        """Check drone state and update internal variables"""
        if self.connection is None:
            return
        
        try:
            # Request heartbeat and system status
            self.connection.mav.request_data_stream_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                1,  # 1 Hz
                1   # Start
            )
            
            # Check for messages
            msg = self.connection.recv_match(type=['HEARTBEAT', 'COMMAND_ACK'], blocking=False)
            if msg:
                if msg.get_type() == 'HEARTBEAT':
                    self.last_heartbeat_time = time.time()
                    # Check armed state from heartbeat
                    armed_state = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                    
                    if armed_state != self.actual_armed_state:
                        self.actual_armed_state = armed_state
                        self.is_armed = armed_state
                        status = "ARMED" if armed_state else "DISARMED"
                        self.get_logger().info(f"Drone state changed: {status}")
                        
                        # If drone auto-disarmed, stop motors
                        if not armed_state and self.is_motors_running:
                            self.is_motors_running = False
                            self.get_logger().info("Drone auto-disarmed, stopping motors")
                
                elif msg.get_type() == 'COMMAND_ACK':
                    # Handle command acknowledgments
                    if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                        if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                            self.get_logger().info("Arm/Disarm command accepted")
                        else:
                            self.get_logger().error(f"Arm/Disarm command failed: {msg.result}")
                            
        except Exception as e:
            self.get_logger().debug(f"State check error: {e}")
    
    def arm_drone(self):
        """Arm the drone"""
        if self.connection is None:
            self.get_logger().error("No connection to drone")
            return
        
        try:
            # Send arm command (without force flag to respect safety settings)
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,  # confirmation
                1,  # param1: 1 to arm, 0 to disarm
                0,  # param2: normal arm (respect safety checks and settings)
                0, 0, 0, 0, 0  # unused parameters
            )
            
            self.get_logger().info("Arm command sent")
            
            # Wait for acknowledgment
            ack_msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=2.0)
            if ack_msg and ack_msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                if ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    self.get_logger().info("Drone arming command accepted")
                    # Let the flight controller handle motor spinning based on its settings
                else:
                    self.get_logger().error(f"Drone arming failed: {ack_msg.result}")
            else:
                self.get_logger().warn("No acknowledgment received for arm command")
                
        except Exception as e:
            self.get_logger().error(f"Failed to arm drone: {e}")
    
    def disarm_drone(self):
        """Disarm the drone"""
        if self.connection is None:
            self.get_logger().error("No connection to drone")
            return
        
        try:
            # Stop motors first if running
            if self.is_motors_running:
                self.stop_motors()
                time.sleep(1.0)  # Wait longer for motors to stop
            
            # Set flight mode back to stabilize/auto to allow disarming
            self.connection.mav.set_mode_send(
                self.connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                0  # Stabilize mode for ArduCopter (safer for disarming)
            )
            
            time.sleep(0.5)  # Wait for mode change
            
            # Send multiple zero throttle commands to ensure clean state
            for i in range(5):
                self.connection.mav.manual_control_send(
                    self.connection.target_system,
                    0,    # x (pitch)
                    0,    # y (roll)
                    900, # z (throttle) - minimum PWM
                    0,    # r (yaw)
                    0     # buttons
                )
                time.sleep(0.1)
            
            # Now send disarm command
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,  # confirmation
                0,  # param1: 1 to arm, 0 to disarm
                0,  # param2: normal disarm (respect safety checks and settings)
                0, 0, 0, 0, 0  # unused parameters
            )
            
            self.get_logger().info("Disarm command sent")
            
            # Wait for acknowledgment
            ack_msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3.0)
            if ack_msg and ack_msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                if ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    self.get_logger().info("Drone disarming command accepted")
                else:
                    error_messages = {
                        1: "TEMPORARILY_REJECTED - Try again",
                        2: "DENIED - Cannot disarm (safety reasons)",
                        3: "UNSUPPORTED - Command not supported",
                        4: "FAILED - Disarm failed (check throttle/mode)",
                        5: "IN_PROGRESS - Disarming in progress"
                    }
                    error_msg = error_messages.get(ack_msg.result, f"Unknown error: {ack_msg.result}")
                    self.get_logger().error(f"Drone disarming failed: {error_msg}")
                    
                    # If disarm failed, try force disarm as last resort
                    if ack_msg.result == 4:  # FAILED
                        self.get_logger().warn("Attempting force disarm as last resort...")
                        self.force_disarm()
            else:
                self.get_logger().warn("No acknowledgment received for disarm command")
                
        except Exception as e:
            self.get_logger().error(f"Failed to disarm drone: {e}")
    
    def force_disarm(self):
        """Force disarm as last resort"""
        try:
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,  # confirmation
                0,  # param1: 0 to disarm
                21196,  # param2: force disarm
                0, 0, 0, 0, 0  # unused parameters
            )
            self.get_logger().info("Force disarm command sent")
            
            # Wait for acknowledgment
            ack_msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3.0)
            if ack_msg and ack_msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                if ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    self.get_logger().info("Force disarm successful")
                else:
                    self.get_logger().error(f"Force disarm also failed: {ack_msg.result}")
                    
        except Exception as e:
            self.get_logger().error(f"Force disarm failed: {e}")
    
    def start_motors(self):
        """Start motors at 5% speed (pseudo takeoff)"""
        if self.connection is None:
            self.get_logger().error("No connection to drone")
            return
        
        if not self.actual_armed_state:
            self.get_logger().error("Cannot start motors: Drone is not armed")
            return
        
        try:
            # Set to manual mode to ensure we have full control
            self.connection.mav.set_mode_send(
                self.connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                19  # Manual mode for ArduCopter
            )
            
            time.sleep(0.2)  # Wait for mode change
            
            # Set throttle to 5% (0.05 normalized, or ~1100 PWM)
            throttle_pwm = int(1000 + (self.motor_speed_percent * 1000))  # 1050 PWM for 5%
            
            # Send manual control (throttle only)
            self.connection.mav.manual_control_send(
                self.connection.target_system,
                0,  # x (pitch)
                0,  # y (roll)
                throttle_pwm,  # z (throttle)
                0,  # r (yaw)
                0   # buttons
            )
            
            self.is_motors_running = True
            self.get_logger().info(f"Motors started at {self.motor_speed_percent*100}% speed (PWM: {throttle_pwm})")
            
            # Create a timer to continuously send throttle commands
            if hasattr(self, 'throttle_timer'):
                self.throttle_timer.cancel()
            self.throttle_timer = self.create_timer(0.1, self.send_throttle_command)
            
        except Exception as e:
            self.get_logger().error(f"Failed to start motors: {e}")
    
    def send_throttle_command(self):
        """Continuously send throttle command while motors are running"""
        if not self.is_motors_running or not self.actual_armed_state:
            if hasattr(self, 'throttle_timer'):
                self.throttle_timer.cancel()
            return
        
        try:
            throttle_pwm = int(1000 + (self.motor_speed_percent * 1000))
            self.connection.mav.manual_control_send(
                self.connection.target_system,
                0,  # x (pitch)
                0,  # y (roll)
                throttle_pwm,  # z (throttle)
                0,  # r (yaw)
                0   # buttons
            )
        except Exception as e:
            self.get_logger().error(f"Failed to send throttle command: {e}")
    
    def stop_motors(self):
        """Stop motors (landing) - return to flight controller control"""
        if self.connection is None:
            self.get_logger().error("No connection to drone")
            return
        
        try:
            # Stop the throttle timer first
            if hasattr(self, 'throttle_timer'):
                self.throttle_timer.cancel()
                self.get_logger().info("Throttle timer stopped")
            
            # Send minimum throttle multiple times to ensure it's received
            for i in range(15):  # More commands for better reliability
                self.connection.mav.manual_control_send(
                    self.connection.target_system,
                    0,    # x (pitch)
                    0,    # y (roll)
                    1000, # z (throttle) - minimum PWM
                    0,    # r (yaw)
                    0     # buttons
                )
                time.sleep(0.05)
            
            # Switch back to stabilize mode to allow safe disarming
            self.connection.mav.set_mode_send(
                self.connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                0  # Stabilize mode for ArduCopter
            )
            
            self.is_motors_running = False
            self.get_logger().info("Motors stopped - returned to stabilize mode")
            
        except Exception as e:
            self.get_logger().error(f"Failed to stop motors: {e}")
    
    def publish_status(self):
        """Publish current drone status"""
        status_msg = String()
        
        if self.connection is None:
            status = "DISCONNECTED"
        elif self.manual_control_enabled:
            status = f"MANUAL CONTROL (P:{self.manual_pitch:.2f} R:{self.manual_roll:.2f} T:{self.manual_throttle:.2f} Y:{self.manual_yaw:.2f})"
        elif self.is_motors_running:
            status = f"FLYING (Motors: {self.motor_speed_percent*100}%)"
        elif self.actual_armed_state:
            status = "ARMED (Ready for takeoff)"
        else:
            status = "DISARMED"
        
        status_msg.data = status
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        drone_controller = DroneController()
        rclpy.spin(drone_controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Cleanup
        if 'drone_controller' in locals():
            try:
                # Stop all timers
                if hasattr(drone_controller, 'throttle_timer'):
                    drone_controller.throttle_timer.cancel()
                if hasattr(drone_controller, 'manual_control_timer'):
                    drone_controller.manual_control_timer.cancel()
                
                # Disable manual control if enabled
                if drone_controller.manual_control_enabled:
                    drone_controller.disable_manual_control()
                    
                # Stop motors if running
                if drone_controller.is_motors_running:
                    drone_controller.stop_motors()
                    time.sleep(1)
                    
                # Disarm if armed
                if drone_controller.actual_armed_state:
                    drone_controller.disarm_drone()
                    
                drone_controller.destroy_node()
            except Exception as e:
                print(f"Cleanup error: {e}")
        rclpy.shutdown()


if __name__ == '__main__':
    main()
