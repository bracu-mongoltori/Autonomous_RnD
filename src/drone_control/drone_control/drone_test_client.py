#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import time


class DroneTestClient(Node):
    def __init__(self):
        super().__init__('drone_test_client')
        
        # Publishers
        self.arm_disarm_pub = self.create_publisher(Bool, '/arm_disarm', 10)
        self.takeoff_land_pub = self.create_publisher(String, '/takeoff_land', 10)
        
        # Subscriber for status
        self.status_sub = self.create_subscription(
            String,
            '/drone_status',
            self.status_callback,
            10
        )
        
        self.get_logger().info("Drone Test Client Started")
        self.get_logger().info("Available commands:")
        self.get_logger().info("  - arm: Arm the drone")
        self.get_logger().info("  - disarm: Disarm the drone")
        self.get_logger().info("  - takeoff: Start motors (pseudo takeoff)")
        self.get_logger().info("  - land: Stop motors")
        self.get_logger().info("  - status: Show current status")
        self.get_logger().info("  - test: Run automatic test sequence")
        self.get_logger().info("  - quit: Exit")
        
        # Start interactive mode
        self.run_interactive_mode()
    
    def status_callback(self, msg):
        """Handle status updates from drone controller"""
        pass  # Status is printed when requested
    
    def run_interactive_mode(self):
        """Run interactive command mode"""
        while rclpy.ok():
            try:
                command = input("\nEnter command: ").strip().lower()
                
                if command == "arm":
                    self.arm_drone()
                elif command == "disarm":
                    self.disarm_drone()
                elif command == "takeoff":
                    self.takeoff()
                elif command == "land":
                    self.land()
                elif command == "status":
                    self.get_logger().info("Status updates are published to /drone_status topic")
                elif command == "test":
                    self.run_test_sequence()
                elif command == "quit":
                    break
                else:
                    self.get_logger().warn(f"Unknown command: {command}")
                
                # Small delay to ensure message is sent
                time.sleep(0.1)
                
            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
    
    def arm_drone(self):
        """Send arm command"""
        msg = Bool()
        msg.data = True
        self.arm_disarm_pub.publish(msg)
        self.get_logger().info("Sent ARM command")
    
    def disarm_drone(self):
        """Send disarm command"""
        msg = Bool()
        msg.data = False
        self.arm_disarm_pub.publish(msg)
        self.get_logger().info("Sent DISARM command")
    
    def takeoff(self):
        """Send takeoff command"""
        msg = String()
        msg.data = "takeoff"
        self.takeoff_land_pub.publish(msg)
        self.get_logger().info("Sent TAKEOFF command")
    
    def land(self):
        """Send land command"""
        msg = String()
        msg.data = "land"
        self.takeoff_land_pub.publish(msg)
        self.get_logger().info("Sent LAND command")
    
    def run_test_sequence(self):
        """Run automatic test sequence"""
        self.get_logger().info("Starting automatic test sequence...")
        
        # Test sequence
        sequences = [
            ("ARM", self.arm_drone),
            ("TAKEOFF", self.takeoff),
            ("LAND", self.land),
            ("DISARM", self.disarm_drone)
        ]
        
        for step_name, step_func in sequences:
            self.get_logger().info(f"Step: {step_name}")
            step_func()
            time.sleep(3)  # Wait 3 seconds between steps
        
        self.get_logger().info("Test sequence completed!")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        test_client = DroneTestClient()
        # The interactive mode is handled inside the constructor
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
