#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
import csv
import time
import os
from datetime import datetime

class RobotController(Node):
    def __init__(self):
        super().__init__("robot_controller")
        
        # 1. Subscriber la datele de Percepție (Generic)
        # Ascultă un punct: x (deviație stânga/dreapta), z (distanță)
        self.target_sub = self.create_subscription(
            Point, 
            "/pose_estimation/image_result", 
            self.control_callback, 
            10
        )
        
        # 2. Publisher către Robot
        self.cmd_pub = self.create_publisher(
            Twist, 
            "/diff_drive_controller/cmd_vel", 
            10
        )

        # 3. Configurare Logging (Excel/CSV)
        # Numele modelului este pasat ca parametru la rulare
        self.declare_parameter("model_name", "unknown_model")
        model_name = self.get_parameter("model_name").value
        
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"log_{model_name}_{timestamp_str}.csv"
        
        # Inițializare fișier CSV
        with open(self.filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            # Header-ul tabelului
            writer.writerow(["Timestamp", "Model", "Detected_Distance_m", "Horizontal_Error", "Linear_Vel", "Angular_Vel"])
            
        self.get_logger().info(f"Controller pornit. Logare in: {self.filename}")

    def control_callback(self, msg):
        """
        msg.x = deviația orizontală (-0.5 stânga ... 0.5 dreapta)
        msg.z = distanța până la persoană (metri)
        """
        
        # --- LOGICA DE MIȘCARE ---
        Kp_turn = -2.0
        Kp_forward = 0.4
        desired_distance = 1.0

        turn_speed = Kp_turn * msg.x
        forward_speed = Kp_forward * (msg.z - desired_distance)

        # clamp (FOARTE IMPORTANT)
        forward_speed = max(min(forward_speed, 0.5), -0.2)
        turn_speed = max(min(turn_speed, 1.0), -1.0)
        
        detected = False

        if msg.z > 0.0 and msg.z < 5.0:
            forward_speed = 0.0
            turn_speed = 0.0
            detected = True

            self.writer.writerow([
                time.time(),
                self.get_parameter("model_name").value,
                msg.z,
                msg.x,
                forward_speed,
                turn_speed
            ])
            self.file.flush()

            self.get_logger().info(f"🎯 Detectat la {msg.z:.2f} m")
            sys.exit(0)

        else:
            forward_speed = 1.0
            turn_speed = 0.0

        # Trimitere comandă
        cmd = Twist()
        cmd.linear.x = float(forward_speed)
        cmd.angular.z = float(turn_speed)
        cmd.header.frame_id = "base_link"
        self.cmd_pub.publish(cmd)

        # --- LOGARE ÎN EXCEL ---
        # Salvăm doar când detectăm ceva, sau tot timpul (depinde de preferință)
        with open(self.filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                time.time(),
                self.get_parameter("model_name").value,
                msg.z if detected else "Thinking/Searching",
                msg.x if detected else "N/A",
                forward_speed,
                turn_speed
            ])

def main():
    rclpy.init()
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()