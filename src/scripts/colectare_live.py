#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import csv
import os
import sys

# Importăm mesajul tău customizat
from cbrn_interfaces.msg import PerceptionMetrics

class ColectorLive(Node):
    def __init__(self):
        super().__init__('colector_live_csv')
        self.output_file = 'comparatie_modele.csv'
        self.max_frames = 5
        self.frames_saved = 0

        # Verificăm dacă fișierul există pentru a scrie capul de tabel doar o dată
        fisier_nou = not os.path.exists(self.output_file)
        self.csv_file = open(self.output_file, 'a', newline='')
        self.writer = csv.writer(self.csv_file)

        if fisier_nou:
            self.writer.writerow(['ID_Model', 'Secunde', 'Nanosecunde', 'Timp_Inferenta', 'Scor_Incredere', 'Este_Detectat', 'Estimare_Distanta'])

        # Ne abonăm direct la țeava de date
        self.subscription = self.create_subscription(
            PerceptionMetrics,
            '/pose_estimation/metrics',
            self.callback_mesaj,
            10
        )

    def callback_mesaj(self, msg):
        # ---> MODIFICARE AICI: Am adăugat extragerea variabilei msg.header.frame_id
        self.writer.writerow([
            msg.model_name,
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
            msg.inference_time,
            msg.confidence_score,
            msg.is_detected,
            msg.distance_estimate
        ])
        
        self.frames_saved += 1

        # Când am strâns 10 cadre, închidem fișierul și oprim automat scriptul
        if self.frames_saved >= self.max_frames:
            print(f"✔️ S-au salvat {self.max_frames} cadre pentru '{msg.model_name}'.")
            self.csv_file.close()
            sys.exit(0) # Oprește scriptul cu succes

def main(args=None):
    rclpy.init(args=args)
    colector = ColectorLive()
    
    try:
        rclpy.spin(colector)
    except SystemExit:
        pass # Scriptul s-a închis intenționat
    finally:
        colector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()