"""
Controlador del Robot para la Pista B (Niveles) - Webots Sim2Real
Candidates Principiantes 2026

Mismo hardware y configuración física que Pista A:
- Motores diferenciales N20 comandados a 7.8V
- Sensores ToF láser (Frontal VL53L1X, Laterales VL53L0X)
- IMU MPU-6050 (Acelerómetro + Giróscopo)
- Cámaras (Webcam frontal + PixyMon picada al suelo)
- Pantalla OLED telemetría
- Scoop frontal para transporte de pelota
"""

from controller import Robot
import math

class TrackBRobot(Robot):
    def __init__(self):
        super().__init__()
        self.time_step = int(self.getBasicTimeStep())
        
        # --- Motores ---
        self.left_motor = self.getDevice("left wheel motor")
        self.right_motor = self.getDevice("right wheel motor")
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # --- Sensores de Distancia ToF ---
        self.tof_front = self.getDevice("tof_front")
        self.tof_left = self.getDevice("tof_left")
        self.tof_right = self.getDevice("tof_right")
        self.tof_front.enable(self.time_step)
        self.tof_left.enable(self.time_step)
        self.tof_right.enable(self.time_step)
        
        # --- IMU y Sensores Inerciales ---
        self.imu = self.getDevice("imu")
        self.imu.enable(self.time_step)
        self.gyro = self.getDevice("gyro")
        self.gyro.enable(self.time_step)
        
        # --- Cámaras ---
        self.camera_floor = self.getDevice("camera")      # Pixy / picada al suelo
        self.camera_front = self.getDevice("main_camera") # Frontal
        self.camera_floor.enable(self.time_step * 2)
        self.camera_front.enable(self.time_step * 2)
        
        # --- Display OLED ---
        self.display = self.getDevice("oled_display")
        self.update_display("PISTA B: READY", "INICIO")
        
        print("Robot Track B Solver inicializado correctamente.")

    def update_display(self, line1, line2=""):
        if self.display:
            self.display.setColor(0x000000)
            self.display.fillRectangle(0, 0, 128, 64)
            self.display.setColor(0x00FF00)
            self.display.setFont("Arial", 10, True)
            self.display.drawText(line1, 4, 12)
            if line2:
                self.display.setColor(0xFFFFFF)
                self.display.drawText(line2, 4, 34)

    def run(self):
        step_count = 0
        while self.step(self.time_step) != -1:
            step_count += 1
            # Telemetría cada segundo
            if step_count % 30 == 0:
                roll, pitch, yaw = self.imu.getRollPitchYaw()
                d_front = self.tof_front.getValue() / 1000.0
                d_left = self.tof_left.getValue() / 1000.0
                d_right = self.tof_right.getValue() / 1000.0
                # Display telemetría
                self.update_display(f"Yaw: {math.degrees(yaw):.1f}°", f"ToF F:{d_front:.2f}m")

if __name__ == "__main__":
    robot = TrackBRobot()
    robot.run()
