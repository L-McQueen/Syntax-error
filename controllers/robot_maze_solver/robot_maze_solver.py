import math
import time
import collections
import statistics
import threading
import cv2
import numpy as np
from controller import Robot

# 1. CONSTANTS & SETUP
TIME_STEP = 32
CELL_SIZE = 0.30
WHEEL_RADIUS = 0.02
MAX_SPEED = 6.28

# 2. HELPER MATH FUNCTIONS
def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

def get_compass_heading(yaw_rad):
    deg = math.degrees(yaw_rad)
    return int(round(deg / 90.0) * 90) % 360

# 3. VISION PROCESSOR THREAD
class VisionProcessor(threading.Thread):
    def __init__(self, solver):
        super().__init__()
        self.solver = solver
        self.daemon = True
        self.running = True
        self.last_color_printed = None
        self.last_aruco_printed = None
        
    def run(self):
        while self.running:
            # 1. Process Upper Camera (ArUco)
            with self.solver.lock:
                upper_frame = self.solver.upper_frame_data
                lower_frame = self.solver.lower_frame_data
                
            if upper_frame is not None:
                gray = cv2.cvtColor(upper_frame, cv2.COLOR_BGRA2GRAY)
                # OpenCV 4.7+ / 5.0 format
                aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
                parameters = cv2.aruco.DetectorParameters()
                detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
                corners, ids, _ = detector.detectMarkers(gray)
                
                disp_upper = cv2.cvtColor(upper_frame, cv2.COLOR_BGRA2BGR)
                if ids is not None and len(ids) > 0:
                    cv2.aruco.drawDetectedMarkers(disp_upper, corners, ids)
                    
                    marker_id = int(np.ravel(ids)[0])
                    if self.last_aruco_printed != marker_id:
                        print(f"[VISION] ARUCO FOUND! ID: {marker_id}")
                        self.last_aruco_printed = marker_id
                        
                    if marker_id == self.solver.last_aruco_id:
                        self.solver.aruco_consecutive += 1
                    else:
                        self.solver.last_aruco_id = marker_id
                        self.solver.aruco_consecutive = 1
                        
                    if self.solver.aruco_consecutive >= 3:
                        with self.solver.lock:
                            self.solver.shared_aruco_text = f"ARUCO ID: {marker_id}"
                            self.solver.aruco_clear_time = time.time() + 3.0
                else:
                    self.solver.aruco_consecutive = 0
                    self.last_aruco_printed = None
                    
                cv2.imshow("Upper Camera (ArUco)", disp_upper)
                
            # 2. Process Lower Camera (Floor Color)
            if lower_frame is not None:
                hsv = cv2.cvtColor(lower_frame, cv2.COLOR_BGRA2BGR)
                hsv = cv2.cvtColor(hsv, cv2.COLOR_BGR2HSV)
                
                h, w = hsv.shape[:2]
                cx, cy = w // 2, h // 2
                roi_size = 40
                roi = hsv[cy-roi_size:cy+roi_size, cx-roi_size:cx+roi_size]
                
                mask_red1 = cv2.inRange(roi, np.array([0, 120, 120]), np.array([10, 255, 255]))
                mask_red2 = cv2.inRange(roi, np.array([160, 120, 120]), np.array([180, 255, 255]))
                mask_red = cv2.bitwise_or(mask_red1, mask_red2)
                
                mask_yellow = cv2.inRange(roi, np.array([20, 120, 120]), np.array([40, 255, 255]))
                mask_orange = cv2.inRange(roi, np.array([11, 120, 120]), np.array([19, 255, 255]))
                mask_cyan = cv2.inRange(roi, np.array([85, 120, 120]), np.array([105, 255, 255]))
                mask_magenta = cv2.inRange(roi, np.array([130, 120, 120]), np.array([160, 255, 255]))
                
                color_name = None
                is_red = False
                threshold = 2000
                
                if cv2.countNonZero(mask_red) > threshold:
                    color_name = "RED"
                    is_red = True
                elif cv2.countNonZero(mask_yellow) > threshold:
                    color_name = "YELLOW"
                elif cv2.countNonZero(mask_orange) > threshold:
                    color_name = "ORANGE"
                elif cv2.countNonZero(mask_cyan) > threshold:
                    color_name = "CYAN"
                elif cv2.countNonZero(mask_magenta) > threshold:
                    color_name = "MAGENTA"
                        
                if color_name:
                    if self.last_color_printed != color_name:
                        print(f"[VISION] Floor Color Detected: {color_name}")
                        self.last_color_printed = color_name
                        
                    if color_name != "RED":
                        with self.solver.lock:
                            self.solver.shared_color_text = f"COLOR: {color_name}"
                            
                with self.solver.lock:
                    if is_red and self.solver.state in ["SCAN", "DECIDE", "MOVE"]:
                        self.solver.found_red_goal = True
                        
                cv2.imshow("Lower Camera (Floor)", lower_frame)
                
            cv2.waitKey(1)
            time.sleep(0.03)

# 4. MAZE SOLVER CLASS & STATE MACHINE
class MazeSolver:
    def __init__(self):
        self.robot = Robot()
        
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        self.left_encoder = self.robot.getDevice('left wheel sensor')
        self.right_encoder = self.robot.getDevice('right wheel sensor')
        self.left_encoder.enable(TIME_STEP)
        self.right_encoder.enable(TIME_STEP)
        
        self.tof_front = self.robot.getDevice('tof_front')
        self.tof_left = self.robot.getDevice('tof_left')
        self.tof_right = self.robot.getDevice('tof_right')
        self.tof_front.enable(TIME_STEP)
        self.tof_left.enable(TIME_STEP)
        self.tof_right.enable(TIME_STEP)
        
        self.imu = self.robot.getDevice('imu')
        self.imu.enable(TIME_STEP)
        
        self.upper_camera = self.robot.getDevice('main_camera')
        self.upper_camera.enable(TIME_STEP)
        
        self.lower_camera = self.robot.getDevice('camera')
        self.lower_camera.enable(TIME_STEP)
        
        self.oled = self.robot.getDevice('oled_display')
        
        self.state = "CALIBRATE"
        self.grid_x = 0
        self.grid_y = 0
        self.visited = {(0, 0)}
        self.route_stack = [(0, 0)]
        
        self.yaw_offset = 0.0
        self.target_yaw = 0.0
        
        self.start_l = 0.0
        self.start_r = 0.0
        
        self.front_tof_buffer = collections.deque(maxlen=5)
        self.left_tof_buffer = collections.deque(maxlen=5)
        self.right_tof_buffer = collections.deque(maxlen=5)
        self.wall_detected = False
        
        self.lock = threading.Lock()
        self.upper_frame_data = None
        self.lower_frame_data = None
        
        self.shared_aruco_text = ""
        self.shared_color_text = ""
        self.found_red_goal = False
        
        self.last_aruco_id = -1
        self.aruco_consecutive = 0
        self.aruco_clear_time = 0.0
        
        self.vision_thread = VisionProcessor(self)
        self.vision_thread.start()

    def get_grid_offset(self, target_heading):
        if target_heading == 0: return (1, 0)
        elif target_heading == 90: return (0, 1)
        elif target_heading == 180: return (-1, 0)
        elif target_heading == 270 or target_heading == -90: return (0, -1)
        return (0, 0)
        
    def get_heading_to_target(self, curr_x, curr_y, target_x, target_y):
        dx = target_x - curr_x
        dy = target_y - curr_y
        if dx == 0 and dy == 1: return 90
        if dx == 1 and dy == 0: return 0
        if dx == 0 and dy == -1: return 270
        if dx == -1 and dy == 0: return 180
        return 0
        
    def get_filtered_front_tof(self):
        if len(self.front_tof_buffer) > 0:
            return statistics.median(self.front_tof_buffer)
        return 0.0

    def get_filtered_left_tof(self):
        if len(self.left_tof_buffer) > 0:
            return statistics.median(self.left_tof_buffer)
        return 0.0

    def get_filtered_right_tof(self):
        if len(self.right_tof_buffer) > 0:
            return statistics.median(self.right_tof_buffer)
        return 0.0
        
    def update_oled(self):
        if not self.oled: return
        w = self.oled.getWidth()
        h = self.oled.getHeight()
        self.oled.setColor(0x000000)
        self.oled.fillRectangle(0, 0, w, h)
        
        with self.lock:
            aruco_txt = self.shared_aruco_text
            color_txt = self.shared_color_text
            if time.time() > self.aruco_clear_time:
                self.shared_aruco_text = ""
                
        self.oled.setFont("Arial", 10, True)
        if aruco_txt:
            self.oled.setColor(0x00FF00) # Green
            self.oled.drawText(aruco_txt, 5, 10)
        if color_txt:
            self.oled.setColor(0xFFFFFF) # White
            self.oled.drawText(color_txt, 5, 30)

    def update(self):
        t = self.robot.getTime()
        raw_yaw = self.imu.getRollPitchYaw()[2]
        current_yaw = normalize_angle(raw_yaw - self.yaw_offset)
        
        val_f = self.tof_front.getValue()
        self.front_tof_buffer.append(val_f / 1000.0 if val_f > 10.0 else val_f)
        
        val_l = self.tof_left.getValue()
        self.left_tof_buffer.append(val_l / 1000.0 if val_l > 10.0 else val_l)
        
        val_r = self.tof_right.getValue()
        self.right_tof_buffer.append(val_r / 1000.0 if val_r > 10.0 else val_r)
        
        upper_img = self.upper_camera.getImage()
        lower_img = self.lower_camera.getImage()
        
        with self.lock:
            if upper_img:
                self.upper_frame_data = np.frombuffer(upper_img, np.uint8).reshape((self.upper_camera.getHeight(), self.upper_camera.getWidth(), 4))
            if lower_img:
                self.lower_frame_data = np.frombuffer(lower_img, np.uint8).reshape((self.lower_camera.getHeight(), self.lower_camera.getWidth(), 4))
                
        self.update_oled()
        
        if self.state == "CALIBRATE":
            if t < 3.0:
                self.left_motor.setVelocity(0.0)
                self.right_motor.setVelocity(0.0)
            else:
                self.yaw_offset = raw_yaw
                print("[CALIBRATE] Calibration complete.")
                self.state = "SCAN"
                
        elif self.state == "SCAN":
            with self.lock:
                do_return = self.found_red_goal
            if do_return:
                print("\n[MAZE] RED TILE CENTER REACHED! Triggering RETURN_HOME...\n")
                self.state = "RETURN_HOME"
                return
            self.left_motor.setVelocity(0.0)
            self.right_motor.setVelocity(0.0)
            self.state = "DECIDE"
            
        elif self.state == "DECIDE":
            val_r = self.tof_right.getValue()
            val_l = self.tof_left.getValue()
            
            filtered_df = self.get_filtered_front_tof()
            dr = val_r / 1000.0 if val_r > 10.0 else val_r
            dl = val_l / 1000.0 if val_l > 10.0 else val_l
            
            robot_heading = get_compass_heading(current_yaw)
            directions = [
                (filtered_df, robot_heading),
                (dr, robot_heading - 90),
                (dl, robot_heading + 90)
            ]
            
            valid_neighbors = []
            for tof_dist, target_heading in directions:
                if tof_dist > 0.25:
                    normalized_target_heading = target_heading % 360
                    dx, dy = self.get_grid_offset(normalized_target_heading)
                    nx = self.grid_x + dx
                    ny = self.grid_y + dy
                    if (nx, ny) not in self.visited:
                        valid_neighbors.append((nx, ny))
            
            if valid_neighbors:
                chosen_nx, chosen_ny = valid_neighbors[0]
                self.route_stack.append((chosen_nx, chosen_ny))
                self.visited.add((chosen_nx, chosen_ny))
                heading_deg = self.get_heading_to_target(self.grid_x, self.grid_y, chosen_nx, chosen_ny)
                self.target_yaw = math.radians(heading_deg)
                self.grid_x = chosen_nx
                self.grid_y = chosen_ny
                print(f"[DECIDE] EXPLORE: Target ({chosen_nx}, {chosen_ny})")
                self.state = "TURN"
            else:
                if len(self.route_stack) > 0:
                    self.route_stack.pop()
                if len(self.route_stack) == 0:
                    print("[DECIDE] Maze unsolvable!")
                    self.state = "FINISHED"
                else:
                    parent_x, parent_y = self.route_stack[-1]
                    heading_deg = self.get_heading_to_target(self.grid_x, self.grid_y, parent_x, parent_y)
                    self.target_yaw = math.radians(heading_deg)
                    self.grid_x = parent_x
                    self.grid_y = parent_y
                    print(f"[DECIDE] BACKTRACK to ({parent_x}, {parent_y})")
                    self.state = "TURN"
                    
        elif self.state == "RETURN_HOME":
            if len(self.route_stack) > 0:
                self.route_stack.pop() # Discard current cell
                
            if len(self.route_stack) > 0:
                target_x, target_y = self.route_stack[-1] # Peek at next cell
                heading_deg = self.get_heading_to_target(self.grid_x, self.grid_y, target_x, target_y)
                self.target_yaw = math.radians(heading_deg)
                self.grid_x = target_x
                self.grid_y = target_y
                print(f"[RETURN_HOME] Reversing to ({target_x}, {target_y})")
                self.state = "TURN"
            else:
                self.left_motor.setVelocity(0.0)
                self.right_motor.setVelocity(0.0)
                print("\n*** SIMULATION COMPLETE: SURVIVED AND RETURNED HOME! ***\n")
                self.state = "FINISHED"
                
        elif self.state == "TURN":
            error = normalize_angle(self.target_yaw - current_yaw)
            turn_speed = 4.0 * error
            if abs(turn_speed) < 1.0:
                turn_speed = math.copysign(1.0, turn_speed)
            self.left_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, -turn_speed)))
            self.right_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, turn_speed)))
            if abs(error) < 0.05:
                self.left_motor.setVelocity(0.0)
                self.right_motor.setVelocity(0.0)
                self.start_l = self.left_encoder.getValue()
                self.start_r = self.right_encoder.getValue()
                self.wall_detected = False
                self.state = "MOVE"
                
        elif self.state == "MOVE":
            cur_l = self.left_encoder.getValue()
            cur_r = self.right_encoder.getValue()
            dist = (abs(cur_l - self.start_l) + abs(cur_r - self.start_r)) / 2.0 * WHEEL_RADIUS
            
            error = normalize_angle(self.target_yaw - current_yaw)
            
            # Active Centering Logic
            centering_correction = 0.0
            K_center = 5.0
            left_dist = self.get_filtered_left_tof()
            right_dist = self.get_filtered_right_tof()
            
            left_valid = left_dist < 0.20
            right_valid = right_dist < 0.20
            
            if left_valid and right_valid:
                centering_error = left_dist - right_dist 
                centering_correction = K_center * centering_error
            elif left_valid:
                centering_error = left_dist - 0.11
                centering_correction = K_center * centering_error
            elif right_valid:
                centering_error = 0.11 - right_dist
                centering_correction = K_center * centering_error
            
            left_s = 5.0 - (2.0 * error) - centering_correction
            right_s = 5.0 + (2.0 * error) + centering_correction
            
            self.left_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, left_s)))
            self.right_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, right_s)))
            
            front_dist = self.get_filtered_front_tof()
            
            if dist >= CELL_SIZE:
                self.left_motor.setVelocity(0.0)
                self.right_motor.setVelocity(0.0)
                with self.lock:
                    self.state = "SCAN"
            else:
                if dist > 0.15:
                    if front_dist < 0.12:
                        self.wall_detected = True
                    elif front_dist > 0.15:
                        self.wall_detected = False
                        
                    if self.wall_detected:
                        self.left_motor.setVelocity(0.0)
                        self.right_motor.setVelocity(0.0)
                        print(f"[MOVE] Emergency Stop! Front={front_dist:.3f}m.")
                        if front_dist < 0.07:
                            self.state = "BACKUP"
                        else:
                            with self.lock:
                                self.state = "SCAN"
                        self.wall_detected = False
                        
        elif self.state == "BACKUP":
            self.left_motor.setVelocity(-3.0)
            self.right_motor.setVelocity(-3.0)
            if self.get_filtered_front_tof() > 0.11:
                self.left_motor.setVelocity(0.0)
                self.right_motor.setVelocity(0.0)
                print("[BACKUP] Reversing complete.")
                with self.lock:
                    self.state = "SCAN"

        elif self.state == "FINISHED":
            self.left_motor.setVelocity(0.0)
            self.right_motor.setVelocity(0.0)

if __name__ == '__main__':
    solver = MazeSolver()
    while solver.robot.step(TIME_STEP) != -1:
        solver.update()
