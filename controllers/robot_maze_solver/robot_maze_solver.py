import math
import time
import collections
import statistics
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

# 3. MAZE SOLVER CLASS & STATE MACHINE
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
        self.wall_detected = False

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

    def update(self):
        t = self.robot.getTime()
        raw_yaw = self.imu.getRollPitchYaw()[2]
        current_yaw = normalize_angle(raw_yaw - self.yaw_offset)
        
        val_f = self.tof_front.getValue()
        df = val_f / 1000.0 if val_f > 10.0 else val_f
        self.front_tof_buffer.append(df)
        
        if self.state == "CALIBRATE":
            if t < 3.0:
                self.left_motor.setVelocity(0.0)
                self.right_motor.setVelocity(0.0)
            else:
                self.yaw_offset = raw_yaw
                print("[CALIBRATE] Calibration complete. Offset:", self.yaw_offset)
                self.state = "SCAN"
                
        elif self.state == "SCAN":
            self.left_motor.setVelocity(0.0)
            self.right_motor.setVelocity(0.0)
            self.state = "DECIDE"
            
        elif self.state == "DECIDE":
            print(f"[DECIDE] Current: ({self.grid_x}, {self.grid_y}) | Visited: {self.visited}")
            
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
                
                print(f"[DECIDE] EXPLORE: Path Found! Target Yaw: {heading_deg}° to ({chosen_nx}, {chosen_ny})")
                self.state = "TURN"
            else:
                # BACKTRACK
                if len(self.route_stack) > 0:
                    self.route_stack.pop() # Remove current dead-end cell
                
                if len(self.route_stack) == 0:
                    print("[DECIDE] Maze unsolvable or back at start! Stopping.")
                    self.left_motor.setVelocity(0.0)
                    self.right_motor.setVelocity(0.0)
                    self.state = "FINISHED"
                else:
                    parent_x, parent_y = self.route_stack[-1]
                    heading_deg = self.get_heading_to_target(self.grid_x, self.grid_y, parent_x, parent_y)
                    self.target_yaw = math.radians(heading_deg)
                    
                    self.grid_x = parent_x
                    self.grid_y = parent_y
                    
                    print(f"[DECIDE] BACKTRACK: Dead-End! Target Yaw: {heading_deg}° to ({parent_x}, {parent_y})")
                    self.state = "TURN"
                
        elif self.state == "TURN":
            error = normalize_angle(self.target_yaw - current_yaw)
            Kp_turn = 4.0
            turn_speed = Kp_turn * error
            
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
                print("[TURN] Complete.")
                self.state = "MOVE"
                
        elif self.state == "MOVE":
            cur_l = self.left_encoder.getValue()
            cur_r = self.right_encoder.getValue()
            
            dist_l = abs(cur_l - self.start_l) * WHEEL_RADIUS
            dist_r = abs(cur_r - self.start_r) * WHEEL_RADIUS
            dist = (dist_l + dist_r) / 2.0
            
            error = normalize_angle(self.target_yaw - current_yaw)
            Kp_move = 2.0
            BASE_SPEED = 5.0
            
            left_s = BASE_SPEED - (Kp_move * error)
            right_s = BASE_SPEED + (Kp_move * error)
            
            self.left_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, left_s)))
            self.right_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, right_s)))
            
            front_dist = self.get_filtered_front_tof()
            
            if dist >= CELL_SIZE:
                self.left_motor.setVelocity(0.0)
                self.right_motor.setVelocity(0.0)
                print(f"[MOVE] Reached {dist:.3f}m. Next cell.")
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
                        print(f"[MOVE] Emergency Stop! Wall detected at {front_dist:.3f}m.")
                        self.state = "SCAN"
                        self.wall_detected = False
                        
        elif self.state == "FINISHED":
            self.left_motor.setVelocity(0.0)
            self.right_motor.setVelocity(0.0)

# 4. EXECUTION
if __name__ == '__main__':
    solver = MazeSolver()
    while solver.robot.step(TIME_STEP) != -1:
        solver.update()
