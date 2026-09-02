"""
Webots Sim2Real Maze Supervisor (ENU Z-Up Coordinate System)
Dynamically generates a 5x5 DFS maze, randomizes materials,
places tiles, obstacles (ramps, stairs, speedbumps), and chaotic lighting.
Teleports the robot to the Start (Green) tile on launch.
"""

from controller import Supervisor
import random
import math

# --- Constants ---
GRID_SIZE = 5
CELL_SIZE = 0.3
WALL_HEIGHT = 0.15
WALL_THICKNESS = 0.01
HALF_CELL = CELL_SIZE / 2.0
MAZE_ORIGIN_X = -(GRID_SIZE * CELL_SIZE) / 2.0
MAZE_ORIGIN_Y = -(GRID_SIZE * CELL_SIZE) / 2.0


class MazeGenerator:
    def __init__(self, width, height):
        self.w = width
        self.h = height
        self.grid = [[[True, True, True, True] for _ in range(height)] for _ in range(width)]

    def generate(self):
        """Recursive Backtracker maze generation."""
        stack = []
        visited = set()
        start_x, start_y = random.randint(0, self.w - 1), random.randint(0, self.h - 1)
        stack.append((start_x, start_y))
        visited.add((start_x, start_y))

        moves = {0: (0, 1), 1: (1, 0), 2: (0, -1), 3: (-1, 0)}

        while stack:
            cx, cy = stack[-1]
            neighbors = []
            for d, (dx, dy) in moves.items():
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < self.w and 0 <= ny < self.h and (nx, ny) not in visited:
                    neighbors.append((d, nx, ny))
            if neighbors:
                d, nx, ny = random.choice(neighbors)
                self.grid[cx][cy][d] = False
                self.grid[nx][ny][(d + 2) % 4] = False
                visited.add((nx, ny))
                stack.append((nx, ny))
            else:
                stack.pop()


class MazeSupervisor(Supervisor):
    def __init__(self):
        super().__init__()
        self.timeStep = int(self.getBasicTimeStep())
        self.root_node = self.getRoot()
        self.children_field = self.root_node.getField('children')
        self.node_counter = 0

    def unique_name(self, prefix="obj"):
        self.node_counter += 1
        return f"{prefix}_{self.node_counter}"

    def get_random_wall_appearance(self):
        """All white for now."""
        return 'PBRAppearance { baseColor 0.95 0.95 0.95 roughness 0.9 metalness 0 }'

    def spawn_node(self, vrml_string):
        self.children_field.importMFNodeFromString(-1, vrml_string)

    def cell_center(self, cx, cy):
        x = MAZE_ORIGIN_X + cx * CELL_SIZE + HALF_CELL
        y = MAZE_ORIGIN_Y + cy * CELL_SIZE + HALF_CELL
        return x, y

    # ----- SPAWNERS (ENU) -----

    def spawn_wall(self, x, y, is_horizontal, length):
        name = self.unique_name("wall")
        mat = self.get_random_wall_appearance()
        if is_horizontal:
            size_str = f"{length} {WALL_THICKNESS} {WALL_HEIGHT}"
        else:
            size_str = f"{WALL_THICKNESS} {length} {WALL_HEIGHT}"
        z = WALL_HEIGHT / 2.0
        vrml = (
            f'DEF {name} Solid {{ '
            f'translation {x:.4f} {y:.4f} {z:.4f} '
            f'children [ Shape {{ appearance {mat} geometry Box {{ size {size_str} }} }} ] '
            f'name "{name}" '
            f'contactMaterial "Wall" '
            f'boundingObject Box {{ size {size_str} }} '
            f'}}'
        )
        self.spawn_node(vrml)

    def spawn_floor_tile(self, x, y, r, g, b, tile_name):
        sz = CELL_SIZE * 0.85
        vrml = (
            f'DEF {tile_name} Solid {{ '
            f'translation {x:.4f} {y:.4f} 0.001 '
            f'children [ Shape {{ '
            f'appearance PBRAppearance {{ baseColor {r} {g} {b} roughness 0.8 metalness 0 }} '
            f'geometry Box {{ size {sz:.3f} {sz:.3f} 0.002 }} '
            f'}} ] '
            f'name "{tile_name}" '
            f'}}'
        )
        self.spawn_node(vrml)

    def spawn_speedbump(self, x, y, is_horizontal):
        name = self.unique_name("bump")
        rot_z = 1.5708 if is_horizontal else 0
        # X and Y are horizontal. Z is vertical.
        vrml = (
            f'DEF {name} Solid {{ '
            f'translation {x:.4f} {y:.4f} 0.00 '
            f'rotation 0 0 1 {rot_z:.4f} '
            f'children [ Shape {{ '
            f'appearance PBRAppearance {{ baseColor 0.1 0.1 0.1 roughness 0.6 metalness 0 }} '
            f'geometry IndexedFaceSet {{ '
            f'  coord Coordinate {{ point [ '
            f'    -0.15 0.05 0, 0.15 0.05 0, 0.15 -0.05 0, -0.15 -0.05 0, '
            f'    -0.15 0.044 0.006, 0.15 0.044 0.006, 0.15 -0.044 0.006, -0.15 -0.044 0.006 '
            f'  ] }} '
            f'  coordIndex [ '
            f'    0, 1, 2, 3, -1, 4, 7, 6, 5, -1, '
            f'    1, 5, 6, 2, -1, 3, 7, 4, 0, -1, '
            f'    0, 4, 5, 1, -1, 2, 6, 7, 3, -1 '
            f'  ] '
            f'}} '
            f'}} ] '
            f'name "{name}" '
            f'boundingObject Box {{ size 0.30 0.10 0.006 }} '
            f'}}'
        )
        self.spawn_node(vrml)

    def spawn_ramp(self, x, y, is_horizontal):
        name = self.unique_name("ramp")
        rot_z = 1.5708 if is_horizontal else 0
        h = 0.0268
        
        vrml = (
            f'DEF {name} Solid {{ '
            f'translation {x:.4f} {y:.4f} 0 '
            f'rotation 0 0 1 {rot_z:.4f} '
            f'children [ Shape {{ '
            f'appearance PBRAppearance {{ baseColor 0.5 0.5 0.5 roughness 0.8 metalness 0.2 }} '
            f'geometry IndexedFaceSet {{ '
            f'  coord Coordinate {{ point [ '
            f'    -0.15 0.15 0, 0.15 0.15 0, 0.15 -0.15 0, -0.15 -0.15 0, '
            f'    -0.15 0.05 {h}, 0.15 0.05 {h}, 0.15 -0.05 {h}, -0.15 -0.05 {h} '
            f'  ] }} '
            f'  coordIndex [ '
            f'    0, 1, 2, 3, -1, 4, 7, 6, 5, -1, '
            f'    1, 5, 6, 2, -1, 3, 7, 4, 0, -1, '
            f'    0, 4, 5, 1, -1, 2, 6, 7, 3, -1 '
            f'  ] '
            f'}} '
            f'}} ] '
            f'name "{name}" '
            f'boundingObject IndexedFaceSet {{ '
            f'  coord Coordinate {{ point [ '
            f'    -0.15 0.15 0, 0.15 0.15 0, 0.15 -0.15 0, -0.15 -0.15 0, '
            f'    -0.15 0.05 {h}, 0.15 0.05 {h}, 0.15 -0.05 {h}, -0.15 -0.05 {h} '
            f'  ] }} '
            f'  coordIndex [ '
            f'    0, 1, 2, 3, -1, 4, 7, 6, 5, -1, '
            f'    1, 5, 6, 2, -1, 3, 7, 4, 0, -1, '
            f'    0, 4, 5, 1, -1, 2, 6, 7, 3, -1 '
            f'  ] '
            f'}} '
            f'}}'
        )
        self.spawn_node(vrml)

    def spawn_stairs(self, x, y, is_horizontal):
        name_base = self.unique_name("stairs")
        rot_z = 1.5708 if is_horizontal else 0
        
        vrml1 = (
            f'DEF {name_base}_step1 Solid {{ '
            f'translation {x:.4f} {y:.4f} 0.0025 '
            f'rotation 0 0 1 {rot_z:.4f} '
            f'children [ '
            f'  Transform {{ '
            f'    translation 0 0.1125 0 '
            f'    children [ Shape {{ appearance PBRAppearance {{ baseColor 0.1 0.1 0.1 roughness 0.9 }} geometry Box {{ size 0.30 0.075 0.005 }} }} ] '
            f'  }} '
            f'] '
            f'name "{name_base}_step1" '
            f'boundingObject Pose {{ translation 0 0.1125 0 children [ Box {{ size 0.30 0.075 0.005 }} ] }} '
            f'}}'
        )
        self.spawn_node(vrml1)
        
        vrml2 = (
            f'DEF {name_base}_step2 Solid {{ '
            f'translation {x:.4f} {y:.4f} 0.005 '
            f'rotation 0 0 1 {rot_z:.4f} '
            f'children [ '
            f'  Transform {{ '
            f'    translation 0 0 0 '
            f'    children [ Shape {{ appearance PBRAppearance {{ baseColor 0.1 0.1 0.1 roughness 0.9 }} geometry Box {{ size 0.30 0.15 0.010 }} }} ] '
            f'  }} '
            f'] '
            f'name "{name_base}_step2" '
            f'boundingObject Pose {{ translation 0 0 0 children [ Box {{ size 0.30 0.15 0.010 }} ] }} '
            f'}}'
        )
        self.spawn_node(vrml2)

        vrml3 = (
            f'DEF {name_base}_step3 Solid {{ '
            f'translation {x:.4f} {y:.4f} 0.0025 '
            f'rotation 0 0 1 {rot_z:.4f} '
            f'children [ '
            f'  Transform {{ '
            f'    translation 0 -0.1125 0 '
            f'    children [ Shape {{ appearance PBRAppearance {{ baseColor 0.1 0.1 0.1 roughness 0.9 }} geometry Box {{ size 0.30 0.075 0.005 }} }} ] '
            f'  }} '
            f'] '
            f'name "{name_base}_step3" '
            f'boundingObject Pose {{ translation 0 -0.1125 0 children [ Box {{ size 0.30 0.075 0.005 }} ] }} '
            f'}}'
        )
        self.spawn_node(vrml3)



    def spawn_grid(self):
        # Spawn horizontal grid lines
        for cy in range(GRID_SIZE + 1):
            y = MAZE_ORIGIN_Y + cy * CELL_SIZE
            x_center = 0.0
            length = GRID_SIZE * CELL_SIZE
            vrml = (
                f'Solid {{ '
                f'translation {x_center:.4f} {y:.4f} 0.0015 '
                f'children [ Shape {{ '
                f'appearance PBRAppearance {{ baseColor 0 0 0 roughness 1 metalness 0 }} '
                f'geometry Box {{ size {length:.4f} 0.004 0.001 }} '
                f'}} ] '
                f'name "grid_h_{cy}" '
                f'}}'
            )
            self.spawn_node(vrml)
        
        # Spawn vertical grid lines
        for cx in range(GRID_SIZE + 1):
            x = MAZE_ORIGIN_X + cx * CELL_SIZE
            y_center = 0.0
            length = GRID_SIZE * CELL_SIZE
            vrml = (
                f'Solid {{ '
                f'translation {x:.4f} {y_center:.4f} 0.0015 '
                f'children [ Shape {{ '
                f'appearance PBRAppearance {{ baseColor 0 0 0 roughness 1 metalness 0 }} '
                f'geometry Box {{ size 0.004 {length:.4f} 0.001 }} '
                f'}} ] '
                f'name "grid_v_{cx}" '
                f'}}'
            )
            self.spawn_node(vrml)

    def spawn_aruco_marker(self, cx, cy, direction):
        cell_x, cell_y = self.cell_center(cx, cy)
        z = 0.10
        offset = WALL_THICKNESS / 2.0 + 0.001
        
        if direction == 'south':
            y = cell_y - HALF_CELL + offset
            size = "0.08 0.001 0.08"
            x = cell_x
        elif direction == 'north':
            y = cell_y + HALF_CELL - offset
            size = "0.08 0.001 0.08"
            x = cell_x
        elif direction == 'west':
            x = cell_x - HALF_CELL + offset
            size = "0.001 0.08 0.08"
            y = cell_y
        elif direction == 'east':
            x = cell_x + HALF_CELL - offset
            size = "0.001 0.08 0.08"
            y = cell_y

        vrml = (
            f'Solid {{ '
            f'translation {x:.4f} {y:.4f} {z:.4f} '
            f'children [ Shape {{ '
            f'appearance PBRAppearance {{ '
            f'  baseColor 1 1 1 roughness 0.5 '
            f'  baseColorMap ImageTexture {{ url [ "textures/aruco.bmp" ] }} '
            f'}} '
            f'geometry Box {{ size {size} }} '
            f'}} ] '
            f'name "aruco_marker" '
            f'}}'
        )
        self.spawn_node(vrml)

    def spawn_floor(self):
        vrml = (
            'DEF MazeFloor Solid { '
            'translation 0 0 -0.01 '
            'children [ Shape { '
            'appearance PBRAppearance { baseColor 0.6 0.6 0.6 roughness 1 metalness 0 } '
            'geometry Box { size 2.0 2.0 0.02 } '
            '} ] '
            'name "maze_floor" '
            'boundingObject Box { size 2.0 2.0 0.02 } '
            '}'
        )
        self.spawn_node(vrml)

    # ----- TELEPORT ROBOT -----

    def teleport_robot_to_start(self, start_x, start_y):
        robot_node = self.getFromDef('SIM2REAL_ROBOT')
        if robot_node is None:
            robot_node = None
            root = self.getRoot()
            children = root.getField('children')
            for i in range(children.getCount()):
                node = children.getMFNode(i)
                name_field = node.getField('name')
                if name_field and name_field.getSFString() == 'Sim2RealRobot':
                    robot_node = node
                    break

        if robot_node:
            trans_field = robot_node.getField('translation')
            # ENU Z-UP: Spawn at Z=0.02
            trans_field.setSFVec3f([start_x, start_y, 0.02])
            rot_field = robot_node.getField('rotation')
            rot_field.setSFRotation([0, 0, 1, 0])
            robot_node.resetPhysics()
            print(f"Robot teleported to Start tile at ({start_x:.2f}, {start_y:.2f})")
        else:
            print("WARNING: Could not find robot node to teleport!")

    # ----- MAIN GENERATION -----

    def generate_environment(self):
        # --- GENERATE ARUCO MARKER DYNAMICALLY ---
        import cv2
        import os
        current_dir = os.path.dirname(os.path.abspath(__file__))
        textures_dir = os.path.join(current_dir, '..', '..', 'worlds', 'textures')
        os.makedirs(textures_dir, exist_ok=True)
        aruco_path = os.path.join(textures_dir, 'aruco.bmp')
        try:
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            marker_image = cv2.aruco.generateImageMarker(aruco_dict, 42, 256)
            cv2.imwrite(aruco_path, marker_image)
        except Exception as e:
            print(f"Failed to generate ArUco: {e}")
            
        self.spawn_floor()

        mg = MazeGenerator(GRID_SIZE, GRID_SIZE)
        mg.generate()

        all_cells = [(cx, cy) for cx in range(GRID_SIZE) for cy in range(GRID_SIZE)]
        random.shuffle(all_cells)

        start_cell = all_cells.pop()
        end_cell = all_cells.pop()
        color_cells = [all_cells.pop() for _ in range(4)]

        sx, sy = self.cell_center(*start_cell)
        self.spawn_floor_tile(sx, sy, 0, 1, 0, "tile_start")
        ex, ey = self.cell_center(*end_cell)
        self.spawn_floor_tile(ex, ey, 1, 0, 0, "tile_end")

        extra_colors = [(0, 1, 1, "tile_cyan"), (1, 1, 0, "tile_yellow"),
                        (1, 0.5, 0, "tile_orange"), (1, 0, 1, "tile_magenta")]
        for i, cell in enumerate(color_cells):
            cx, cy = self.cell_center(*cell)
            r, g, b, tname = extra_colors[i]
            self.spawn_floor_tile(cx, cy, r, g, b, tname)

        
        walls_list = []
        for cx in range(GRID_SIZE):
            for cy in range(GRID_SIZE):
                walls = mg.grid[cx][cy]
                cell_x, cell_y = self.cell_center(cx, cy)

                if walls[2]:  # South (Y-)
                    self.spawn_wall(cell_x, cell_y - HALF_CELL, True, CELL_SIZE)
                    walls_list.append((cx, cy, 'south'))
                if walls[1]:  # East (X+)
                    self.spawn_wall(cell_x + HALF_CELL, cell_y, False, CELL_SIZE)
                    walls_list.append((cx, cy, 'east'))
                if cy == GRID_SIZE - 1 and walls[0]:  # North border (Y+)
                    self.spawn_wall(cell_x, cell_y + HALF_CELL, True, CELL_SIZE)
                    walls_list.append((cx, cy, 'north'))
                if cx == 0 and walls[3]:  # West border (X-)
                    self.spawn_wall(cell_x - HALF_CELL, cell_y, False, CELL_SIZE)
                    walls_list.append((cx, cy, 'west'))
                    
        # Spawn the grid lines
        self.spawn_grid()
        
        # Spawn ArUco marker on a random wall
        if walls_list:
            target_wall = random.choice(walls_list)
            self.spawn_aruco_marker(*target_wall)
            
        reserved_cells = {start_cell, end_cell} | set(color_cells)
        
        vertical_corridors = []
        horizontal_corridors = []
        
        for cx in range(GRID_SIZE):
            for cy in range(GRID_SIZE):
                if (cx, cy) in reserved_cells:
                    continue
                
                walls = mg.grid[cx][cy] 
                if walls[1] and walls[3] and not walls[0] and not walls[2]:
                    vertical_corridors.append((cx, cy))
                elif walls[0] and walls[2] and not walls[1] and not walls[3]:
                    horizontal_corridors.append((cx, cy))

        
        placed_ramps = []
        placed_stairs = []

        # Combine all available corridors
        all_corridors = [(c, False) for c in vertical_corridors] + [(c, True) for c in horizontal_corridors]
        random.shuffle(all_corridors)

        # 1. Spawn Ramp (Max 1)
        if all_corridors:
            cell, is_horizontal = all_corridors.pop()
            cx, cy = cell
            rx, ry = self.cell_center(cx, cy)
            self.spawn_ramp(rx, ry, is_horizontal)
            placed_ramps.append((cx, cy))

        # 2. Spawn Stairs (Max 3)
        num_stairs = 0
        temp_corridors = []
        while all_corridors and num_stairs < 3:
            cell, is_horizontal = all_corridors.pop()
            cx, cy = cell
            
            # Check adjacency to ramps
            is_adjacent = False
            for rx, ry in placed_ramps:
                if abs(cx - rx) + abs(cy - ry) <= 1:
                    is_adjacent = True
                    break
            
            if not is_adjacent:
                stx, sty = self.cell_center(cx, cy)
                self.spawn_stairs(stx, sty, is_horizontal)
                placed_stairs.append((cx, cy))
                num_stairs += 1
            else:
                # Keep it for speedbumps
                temp_corridors.append((cell, is_horizontal))
                
        all_corridors.extend(temp_corridors)
        random.shuffle(all_corridors)

        # 3. Spawn Speedbumps (Max 4)
        num_bumps = 0
        while all_corridors and num_bumps < 4:
            cell, is_horizontal = all_corridors.pop()
            cx, cy = cell
            bx, by = self.cell_center(cx, cy)
            self.spawn_speedbump(bx, by, is_horizontal)
            num_bumps += 1

        for _ in range(4):
            lx = random.uniform(-0.6, 0.6)
            ly = random.uniform(-0.6, 0.6)
            lz = random.uniform(0.4, 0.8)
            intensity = random.uniform(0.3, 0.8)
            cr = random.uniform(0.85, 1.0)
            cg = random.uniform(0.85, 1.0)
            cb = random.uniform(0.85, 1.0)
            vrml = (
                f'PointLight {{ '
                f'color {cr:.3f} {cg:.3f} {cb:.3f} '
                f'intensity {intensity:.3f} '
                f'location {lx:.3f} {ly:.3f} {lz:.3f} '
                f'attenuation 0 0 4 '
                f'castShadows TRUE '
                f'}}'
            )
            self.spawn_node(vrml)

        self.teleport_robot_to_start(sx, sy)

        print(f"Maze generated: Start={start_cell}, End={end_cell}")
        print(f"Total nodes spawned: {self.node_counter}")

    def run(self):
        print("Supervisor Generating Environment...")
        self.generate_environment()
        while self.step(self.timeStep) != -1:
            pass


if __name__ == '__main__':
    supervisor = MazeSupervisor()
    supervisor.run()
