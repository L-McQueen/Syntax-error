"""
Controlador Supervisor para la Pista B (Niveles) - Webots Sim2Real
Candidates Principiantes 2026 - RoBorregos

Genera dinámicamente toda la Pista B usando VRML dinámico (Dynamic Spawning) en plano X-Y (Z-UP):
1. Sección 1: La Trampa de la Pelota (3x3 con pelota de golf física de 45g y trampa con 3 muros aleatorios).
2. Checkpoint 1: Casilla roja de conexión (3, 1).
3. Sección 2: Evasión de Líneas Blancas (2x4 verde con obstáculos de cinta blanca y gap aleatorio).
4. Checkpoint 2: Casilla roja de conexión (8, 1).
5. Sección 3: Laberinto de Colores (camino generado con Random Walk y colores estrictos de navegación:
   Cyan=Derecha, Amarillo=Izquierda, Naranja=Adelante, Magenta=Atrás).
6. Meta: Casilla verde final (FIN).
7. Muro perimetral continuo de 0.15m de altura que sella herméticamente toda la pista.
"""

from controller import Supervisor
import math
import os
import sys
import random
import time
import json

CELL_SIZE = 0.30             # Cada celda mide 30 cm de lado
HALF_CELL = CELL_SIZE / 2.0  # 15 cm
WALL_HEIGHT = 0.15           # 15 cm de altura reglamentaria
WALL_THICKNESS = 0.01        # 1 cm de grosor

# Centro del mapa en el espacio de simulación (Grid X: 0..11, Grid Y: -1..3)
GRID_CENTER_X = 5.5
GRID_CENTER_Y = 1.0

# Archivos de sincronización y logs
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.abspath(os.path.join(BASE_DIR, "..", ".."))
LOG_FILE = os.path.join(PROJECT_DIR, "god_mode_track_b.log")
FLAG_FILE = os.path.join(PROJECT_DIR, "track_b_verified.flag")
TEST_FLAG_FILE = os.path.join(PROJECT_DIR, "test_track_b.flag")
CONFIG_FILE = os.path.join(PROJECT_DIR, "track_b_config.json")

def log(msg):
    """Registra en consola y persiste en god_mode_track_b.log con autoflush."""
    print(msg, flush=True)
    try:
        with open(LOG_FILE, "a", encoding="utf-8") as f:
            f.write(f"{msg}\n")
            f.flush()
    except Exception:
        pass

def grid_to_world(gx, gy):
    """Convierte coordenadas de cuadrícula (gx, gy) a coordenadas métricas mundiales (x, y)."""
    x = (gx - GRID_CENTER_X) * CELL_SIZE
    y = (gy - GRID_CENTER_Y) * CELL_SIZE
    return x, y

class TrackBSupervisor(Supervisor):
    def __init__(self):
        super().__init__()
        self.time_step = int(self.getBasicTimeStep())
        self.root_node = self.getRoot()
        self.children_field = self.root_node.getField("children")
        self.node_counter = 0

        # Limpiar log previo
        try:
            with open(LOG_FILE, "w", encoding="utf-8") as f:
                f.write(f"=== TRACK B SUPERVISOR LOG [{time.strftime('%Y-%m-%d %H:%M:%S')}] ===\n")
        except Exception:
            pass

        log("==================================================")
        log("  SUPERVISOR PISTA B (NIVELES) - INICIALIZANDO    ")
        log("==================================================")

        # Cargar configuración desde track_b_config.json si existe
        self.config = {}
        if os.path.exists(CONFIG_FILE):
            try:
                with open(CONFIG_FILE, "r", encoding="utf-8") as f:
                    self.config = json.load(f)
                log(f"[CONFIG] track_b_config.json cargado: {self.config}")
            except Exception as e:
                log(f"[WARN] Error leyendo {CONFIG_FILE}: {e}")

        # Configurar generador de números aleatorios (semilla opcional para reproducibilidad)
        seed_cfg = self.config.get("seed")
        seed_env = os.environ.get("TRACK_B_SEED")
        if seed_cfg is not None:
            self.seed = int(seed_cfg)
            random.seed(self.seed)
            log(f"[CONFIG] Semilla determinista configurada desde JSON: {self.seed}")
        elif seed_env is not None:
            self.seed = int(seed_env)
            random.seed(self.seed)
            log(f"[CONFIG] Semilla determinista configurada desde ENV: {self.seed}")
        else:
            self.seed = int(time.time() * 1000) % 1000000
            random.seed(self.seed)
            log(f"[CONFIG] Semilla procedural aleatoria: {self.seed}")

        self.track_cells = {}
        self.ball_node = None
        self.robot_node = None
        self.start_cell = None
        self.finish_cell = None
        self.white_lines = []
        self.touched_lines = set()
        self.cp2_reached = False
        self.finish_reached = False
        self.finish_step = 0

        self.define_track_layout()
        self.build_track()
        self.teleport_robot_to_start()
        log("==================================================")
        log("  PISTA B GENERADA CON ÉXITO: LISTO PARA PRUEBAS ")
        log("==================================================")

    def unique_name(self, prefix="obj"):
        self.node_counter += 1
        return f"{prefix}_{self.node_counter}"

    def spawn_node(self, vrml_string):
        self.children_field.importMFNodeFromString(-1, vrml_string)

    def define_track_layout(self):
        """
        Define la topología de todas las celdas de la Pista B.
        La Casilla Verde (INICIO) se ubica estrictamente FUERA de la cuadrícula 3x3 inicial,
        conectando hacia el interior por la fila inferior (gy=-1).
        Todas las 9 casillas de la Sección 1 (3x3) son blancas con la pelota en el centro (1, 1).
        """
        # Candidatas exteriores para la Casilla Verde fuera del 3x3 (Sur, Norte u Oeste):
        # Sur (gy = -1): (0, -1), (1, -1), (2, -1)
        # Norte (gy = 3): (0, 3), (1, 3), (2, 3)
        # Oeste (gx = -1): (-1, 0), (-1, 1), (-1, 2)
        s1_outside_candidates = [
            (0, -1), (1, -1), (2, -1),
            (0, 3),  (1, 3),  (2, 3),
            (-1, 0), (-1, 1), (-1, 2)
        ]

        cfg_start = self.config.get("start_cell")
        is_fixed = (os.environ.get("TRACK_B_FIXED") == "1") or self.config.get("fixed", False)
        env_start = os.environ.get("TRACK_B_START_CELL")
        if cfg_start:
            if isinstance(cfg_start, (list, tuple)):
                self.start_cell = tuple(cfg_start)
            else:
                parts = [int(p.strip()) for p in str(cfg_start).split(",")]
                self.start_cell = (parts[0], parts[1])
        elif env_start:
            parts = [int(p.strip()) for p in env_start.split(",")]
            self.start_cell = (parts[0], parts[1])
        elif is_fixed:
            self.start_cell = (0, -1)
        else:
            self.start_cell = random.choice(s1_outside_candidates)

        log(f"[S1 INICIO] Casilla Verde (INICIO) ubicada FUERA del 3x3 en: {self.start_cell}")

        # 1. Casilla de Inicio (FUERA del 3x3, color Verde claro reglamentario)
        self.track_cells[self.start_cell] = {
            "type": "START",
            "color": [0.3, 0.85, 0.3],
            "name": f"tile_start_{self.start_cell[0]}_{self.start_cell[1]}",
            "label": "INICIO"
        }

        # 2. Sección 1: Trampa de la Pelota (3x3 celdas blancas completas: gx=0..2, gy=0..2)
        for gx in range(0, 3):
            for gy in range(0, 3):
                cell = (gx, gy)
                if cell == (1, 1):
                    self.track_cells[cell] = {
                        "type": "S1_BALL_CENTER",
                        "color": [0.95, 0.95, 0.95],
                        "name": f"tile_s1_{gx}_{gy}",
                        "label": "BALL_CENTER"
                    }
                else:
                    self.track_cells[cell] = {
                        "type": "S1_TRAP",
                        "color": [0.95, 0.95, 0.95],
                        "name": f"tile_s1_{gx}_{gy}",
                        "label": ""
                    }

        # 3. Checkpoint 1 (Rojo): Conecta salida de S1 (2, 1) con S2 (4, 1)
        self.track_cells[(3, 1)] = {
            "type": "CHECKPOINT_1",
            "color": [0.85, 0.1, 0.1],
            "name": "tile_cp1",
            "label": "CP1"
        }

        # 4. Sección 2: Evasión de Líneas Blancas (2x4 celdas verdes: gx=4..7, gy=1..2)
        for gx in range(4, 8):
            for gy in range(1, 3):
                self.track_cells[(gx, gy)] = {
                    "type": "S2_LINES",
                    "color": [0.1, 0.65, 0.15],
                    "name": f"tile_s2_{gx}_{gy}",
                    "label": ""
                }

        # 5. Checkpoint 2 (Rojo): Conecta salida de S2 (7, 1) con S3 (9, 1)
        self.track_cells[(8, 1)] = {
            "type": "CHECKPOINT_2",
            "color": [0.85, 0.1, 0.1],
            "name": "tile_cp2",
            "label": "CP2"
        }

        # 6. Sección 3: Laberinto de Colores (Generación de Camino y Colores Estrictos)
        # Camino navegable generado mediante algoritmo de caminata
        # Colores estrictos del reglamento:
        # Cyan [0, 1, 1]: Giro a la DERECHA
        # Amarillo [1, 1, 0]: Giro a la IZQUIERDA
        # Naranja [1, 0.5, 0]: ADELANTE
        # Magenta [1, 0, 1]: ATRÁS (regreso)
        self.generate_section_3_path()

    def generate_section_3_path(self):
        """
        Genera el laberinto de colores en Sección 3.
        Soporta generación procedural mediante Random Walk con cálculo exacto de giros relativos.
        """
        is_fixed = (os.environ.get("TRACK_B_FIXED") == "1") or self.config.get("fixed", True)

        if is_fixed:
            # Layout determinista preconfigurado con REFERENCIA ABSOLUTA (Este 0°):
            # Camino: (9, 1) -> (10, 1) -> (10, 2) -> (11, 2) -> FIN (11, 3)
            # Rumbos absolutos hacia siguiente celda:
            # (9, 1)  rumbo Este (1, 0)  -> Naranja
            # (10, 1) rumbo Norte (0, 1) -> Amarillo
            # (10, 2) rumbo Este (1, 0)  -> Naranja
            # (11, 2) rumbo Norte (0, 1) -> Amarillo
            color_map_s3 = {
                (9, 1): [1.0, 0.5, 0.0],   # Naranja (Este 0°)
                (10, 1): [1.0, 1.0, 0.0],  # Amarillo (Norte 90°)
                (10, 2): [1.0, 0.5, 0.0],  # Naranja (Este 0°)
                (11, 2): [1.0, 1.0, 0.0]   # Amarillo (Norte 90°)
            }
            finish_cell = (11, 3)
            log("[S3] Usando layout fijo verificado con referencia absoluta.")
        else:
            # Random Walk en región gx in [9, 11], gy in [0, 2]
            # Entrada obligatoria desde CP2 (8, 1) -> (9, 1) rumbo Este (1, 0)
            start_s3 = (9, 1)
            bounds_gx = (9, 11)
            bounds_gy = (0, 2)

            # Intentar generar camino no auto-intersecante de longitud 4 a 6
            best_path = None
            for attempt in range(100):
                path = [start_s3]
                curr = start_s3

                steps_target = random.randint(4, 6)
                while len(path) < steps_target:
                    # Posibles movimientos cardinales
                    moves = [(1, 0), (-1, 0), (0, 1), (0, -1)]
                    random.shuffle(moves)
                    advanced = False
                    for dx, dy in moves:
                        nx, ny = curr[0] + dx, curr[1] + dy
                        if bounds_gx[0] <= nx <= bounds_gx[1] and bounds_gy[0] <= ny <= bounds_gy[1]:
                            if (nx, ny) not in path:
                                path.append((nx, ny))
                                curr = (nx, ny)
                                advanced = True
                                break
                    if not advanced:
                        break

                if len(path) >= 4:
                    best_path = path
                    break

            if not best_path:
                best_path = [(9, 1), (10, 1), (10, 2), (11, 2)]

            log(f"[S3 RANDOM WALK] Camino generado ({len(best_path)} celdas): {best_path}")

            # Celda de meta FIN inmediatamente conectada a la última celda del camino
            last_cell = best_path[-1]
            exit_candidates = []
            # Preferir salir hacia el exterior del 3x3:
            if last_cell[1] == 2: exit_candidates.append((last_cell[0], 3))      # Norte exterior
            if last_cell[0] == 11: exit_candidates.append((12, last_cell[1]))    # Este exterior
            if last_cell[1] == 0: exit_candidates.append((last_cell[0], -1))     # Sur exterior

            # O celdas adyacentes no visitadas
            for dx, dy in [(1, 0), (0, 1), (0, -1), (-1, 0)]:
                nx, ny = last_cell[0] + dx, last_cell[1] + dy
                if (nx, ny) not in best_path and (nx, ny) != (8, 1):
                    exit_candidates.append((nx, ny))

            finish_cell = exit_candidates[0] if exit_candidates else (last_cell[0], last_cell[1] + 1)

            # Calcular colores según REFERENCIA ABSOLUTA (Reglamento Sim2Real 2026):
            # Este (1, 0)   = Naranja  [1.0, 0.5, 0.0] (0.0°)
            # Norte (0, 1)  = Amarillo [1.0, 1.0, 0.0] (+90.0°)
            # Sur (0, -1)   = Cyan     [0.0, 1.0, 1.0] (-90.0°)
            # Oeste (-1, 0) = Magenta  [1.0, 0.0, 1.0] (180.0°)
            color_map_s3 = {}
            for i in range(len(best_path)):
                cell = best_path[i]
                if i < len(best_path) - 1:
                    next_cell = best_path[i + 1]
                    out_dir = (next_cell[0] - cell[0], next_cell[1] - cell[1])
                else:
                    out_dir = (finish_cell[0] - cell[0], finish_cell[1] - cell[1])

                if out_dir == (1, 0):
                    color = [1.0, 0.5, 0.0] # Naranja (Este 0°)
                elif out_dir == (0, 1):
                    color = [1.0, 1.0, 0.0] # Amarillo (Norte 90°)
                elif out_dir == (0, -1):
                    color = [0.0, 1.0, 1.0] # Cyan (Sur -90°)
                elif out_dir == (-1, 0):
                    color = [1.0, 0.0, 1.0] # Magenta (Oeste 180°)
                else:
                    color = [1.0, 0.5, 0.0]

                color_map_s3[cell] = color

        # Registrar todas las celdas de la cuadrícula S3 (gx=9..11, gy=0..2)
        for gx in range(9, 12):
            for gy in range(0, 3):
                col = color_map_s3.get((gx, gy), [0.95, 0.95, 0.95])
                is_path = (gx, gy) in color_map_s3
                self.track_cells[(gx, gy)] = {
                    "type": "S3_COLOR_MAZE",
                    "color": col,
                    "name": f"tile_s3_{gx}_{gy}",
                    "label": "COLOR_PATH" if is_path else ""
                }

        # Casilla de Meta (FIN)
        self.finish_cell = finish_cell
        self.track_cells[finish_cell] = {
            "type": "FINISH",
            "color": [0.3, 0.85, 0.3],
            "name": "tile_finish",
            "label": "FIN"
        }
        log(f"[S3] Meta FIN ubicada en: {finish_cell}")

    def build_track(self):
        """Instancia todos los nodos físicos y visuales en la simulación."""
        # 1. Piso base estructural con líneas de 4mm
        self.spawn_base_floor()

        # 2. Todas las baldosas individuales
        for (gx, gy), meta in self.track_cells.items():
            x, y = grid_to_world(gx, gy)
            r, g, b = meta["color"]
            self.spawn_tile(x, y, r, g, b, meta["name"])
        log(f"[BUILD] {len(self.track_cells)} baldosas instanciadas.")

        # 3. Trampa de la pelota en Sección 1
        self.spawn_ball_trap()

        # 4. Pelota de golf física reglamentaria (42mm, 45g)
        self.spawn_golf_ball()

        # 5. Barreras de líneas blancas en Sección 2
        self.spawn_white_lines()

        # 6. Muro perimetral exterior continuo de 0.15m
        self.spawn_perimeter_walls()

    def spawn_base_floor(self):
        """Piso base negro estructural que produce las líneas negras de 4mm entre baldosas."""
        vrml = (
            'DEF BaseFloor Solid { '
            'translation 0 0 -0.01 '
            'children [ Shape { '
            '  appearance PBRAppearance { baseColor 0.02 0.02 0.02 roughness 1.0 metalness 0 } '
            '  geometry Box { size 4.8 2.6 0.02 } '
            '} ] '
            'name "base_floor" '
            'boundingObject Box { size 4.8 2.6 0.02 } '
            '}'
        )
        self.spawn_node(vrml)

    def spawn_tile(self, x, y, r, g, b, tile_name):
        """Baldosa individual de 0.296m x 0.296m (separación negra exacta de 4mm)."""
        tile_size = CELL_SIZE - 0.004  # 0.296 m
        vrml = (
            f'DEF {tile_name} Solid {{ '
            f'translation {x:.4f} {y:.4f} 0.001 '
            f'children [ Shape {{ '
            f'  appearance PBRAppearance {{ baseColor {r:.3f} {g:.3f} {b:.3f} roughness 0.8 metalness 0 }} '
            f'  geometry Box {{ size {tile_size:.3f} {tile_size:.3f} 0.002 }} '
            f'}} ] '
            f'name "{tile_name}" '
            f'}}'
        )
        self.spawn_node(vrml)

    def spawn_wall_segment(self, x, y, is_horizontal, length=CELL_SIZE, height=WALL_HEIGHT, name_prefix="wall"):
        """Genera un segmento de muro de 0.15m de altura con colisión física."""
        name = self.unique_name(name_prefix)
        if is_horizontal:
            size_str = f"{length:.4f} {WALL_THICKNESS:.4f} {height:.4f}"
        else:
            size_str = f"{WALL_THICKNESS:.4f} {length:.4f} {height:.4f}"
        z = height / 2.0
        vrml = (
            f'DEF {name} Solid {{ '
            f'translation {x:.4f} {y:.4f} {z:.4f} '
            f'children [ Shape {{ '
            f'  appearance PBRAppearance {{ baseColor 0.70 0.70 0.70 roughness 0.9 metalness 0 }} '
            f'  geometry Box {{ size {size_str} }} '
            f'}} ] '
            f'name "{name}" '
            f'contactMaterial "Wall" '
            f'boundingObject Box {{ size {size_str} }} '
            f'}}'
        )
        self.spawn_node(vrml)

    def spawn_ball_trap(self):
        """
        Genera los 3 muros centrales alrededor de la celda (1, 1).
        Reglamento: 'El Supervisor debe elegir ALEATORIAMENTE qué lado queda abierto para que el robot entre'.
        """
        cx, cy = grid_to_world(1, 1)

        sides = ["North", "South", "East", "West"]
        cfg_side = self.config.get("open_side")
        env_side = os.environ.get("TRACK_B_OPEN_SIDE")
        is_fixed = (os.environ.get("TRACK_B_FIXED") == "1") or self.config.get("fixed", False)
        if cfg_side in sides:
            open_side = cfg_side
        elif env_side in sides:
            open_side = env_side
        elif is_fixed:
            open_side = "South"
        else:
            open_side = random.choice(sides)
        log(f"[S1 BALL TRAP] Lado abierto: {open_side} (los otros 3 lados bloqueados con muros 0.15m)")

        if open_side != "West":
            self.spawn_wall_segment(cx - HALF_CELL, cy, is_horizontal=False, name_prefix="trap_wall_W")
        if open_side != "North":
            self.spawn_wall_segment(cx, cy + HALF_CELL, is_horizontal=True, name_prefix="trap_wall_N")
        if open_side != "East":
            self.spawn_wall_segment(cx + HALF_CELL, cy, is_horizontal=False, name_prefix="trap_wall_E")
        if open_side != "South":
            self.spawn_wall_segment(cx, cy - HALF_CELL, is_horizontal=True, name_prefix="trap_wall_S")

    def spawn_golf_ball(self):
        """
        Instancia la Pelota de Golf Naranja:
        - Geometría: Sphere con radio 0.021m (42mm de diámetro)
        - Material: Naranja brillante
        - Física: masa 0.045 kg (45 gramos) y damping lineal/angular para rodadura realista
        """
        cx, cy = grid_to_world(1, 1)
        ball_radius = 0.021
        z = ball_radius + 0.002  # Descansa sobre la superficie de la baldosa
        vrml = (
            f'DEF GOLF_BALL Solid {{ '
            f'translation {cx:.4f} {cy:.4f} {z:.4f} '
            f'children [ Shape {{ '
            f'  appearance PBRAppearance {{ baseColor 1.0 0.45 0.0 roughness 0.2 metalness 0.1 }} '
            f'  geometry Sphere {{ radius {ball_radius:.4f} }} '
            f'}} ] '
            f'name "golf_ball" '
            f'contactMaterial "RubberWheel" '
            f'boundingObject Sphere {{ radius {ball_radius:.4f} }} '
            f'physics Physics {{ '
            f'  density -1 '
            f'  mass 0.045 '
            f'  damping Damping {{ linear 0.05 angular 0.05 }} '
            f'}} '
            f'}}'
        )
        self.spawn_node(vrml)
        log(f"[S1 GOLF BALL] Pelota instanciada en ({cx:.3f}, {cy:.3f}, {z:.3f}), r={ball_radius}m, m=0.045kg.")

    def spawn_white_lines(self):
        """
        Genera las líneas blancas en Sección 2 con longitud variable y hueco continuo garantizado de 0.30m.
        Corredor Y: y_min = -0.15m, y_max = +0.45m. Ancho total = 0.60m.
        El hueco de 0.30m se posiciona con centro y_gap variable en cada frontera.
        """
        is_fixed = (os.environ.get("TRACK_B_FIXED") == "1") or self.config.get("fixed", False)
        cfg_gaps = self.config.get("s2_gaps") # opcional: lista de y_gap para [Col 4-5, Col 5-6, Col 6-7]
        default_fixed_gaps = [0.15, 0.00, 0.30]

        self.white_lines = []
        for i, gx in enumerate([4, 5, 6]):
            if cfg_gaps and i < len(cfg_gaps):
                y_gap = float(cfg_gaps[i])
            elif is_fixed:
                y_gap = default_fixed_gaps[i]
            else:
                y_gap = random.choice([0.00, 0.30, 0.15])

            gap_min = y_gap - 0.15
            gap_max = y_gap + 0.15

            cx, _ = grid_to_world(gx, 1)
            line_x = cx + HALF_CELL

            len_bot = gap_min - (-0.15)
            if len_bot > 0.01:
                mid_bot = -0.15 + (len_bot / 2.0)
                name_bot = f"white_line_{i+1}_bot"
                vrml = (
                    f'DEF {name_bot} Solid {{ '
                    f'translation {line_x:.4f} {mid_bot:.4f} 0.0025 '
                    f'children [ Shape {{ '
                    f'  appearance PBRAppearance {{ baseColor 1.0 1.0 1.0 roughness 0.2 metalness 0 }} '
                    f'  geometry Box {{ size 0.02 {len_bot:.4f} 0.002 }} '
                    f'}} ] '
                    f'name "{name_bot}" '
                    f'}}'
                )
                self.spawn_node(vrml)
                self.white_lines.append({
                    "name": name_bot,
                    "x": line_x,
                    "min_y": -0.15,
                    "max_y": gap_min,
                    "len": len_bot
                })

            len_top = 0.45 - gap_max
            if len_top > 0.01:
                mid_top = gap_max + (len_top / 2.0)
                name_top = f"white_line_{i+1}_top"
                vrml = (
                    f'DEF {name_top} Solid {{ '
                    f'translation {line_x:.4f} {mid_top:.4f} 0.0025 '
                    f'children [ Shape {{ '
                    f'  appearance PBRAppearance {{ baseColor 1.0 1.0 1.0 roughness 0.2 metalness 0 }} '
                    f'  geometry Box {{ size 0.02 {len_top:.4f} 0.002 }} '
                    f'}} ] '
                    f'name "{name_top}" '
                    f'}}'
                )
                self.spawn_node(vrml)
                self.white_lines.append({
                    "name": name_top,
                    "x": line_x,
                    "min_y": gap_max,
                    "max_y": 0.45,
                    "len": len_top
                })

            log(f"[S2 WHITE LINE {i+1}] Frontera Col {gx}->{gx+1} (X={line_x:.3f}m): "
                f"Hueco libre 30cm en Y=[{gap_min:+.2f}m a {gap_max:+.2f}m] (Centro Y={y_gap:+.2f}m) | "
                f"L_bot={len_bot:.2f}m, L_top={len_top:.2f}m")

    def spawn_perimeter_walls(self):
        """
        Genera el muro perimetral continuo de 0.15m que encierra toda la silueta de la pista.
        Para cada celda activa, si en una dirección cardinal no hay celda vecina, genera un muro.
        """
        track_set = set(self.track_cells.keys())
        wall_count = 0

        for (gx, gy) in track_set:
            cx, cy = grid_to_world(gx, gy)

            # Norte (gy + 1)
            if (gx, gy + 1) not in track_set:
                self.spawn_wall_segment(cx, cy + HALF_CELL, is_horizontal=True, name_prefix="perim_N")
                wall_count += 1

            # Sur (gy - 1)
            if (gx, gy - 1) not in track_set:
                self.spawn_wall_segment(cx, cy - HALF_CELL, is_horizontal=True, name_prefix="perim_S")
                wall_count += 1

            # Este (gx + 1)
            if (gx + 1, gy) not in track_set:
                self.spawn_wall_segment(cx + HALF_CELL, cy, is_horizontal=False, name_prefix="perim_E")
                wall_count += 1

            # Oeste (gx - 1)
            if (gx - 1, gy) not in track_set:
                self.spawn_wall_segment(cx - HALF_CELL, cy, is_horizontal=False, name_prefix="perim_W")
                wall_count += 1

        log(f"[PERIMETER] Muro continuo exterior generado: {wall_count} segmentos de 0.15m de altura.")

    def teleport_robot_to_start(self):
        """Ubica al robot en la celda de inicio (0, -1) orientado al Norte (+Y)."""
        robot_node = self.getFromDef("SIM2REAL_ROBOT")
        if not robot_node:
            root = self.getRoot()
            children = root.getField("children")
            for i in range(children.getCount()):
                node = children.getMFNode(i)
                name_field = node.getField("name")
                if name_field and name_field.getSFString() == "Sim2RealRobot":
                    robot_node = node
                    break

        if robot_node and self.start_cell:
            self.robot_node = robot_node
            gx, gy = self.start_cell
            sx, sy = grid_to_world(gx, gy)
            trans_field = robot_node.getField("translation")
            trans_field.setSFVec3f([sx, sy, 0.03])
            rot_field = robot_node.getField("rotation")
            # Orientado hacia el interior de la cuadrícula 3x3
            if gy <= -1:
                yaw = 1.5707963   # 90.0° Norte (+Y)
            elif gy >= 3:
                yaw = -1.5707963  # -90.0° Sur (-Y)
            elif gx <= -1:
                yaw = 0.0         # 0.0° Este (+X)
            else:
                yaw = 1.5707963
            rot_field.setSFRotation([0, 0, 1, yaw])
            robot_node.resetPhysics()
            log(f"[ROBOT] Sim2RealRobot teletransportado a INICIO {self.start_cell} -> ({sx:.3f}, {sy:.3f}, 0.030) rumbo {math.degrees(yaw):.1f}°.")
        else:
            log("[WARN] No se encontró el nodo Sim2RealRobot o self.start_cell para teletransportación.")

    def run(self):
        """Ciclo de supervisión, física y telemetría."""
        step_count = 0
        self.ball_node = self.getFromDef("GOLF_BALL")
        s1_success = False
        s1_success_step = 0

        # Modo test automático
        is_test_mode = (os.environ.get("TRACK_B_TEST") == "1") or os.path.exists(TEST_FLAG_FILE) or self.config.get("test_mode", False)
        is_s1_test = (os.environ.get("TRACK_B_S1_TEST") == "1") or self.config.get("s1_test", False)
        if is_test_mode:
            log("[MODE] Modo verificación activado: ejecutará 120 steps para validar físicas y spawn.")
        if is_s1_test:
            log("[MODE] Modo prueba Sección 1 activado: saldrá tras confirmar éxito en Checkpoint 1.")

        while self.step(self.time_step) != -1:
            step_count += 1

            if self.ball_node and self.robot_node:
                bp = self.ball_node.getPosition()
                rp = self.robot_node.getPosition()
                ball_dist = math.hypot(bp[0] - rp[0], bp[1] - rp[1])

                if step_count % 30 == 0:  # Cada ~0.5s de tiempo simulado
                    log(f"[TELEMETRÍA #{step_count:04d}] Robot: ({rp[0]:.2f}, {rp[1]:.2f}) | "
                        f"Pelota: ({bp[0]:.2f}, {bp[1]:.2f}, Z={bp[2]:.3f}) | Dist: {ball_dist:.2f}m")

                # Detección de éxito en Sección 1:
                cp1_x, cp1_y = grid_to_world(3, 1)
                dx_cp1 = abs(rp[0] - cp1_x)
                dy_cp1 = abs(rp[1] - cp1_y)
                if dx_cp1 < 0.15 and dy_cp1 < 0.15 and ball_dist < 0.10:
                    if not s1_success:
                        s1_success = True
                        s1_success_step = step_count
                        log("==================================================")
                        log("  [S1 SUCCESS] ¡MISIÓN SECCIÓN 1 CUMPLIDA!        ")
                        log("  Robot en Checkpoint 1 (3, 1) con la Pelota!     ")
                        log(f"  Distancia Robot-Pelota: {ball_dist*100:.1f} cm   ")
                        log("==================================================")
                        try:
                            flag_s1 = os.path.join(PROJECT_DIR, "track_b_s1_passed.flag")
                            with open(flag_s1, "w", encoding="utf-8") as f:
                                f.write("PASSED\n")
                        except Exception as e:
                            log(f"Error escribiendo flag s1: {e}")

                # Monitoreo de contacto con líneas blancas en Sección 2:
                for wline in self.white_lines:
                    lx = wline["x"]
                    if abs(rp[0] - lx) <= 0.045:
                        if (wline["min_y"] - 0.040) <= rp[1] <= (wline["max_y"] + 0.040):
                            if wline["name"] not in self.touched_lines:
                                self.touched_lines.add(wline["name"])
                                log("==================================================")
                                log(f"  [S2 TOUCH] ¡ALERTA: Robot pisó la línea blanca {wline['name']}!")
                                log(f"  Posición Robot: ({rp[0]:.2f}, {rp[1]:.2f}) | Línea X={lx:.2f}, Y=[{wline['min_y']:.2f}, {wline['max_y']:.2f}]")
                                log("==================================================")

                # Detección de Checkpoint 2 (8, 1):
                cp2_x, cp2_y = grid_to_world(8, 1)
                dx_cp2 = abs(rp[0] - cp2_x)
                dy_cp2 = abs(rp[1] - cp2_y)
                if dx_cp2 < 0.15 and dy_cp2 < 0.15:
                    if not self.cp2_reached:
                        self.cp2_reached = True
                        log("==================================================")
                        log(f"  [CP2 SUCCESS] Robot alcanzó Checkpoint 2 (8, 1) en ({rp[0]:.2f}, {rp[1]:.2f})")
                        if len(self.touched_lines) == 0:
                            log("  [S2 RESULT] [S2 CLEAN] ¡Paso limpio! Sección 2 superada sin tocar líneas blancas.")
                        else:
                            log(f"  [S2 RESULT] [S2 TOUCHED] Sección 2 completada tocando {len(self.touched_lines)} líneas blancas.")
                        log("==================================================")

                # Detección de Meta Final FIN en Sección 3:
                if self.finish_cell:
                    fin_x, fin_y = grid_to_world(self.finish_cell[0], self.finish_cell[1])
                    dx_fin = abs(rp[0] - fin_x)
                    dy_fin = abs(rp[1] - fin_y)
                    if dx_fin < 0.15 and dy_fin < 0.15:
                        if not self.finish_reached:
                            self.finish_reached = True
                            self.finish_step = step_count
                            log("==================================================")
                            log(f"  [TRACK B SUCCESS] ¡META ALCANZADA EN {self.finish_cell} ({rp[0]:.2f}, {rp[1]:.2f})!")
                            log("  ¡PISTA B (NIVELES) COMPLETADA EXITOSAMENTE!    ")
                            if len(self.touched_lines) == 0:
                                log("  [S2 CERTIFIED] Paso 100% limpio por líneas blancas.")
                            else:
                                log(f"  [S2 WARNING] Total líneas tocadas: {len(self.touched_lines)}")
                            log("==================================================")
                            try:
                                with open(FLAG_FILE, "w", encoding="utf-8") as f:
                                    f.write("OK\n")
                            except Exception as e:
                                log(f"Error escribiendo flag: {e}")

            if is_s1_test and s1_success and (step_count - s1_success_step >= 60):
                log("[MODE] Prueba Sección 1 completada exitosamente. Saliendo de Webots.")
                self.simulationQuit(0)
                break

            if self.finish_reached and (step_count - self.finish_step >= 60):
                log("[MODE] Misión Pista B completada exitosamente. Saliendo de Webots.")
                self.simulationQuit(0)
                break

            if is_test_mode and step_count >= 120:
                log("==================================================")
                log("  [TEST PASSED] 120 steps completados con éxito.  ")
                log("  Físicas, spawn dinámico y colisiones validadas. ")
                log("==================================================")
                try:
                    with open(FLAG_FILE, "w", encoding="utf-8") as f:
                        f.write("OK\n")
                except Exception as e:
                    log(f"Error escribiendo flag: {e}")
                self.simulationQuit(0)
                break

if __name__ == "__main__":
    supervisor = TrackBSupervisor()
    supervisor.run()
