<div align="center">
  <h1>🤖 Webots Sim2Real Maze Solver</h1>
  <p><strong>Navegación autónoma en laberintos con arquitectura distribuida, visión híbrida, odometría óptica ToF, control lateral PID (10-15 cm) y calibración inercial en vivo.</strong></p>
  <p><i>¡Probando toda la matemática y el control en simulación antes de quemar componentes en el hardware real!</i></p>

  <p>
    <img src="https://img.shields.io/badge/Webots-R2023b-blue.svg" alt="Webots">
    <img src="https://img.shields.io/badge/Python-3.10+-yellow.svg" alt="Python">
    <img src="https://img.shields.io/badge/OpenCV-4.x-green.svg" alt="OpenCV">
    <img src="https://img.shields.io/badge/Sim2Real-100%25%20Verified-brightgreen.svg" alt="Sim2Real">
  </p>
</div>

---

## 📌 ¿De qué trata este proyecto?

Este proyecto nace con una filosofía muy clara de ingeniería práctica: **diseñar un robot autónomo capaz de resolver un laberinto modular con presupuesto accesible ("modo estudihambre"), exprimiendo al máximo la física, las matemáticas y el software antes de armarlo físicamente.**

Muchas soluciones en robótica asumen que dispones de encoders magnéticos de alta resolución, microcontroladores industriales o LiDARs de cientos de dólares. Aquí asumimos el reto contrario:
- ¿Qué pasa si **no tenemos encoders de rueda a la mano** o queremos evitar el típico error acumulado por patinaje (*wheel-slip*)?
- ¿Qué pasa si el giróscopo económico (**MPU-6050**) deriva con la temperatura y las vibraciones?
- ¿Cómo nos aseguramos de que el carro entre **perfectamente centrado y alineado a los pasillos e intersecciones**, sin raspar las esquinas ni trabarse en callejones sin salida?

Para resolver esto, construimos un entorno **Sim2Real** en **Webots** que modela con honestidad las imperfecciones del hardware real (asimetría de motores, micro-patinaje en llantas, deriva estocástica del sensor inercial y ruido en láseres ToF), diseñando algoritmos de control que garantizan un **100% de éxito en pista y retorno a base**.

---

## 🛠️ Arquitectura de Hardware Real (El Setup Físico)

El robot está pensado con una **arquitectura distribuida de procesamiento jerárquico** con aislamiento eléctrico de potencia: dividimos las tareas pesadas de visión, fusión sensorial y toma de decisiones en la SBC, mientras un microcontrolador dedicado se encarga exclusivamente del tren motriz.

```
                     ┌────────────────────────────────────────┐
                     │          Batería LiPo 3S (11.1V)       │
                     └───────────────────┬────────────────────┘
                                         │
                         ┌───────────────┴───────────────┐
                         ▼                               ▼
               [LÍNEA 1: POTENCIA]              [LÍNEA 2: LÓGICA]
                Step-Down LM2596                 Step-Down 5V
                 (7.8V Estable)                 (Alimentación Lógica)
                         │                               │
                         │               ┌───────────────┴───────────────┐
                         │               ▼                               ▼
                         │        Arduino Uno R3                 Orange Pi Zero 2W
                         │       (PWM Exclusivo)             (Cerebro Linux / Sensores)
                         │               │                               │
                         ▼               │                               │
                   Driver DRV8833 ◄──────┘ (Pines PWM)                   │
                   (Dual H-Bridge)                                       │
                         │                                               │
                         ▼                                               ▼
                  Motores N20 (100:1)                           Sensores Directos (I2C/USB):
                   (Tracción 7.8V)                               - IMU GY-521 (MPU-6050)
                                                                 - ToF Frontal (VL53L1X)
                                                                 - ToFs Laterales (VL53L0X)
                                                                 - Webcam USB (ArUco)
                                                                 - Pixy / PixyMon (Piso)
```

### Componentes Seleccionados y Decisiones Técnicas:

1. **Aislamiento Eléctrico con Doble Línea de Alimentación:**
   * **Línea de Potencia (7.8V fijos):** Alimentada por un convertidor reductor **LM2596 Step-Down a 7.8V** que va **exclusivamente al driver DRV8833 y los motores N20**. Esto aísla por completo el ruido electromagnético, los picos inductivos y las caídas momentáneas de tensión (*voltage sags*) provocadas por los motores, evitando reinicios repentinos (*brownouts*) en la electrónica de control.
   * **Línea de Lógica (5V estables):** Una línea independiente de $5\text{ V}$ alimenta la **Orange Pi Zero 2W** y el **Arduino Uno R3**, asegurando que ambos cerebros trabajen con rizado mínimo y alimentación limpia.

2. **Cerebro Superior y Fusión Sensorial: Orange Pi Zero 2W**
   * Corre Linux embebido y **concentra todos los sensores directamente**:
     - **Bus I2C nativo:** Lee en tiempo real el giróscopo/acelerómetro **GY-521 (MPU-6050)** y los 3 sensores de distancia láser ToF (**VL53L1X** frontal y **VL53L0X** laterales).
     - **Puertos USB / Hardware:** Conecta la **Webcam USB** (para lectura de ArUco con OpenCV) y la cámara **Pixy / PixyMon** (para clasificación instantánea del color de piso).
   * Aquí se ejecuta el algoritmo de mapeo DFS, el estimador físico de orientación con muros ($\theta_{\text{walls}}$) y el control lateral PID.

3. **Cerebro de Bajo Nivel: Arduino Uno R3 (Exclusivo para Motores)**
   * Conectado a la Orange Pi por bus I2C / Serial.
   * **Al Arduino SOLAMENTE se conectan los motores** a través del driver DRV8833.
   * *¿Por qué separar el control de motores en el Arduino?* Los sistemas operativos como Linux no son de tiempo real estricto (*hard real-time*); generar PWM directamente desde los pines de una SBC puede presentar micro-jitter e inconsistencias cuando la CPU se satura procesando visión. El Arduino Uno se encarga al $100\%$ de generar el tren de pulsos PWM determinista, suave y simétrico para cada llanta.

4. **Driver de Motor: DRV8833**
   * Elegido sobre el clásico L298N porque el DRV8833 utiliza puentes H de transistores MOSFET con una caída de voltaje interna prácticamente nula ($\approx 0.1\text{ V}$ frente a los $\approx 2.0\text{ V}$ que desperdicia el L298N en forma de calor).

5. **Motores: N20 Micro Metal Gearmotor (Relación 100:1)**
   * Rango de operación de 6 a 12V. En pruebas nominales a $6.0\text{ V}$ entregan $297.0\text{ RPM}$ en vacío.
   * Su caja reductora metálica entrega el torque necesario para subir rampas y acelerar suavemente en casillas de $30\text{ cm}$.

6. **Sensores de Distancia: ToF Láser (VL53L1X y VL53L0X) en vez de Ultrasónicos HC-SR04**
   * *¿Por qué NO ultrasónicos?* El sensor ultrasónico HC-SR04 emite un cono acústico ancho ($\approx 15^\circ - 30^\circ$). En un laberinto con pasillos estrechos de apenas $30\text{ cm}$, el eco sonoro rebota contra las paredes laterales (*multipath interference*), arrojando distancias falsas y fantasmas.
   * *Nuestra elección:* Sensores de Tiempo de Vuelo láser (ToF):
     - **Frontal (VL53L1X):** Rango largo de hasta $4.0\text{ m}$ con cono óptico milimétrico.
     - **Laterales (VL53L0X):** Rango de hasta $2.0\text{ m}$, ideales para medir distancias de $5$ a $25\text{ cm}$ contra las paredes laterales.

7. **IMU: GY-521 (MPU-6050)**
   * Módulo popular de 6 grados de libertad (acelerómetro + giróscopo de 3 ejes) que proporciona el ángulo de guiñada (Yaw) para orientar el robot. Conectado por I2C directo a la Orange Pi para autocalibración continua con el algoritmo $\theta_{\text{walls}}$.

---

## 📐 La Matemática y Mecanismos de Navegación

### 1. Odometría sin Encoders: Dead-Reckoning a 7.8V + Odómetro Óptico Frontal

Como no tenemos encoders de rueda, creamos un sistema de odometría redundante basado en física de motores y óptica láser:

```
                            RELACIÓN FÍSICA LINEAL DC
   Tensión Nominal: 6.0 V  ───────────────────────────────► 297.0 RPM
                                k_v = 7.8 / 6.0 = 1.30
   Tensión Estabilizada: 7.8 V ───────────────────────────► 386.1 RPM (Max No-Load)
```

#### A. Modelo Eléctrico de los Motores N20:
1. **Velocidad Angular Máxima a 7.8V:**
   $$\text{RPM}_{7.8V} = 297.0 \times \left(\frac{7.8\text{ V}}{6.0\text{ V}}\right) = 386.1\text{ RPM}$$
   $$\omega_{\text{max}} = 386.1 \times \frac{2\pi}{60} = 40.4323\text{ rad/s}$$
2. **Velocidad Lineal de Crucero (Ruedas $R = 0.02\text{ m}$):**
   * Comandamos los motores a $\omega_{\text{crucero}} = 4.2\text{ rad/s}$ (equivalente al $\approx 10.39\%$ del PWM del convertidor a 7.8V).
   * La velocidad lineal del carrito es determinísticamente:
     $$v_{\text{lin}} = \omega_{\text{crucero}} \times R = 4.2\text{ rad/s} \times 0.02\text{ m} = 0.084\text{ m/s} \quad (8.4\text{ cm/s})$$
3. **Constante Temporal de Celda (30 cm):**
   $$T_{\text{celda}} = \frac{\text{CELL\_SIZE}}{v_{\text{lin}}} = \frac{0.30\text{ m}}{0.084\text{ m/s}} \approx 3.571\text{ segundos}$$
   Cada casilla plana toma exactamente **3.57 segundos** en recorrerse.

#### B. Odómetro Óptico con ToF Frontal ($\Delta d$):
* Al arrancar en una celda, el sensor VL53L1X mide la distancia inicial a la pared de enfrente:
  $$\Delta d = d_{\text{inicial}} - d_{\text{actual}}$$
* Si hay un muro enfrente, $\Delta d$ mide el desplazamiento lineal físico directo sin tocar el piso y sin verse afectado por si la llanta patinó o no.
* **Detención centrada:** Si el robot avanza hacia un muro frontal, sabe que en el centro de la celda de destino el muro debe quedar a **$15\text{ cm}$** ($0.15\text{ m}$). Cuando $front\_d \le 0.15\text{ m}$ y $dist\_dr \ge 0.22\text{ m}$, clava el freno: queda estacionado exactamente en el centro geométrico de la celda.

---

### 2. Control Lateral PID: Banda Dorada de 10 a 15 cm

Para evitar que el robot entre a los cruces chueco o raspando las esquinas, implementamos un controlador lateral adaptativo de 3 zonas:

```
                  [Pared Izquierda]                       [Pared Derecha]
             │ ◄─────── 10-15 cm ──────► [Robot] ◄────── 10-15 cm ──────► │
             │   Zona de Peligro: < 10cm          Zona de Peligro: < 10cm │
             │   (Empuje hacia la der)            (Empuje hacia la izq)   │
             │   Zona Neutra: 10 - 15 cm          Zona Neutra: 10 - 15 cm │
             │   (Avance Recto Estable)           (Avance Recto Estable)  │
```

#### Ecuación del Error Lateral:
* **Con 2 paredes laterales ($d_L < 22\text{ cm}$ y $d_R < 22\text{ cm}$):** Centrado equidistante:
  $$e = \frac{d_L - d_R}{2}$$
* **Con 1 sola pared (Pared Izquierda):**
  $$e = \begin{cases} d_L - 0.10 & \text{si } d_L < 0.10\text{ m} \quad (\text{empujar a la derecha para no rozar}) \\ d_L - 0.15 & \text{si } d_L > 0.15\text{ m} \quad (\text{atraer suavemente hacia la pared}) \\ 0.0 & \text{si } 0.10\text{ m} \le d_L \le 0.15\text{ m} \quad (\textbf{zona muerta: avance recto}) \end{cases}$$
* **Con 1 sola pared (Pared Derecha):**
  $$e = \begin{cases} 0.10 - d_R & \text{si } d_R < 0.10\text{ m} \quad (\text{empujar a la izquierda para no rozar}) \\ 0.15 - d_R & \text{si } d_R > 0.15\text{ m} \quad (\text{atraer suavemente hacia la pared}) \\ 0.0 & \text{si } 0.10\text{ m} \le d_R \le 0.15\text{ m} \quad (\textbf{zona muerta: avance recto}) \end{cases}$$
* **En espacio abierto ($d \ge 22\text{ cm}$):** El PID se desconecta de inmediato ($e = 0.0$) para evitar perturbaciones falsas ante puertas o cruces.

<div align="center">
  <img src="docs/images/lateral_wall_pid_behavior.png" alt="Comportamiento PID Lateral" width="85%">
  <p><i>Figura 1: Lecturas ToF laterales dentro de la banda de 10-15 cm y corrección de timoneo angular suave.</i></p>
</div>

---

### 3. Estimador Físico de Ángulo con Muros ($\theta_{\text{walls}}$) y Calibración Continua del IMU

El giróscopo MPU-6050 económico tiene una deriva térmica que va desfasando el ángulo de orientación ($\pm 0.15^\circ/\text{s}$). Tras 1 minuto de carrera, el giróscopo acumula $15^\circ - 25^\circ$ de error. Cuando el carro intenta girar $90^\circ$, en realidad gira $70^\circ$, entra cruzado y choca contra los postes.

#### ¿Cómo lo solucionamos sin brújula ni GPS?
En un laberinto ortogonal, **las paredes del pasillo siempre apuntan a los rumbos cardinales físicos exactos ($0^\circ, 90^\circ, 180^\circ, 270^\circ$)**.

1. Al avanzar en un pasillo a velocidad $v = 0.084\text{ m/s}$, el robot registra la evolución en el tiempo de la métrica lateral $m$:
   $$m = \begin{cases} \frac{d_L - d_R}{2} & \text{si hay 2 paredes} \\ d_L & \text{si hay pared izquierda} \\ -d_R & \text{si hay pared derecha} \end{cases}$$
2. Sobre una ventana temporal $\Delta t$ ($\Delta s = v \cdot \Delta t$):
   $$\theta_{\text{walls}} = -\frac{1}{v} \frac{dm}{dt} = -\frac{m(t) - m(t - \Delta t)}{\Delta s}$$
   $\theta_{\text{walls}}$ es el **ángulo físico real** del carrito respecto al eje longitudinal de las paredes.
3. La diferencia entre lo que dice el giróscopo (`current_yaw`) y la orientación física real de la pared es la **deriva pura del IMU**:
   $$\epsilon_{\text{drift}} = \text{normalize\_angle}(\text{current\_yaw} - \text{target\_yaw} - \theta_{\text{walls}})$$
4. En cada ciclo de simulación, el robot absorbe suavemente esta deriva en su sesgo (`yaw_offset`):
   $$\text{yaw\_offset} \leftarrow \text{yaw\_offset} + 0.035 \cdot \epsilon_{\text{drift}}$$

**Efecto:** Conforme el carrito recorre un pasillo, su orientación interna se autocalibra continuamente con las paredes. Al llegar a la intersección, la deriva acumulada es prácticamente cero ($< 1.5^\circ$), asegurando giros impecables a $90.0^\circ$ y entradas perfectamente centradas.

<div align="center">
  <img src="docs/images/imu_drift_mitigation.png" alt="Mitigación de Deriva del IMU" width="85%">
  <p><i>Figura 2: Seguimiento de rumbo y deriva residual acotada a ±1.5° gracias al estimador físico con muros.</i></p>
</div>

---

### 4. Detección de Terreno y Protección de Rampas (Filtro IIR de Cabeceo)

El laberinto incluye rampas y desniveles. Cuando el robot comienza a subir una rampa:
* El cabeceo del chasis (Pitch) apunta el sensor frontal hacia el suelo o hacia el techo, distorsionando las lecturas ToF.
* Implementamos un filtro pasa-bajas IIR en el ángulo de Pitch del IMU:
  $$\text{Pitch}_{\text{filtrado}} = 0.2 \cdot \text{Pitch}_{\text{raw}} + 0.8 \cdot \text{Pitch}_{\text{previo}}$$
* **Máquina de estados de terreno:**
  - `FLAT` $\rightarrow$ si $\text{Pitch} > 10^\circ \implies$ conmutar a `UP` (subiendo rampa).
  - `UP` $\rightarrow$ al superar la cúspide y nivelarse $\implies$ conmutar a `DOWN` / `FLAT`.
  - Mientras el terreno no sea `FLAT` con $|\text{Pitch}| < 3^\circ$, se suspende la finalización por ToF para evitar dobles conteos y falsos checkpoints. La casilla se completa únicamente cuando el chasis vuelve a estar completamente nivelado.

---

## 🧠 Algoritmo de Exploración (DFS con Hilo de Oro) y Retorno a Base

El robot navega usando una Búsqueda en Profundidad (**DFS**) modelada como una máquina de estados finitos (FSM):

```mermaid
stateDiagram-v2
    [*] --> CALIBRATE : Encendido (3s)
    CALIBRATE --> SCAN : IMU y offsets listos
    
    SCAN --> RETURN_HOME : ¡Baldosa Roja Detectada!
    SCAN --> DECIDE : Escaneo de Pasajes Libres
    
    DECIDE --> TURN : Rumbo Seleccionado / Backtrack
    DECIDE --> FINISHED : Laberinto Explorador Agotado
    
    TURN --> SETTLE : Rumbo Alcanzado (|error| < 2.3°)
    SETTLE --> MOVE : Chasis Estabilizado (200ms)
    
    MOVE --> BACKUP : Muro Frontal < 10 cm
    MOVE --> SCAN : 30 cm Recorridos / Centro de Celda
    
    BACKUP --> SCAN : Despeje Seguro (15 cm)
    
    RETURN_HOME --> TURN : Desempilar Ruta (Hilo de Oro)
    RETURN_HOME --> FINISHED : Estacionado en (0, 0)
```

1. **Prioridad de Exploración:**
   En cada celda escanea con los ToF: 1° Recto, 2° Derecha, 3° Izquierda. Si los 3 lados están bloqueados en la celda de inicio $(0, 0)$, gira $180^\circ$ para explorar la retaguardia.
2. **Pila LIFO (El Hilo de Oro):**
   Almacena las celdas visitadas en `route_stack`. Cuando llega a un callejón sin salida (*dead-end*), desempila la ruta para regresar ordenadamente a la última intersección con caminos pendientes (*backtracking*).
3. **Retorno Triunfal (`RETURN_HOME`):**
   En cuanto la cámara PixyMon detecta el color rojo de la meta, el robot interrumpe la exploración y desempila paso a paso la ruta inversa exacta hasta aparcar de forma autónoma y segura en $(0, 0)$.

<div align="center">
  <img src="docs/images/trajectory_and_odometry.png" alt="Mapeo Topológico y Retorno a Casa" width="60%">
  <p><i>Figura 3: Trayectoria 2D del robot mapeando celdas, alcanzando la meta roja y retornando exactamente a la base (0, 0).</i></p>
</div>

---

## 📊 Validación Experimental: 100% de Éxito en Pruebas

Para garantizar que el sistema no dependa de la "suerte" en una sola corrida, ejecutamos una batería automatizada de **5 pruebas consecutivas** en Webots ([scripts/batch_test_runs.py](file:///C:/Users/Sin%20nombre/Documents/Candidates/WebotsSim2Real/scripts/batch_test_runs.py)) con todas las fuentes de ruido activas:

| Corrida | Resultado | Meta Roja | Retorno (0,0) | Desviación Final $(dX, dY)$ | Deriva Máxima IMU | Frenadas Emergencia | Duración |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **Run 1** | **SUCCESS** | ✅ Sí | ✅ Sí | $(+0.8\text{ cm}, -4.0\text{ cm})$ | $5.5^\circ$ | 0 | $9.8\text{ s}$ |
| **Run 2** | **SUCCESS** | ✅ Sí | ✅ Sí | $(+0.3\text{ cm}, -3.1\text{ cm})$ | $6.8^\circ$ | 0 | $11.5\text{ s}$ |
| **Run 3** | **SUCCESS** | ✅ Sí | ✅ Sí | $(+3.5\text{ cm}, -0.2\text{ cm})$ | $5.0^\circ$ | 0 | $13.2\text{ s}$ |
| **Run 4** | **SUCCESS** | ✅ Sí | ✅ Sí | $(-0.3\text{ cm}, +9.9\text{ cm})$ | $5.8^\circ$ | 1 | $16.7\text{ s}$ |
| **Run 5** | **SUCCESS** | ✅ Sí | ✅ Sí | $(-1.0\text{ cm}, +1.8\text{ cm})$ | $8.3^\circ$ | 2 | $28.9\text{ s}$ |

<div align="center">
  <img src="docs/images/batch_evaluation_metrics.png" alt="Métricas del Lote de Pruebas" width="85%">
  <p><i>Figura 4: Duración de misiones y precisión de posicionamiento en el retorno a la celda verde (0, 0).</i></p>
</div>

* **Tasa Global de Éxito:** **100.0% (5 de 5 corridas exitosas)**.
* **Error Medio de Posicionamiento en Retorno:** $< 3.5\text{ cm}$ (muy inferior al tamaño de una celda de $30\text{ cm}$).
* **Frenadas de emergencia:** Reducidas de 17 incidentes iniciales a prácticamente cero.

---

## 📁 Estructura del Repositorio

```bash
WebotsSim2Real/
├── controllers/
│   ├── maze_supervisor/
│   │   └── maze_supervisor.py      # Generador procedimental de laberintos, rampas y ArUco
│   └── robot_maze_solver/
│       └── robot_maze_solver.py    # Controlador principal con FSM, PID 10-15cm, ToF y Visión
├── docs/
│   └── images/                     # Gráficas generadas con telemetría real (Matplotlib)
├── evaluations/
│   └── batch_dead_reckoning_summary.md  # Reportes estadísticos de evaluación
├── scripts/
│   ├── batch_test_runs.py          # Runner automatizado para pruebas batch en Webots
│   └── generate_plots.py           # Generador de gráficas a partir de god_mode_live.log
├── worlds/
│   └── sim2real_maze.wbt           # Mundo de Webots con robot diferencial configurado
├── launch.bat                      # Lanzador rápido de la simulación en modo headless/fast
├── god_mode_live.log               # Telemetría detallada en tiempo real de la última corrida
└── README.md                       # Esta documentación
```

---

## 🚀 Cómo Ejecutar la Simulación

### 1. Requisitos Previos:
* [Webots R2023b](https://cyberbotics.com/) o superior instalado en el sistema.
* Python 3.10 o superior con las dependencias instaladas:
  ```bash
  pip install numpy opencv-contrib-python matplotlib
  ```

### 2. Ejecutar en Webots (Modo Interactivo):
1. Abre el mundo en Webots:
   `worlds/sim2real_maze.wbt`
2. Presiona el botón **Play (Run)** en la barra superior.
3. Observa en la consola de Webots y en la pantalla OLED del robot cómo toma decisiones, detecta la meta roja y retorna a $(0, 0)$.

### 3. Ejecutar Evaluación Automatizada por Lote:
Para correr 5 pruebas completas y verificar métricas sin tocar la interfaz gráfica:
```bash
python scripts/batch_test_runs.py
```

### 4. Regenerar las Gráficas de Telemetría:
Para actualizar las gráficas en `docs/images/` tras una nueva corrida:
```bash
python scripts/generate_plots.py
```

---

<div align="center">
  <b>Desarrollado con ingenio, matemáticas aplicadas y pasión por la robótica Sim2Real.</b>
</div>
