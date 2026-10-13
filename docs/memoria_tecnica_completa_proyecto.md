# 🏛️ Memoria Técnica Integral del Proyecto: Webots Maze Simulation Architect & Sim2Real
## Archivo Histórico de Contexto Completo: Candidates 2026 (Enigma Seteki)

> **ID de Conversación Histórica:** `dea9335a-3a67-47bc-96b3-6c44f2892599`  
> **Título Original:** *Webots Maze Simulation Architect*  
> **Fecha de Consolidación:** Octubre 2026  
> **Propósito:** Registro completo y exhaustivo de todos los requerimientos, experimentos, derivaciones matemáticas, decisiones arquitectónicas, código desarrollado y problemas resueltos a lo largo de las 468 interacciones del proyecto.

---

## 📑 Índice General
1. [Visión General y Metodología Sim2Real](#1-visión-general-y-metodología-sim2real)
2. [Pista A: Laberinto Ortogonal (MAZE 5x5)](#2-pista-a-laberinto-ortogonal-maze-5x5)
   - Generación Procedural y Supervisors en Webots
   - Algoritmo de Exploración DFS y Resolución de Callejones
   - Modelado Físico del Ruido (IMU MPU-6050 y ToFs Láser)
   - Odometría sin Encoders: Dead-Reckoning a 7.8V + ToF Frontal
   - Estimador de Muros $\theta_{\text{walls}}$ y Autocalibración del Giroscopio
   - Detección de Marcadores ArUco con OpenCV y Persistencia de 3.5s
3. [Pista B: Desniveles, Rampas y Recolección](#3-pista-b-desniveles-rampas-y-recolección)
   - Sección 1: Cuadrícula Inicial y Captura de Pelota
   - Sección 2: Evasión Reactiva de Líneas Blancas con Filtro HSV
   - Sección 3: Laberinto de Colores y Detección de Baldosas
   - Filtro Complementario de Cabeceo (Pitch en Y) y Detección de Terreno
   - Mecanismo Basculante de Pelotas (Micro-servo SG90)
4. [Arquitectura de Hardware y Electrónica Física](#4-arquitectura-de-hardware-y-electrónica-física)
   - Aislamiento Galvánico con Doble LM2596 (7.8V vs 5.0V)
   - Comunicación Serial UART Simplex sin Convertidor de Nivel (3.3V a 5.0V)
   - Gestión del Bus I2C con Reasignación de Direcciones por XSHUT
   - Selección de Componentes: DRV8833 vs L298N, ToFs vs Ultrasonidos
5. [Capa de Abstracción de Hardware (HAL) y Software Embebido](#5-capa-de-abstracción-de-hardware-hal-y-software-embebido)
   - Arquitectura Multihilo en Orange Pi Zero 2W (`robot_hal.py`)
   - Firmware de Tracción C++ en Arduino Uno con Watchdog Failsafe (300 ms)
   - Interfaz Gráfica OLED SSD1306 y Pulsadores Físicos (`menu.py`)
   - Orquestador de Competencia FSM (`main.py`)
6. [Registro de Planes Técnicos y Hitos de la Conversación Histórica](#6-registro-de-planes-técnicos-y-hitos-de-la-conversación-histórica)
   - Plan de la Semana 6 (Integración Física y HAL)
   - Plan de Limpieza y Puesta a Punto Final

---

## 1. Visión General y Metodología Sim2Real

El proyecto nació con una premisa de ingeniería honesta y rigurosa: **construir un robot autónomo para Candidates Principiantes 2026 maximizando el software, la física y las matemáticas antes de tocar hardware físico**, evitando gastar presupuesto en sensores industriales o encoders de rueda caros que en pista real sufren de patinaje (*wheel-slip*).

### La Filosofía Sim2Real:
1. Se construyó un **gemelo digital en Webots R2023b** que modela con precisión matemática las imperfecciones del mundo real:
   - Deriva estocástica del sensor inercial GY-521 (MPU-6050) debida a temperatura y vibraciones.
   - Ruido de medición en los sensores láser de tiempo de vuelo (Pololu VL53L1X y VL53L0X).
   - Asimetría de potencia entre los motores N20 y micropatinaje de neumáticos sobre el suelo.
2. Cada algoritmo de navegación, control lateral y fusión sensorial fue sometido a **baterías de 50 evaluaciones consecutivas con semillas aleatorias** en Webots en modo *fast* sin renderizado gráfico, logrando una tasa de éxito del 100% antes de compilarse en la computadora de abordo física.

---

## 2. Pista A: Laberinto Ortogonal (MAZE 5x5)

### Generación Procedural y Supervisor
* **Dimensiones:** Cuadrícula de 5x5 celdas ($30\text{ cm} \times 30\text{ cm}$), área total de $1.50\text{ m} \times 1.50\text{ m}$, paredes de 15 cm de altura y 1 cm de espesor.
* **Supervisor Webots (`maze_supervisor.py`):** Genera en tiempo de ejecución laberintos perfectos utilizando el algoritmo **Recursive Backtracker (DFS)**, inyectando nodos VRML dinámicamente.
* **Elementos en Pista:**
  - Casilla de inicio verde `(0, 0)`.
  - Casilla de meta roja.
  - 4 baldosas de color de bonificación: Cyan, Amarillo, Naranja y Magenta.
  - Marcadores ArUco (`DICT_4X4_50`) adheridos a paredes verticales.

### Algoritmo de Exploración DFS
* Modela el laberinto como un grafo conectado.
* **Criterio de Pared con ToFs:**
  $$\text{Si } d_{\text{ToF}} > 0.28\text{ m} \implies \text{Pasaje Abierto (Vecino Válido)}$$
  $$\text{Si } d_{\text{ToF}} \le 0.28\text{ m} \implies \text{Pared Detectada (Camino Bloqueado)}$$
  *(Margen de seguridad $> 30\text{ cm}$ que elimina falsos positivos).*
* **Prioridad Determinista:** Frente $\rightarrow$ Derecha $\rightarrow$ Izquierda $\rightarrow$ Retaguardia (180°).
* **Backtracking en Callejones sin Salida:** Extrae la casilla de la pila (`route_stack.pop()`), calcula el ángulo hacia el nodo padre y retrocede automáticamente sin perderse.

### Modelado de Ruido y Deriva Inercial
* En Webots se simuló el comportamiento real del MPU-6050:
  ```python
  GYRO_BIAS = 0.002              # Sesgo estático
  GYRO_RANDOM_WALK_STD = 0.00004 # Paseo aleatorio térmico (drift continuo)
  ```
* En 60 segundos de carrera, este ruido acumula entre $15^\circ$ y $25^\circ$ de error angular si no se compensa.

### Odometría sin Encoders: Dead-Reckoning a 7.8V + ToF Frontal
* Motores N20 (100:1): Tensión estabilizada por LM2596 a $7.8\text{ V}$.
  $$\text{RPM}_{7.8\text{V}} = 260.0 \times \left(\frac{7.8\text{ V}}{6.0\text{ V}}\right) = 338.0\text{ RPM}$$
  $$\omega_{\text{max}} = 338.0 \times \frac{2\pi}{60} = 35.3953\text{ rad/s}$$
* Con ruedas de radio $R = 0.02\text{ m}$ y $\omega_{\text{crucero}} = 4.2\text{ rad/s}$:
  $$v_{\text{lin}} = \omega_{\text{crucero}} \times R = 0.084\text{ m/s} \quad (8.4\text{ cm/s})$$
  $$T_{\text{celda}} = \frac{0.30\text{ m}}{0.084\text{ m/s}} \approx 3.571\text{ s}$$
* **Reseteo Óptico Frontal:** Al aproximarse a una pared frontal ($< 40\text{ cm}$), el sensor láser VL53L1X sustituye la estimación por tiempo por la distancia absoluta milimétrica, cancelando el micropatinaje de llantas.

### Estimador Físico de Ángulo con Muros ($\theta_{\text{walls}}$)
Aprovechando que los muros del laberinto son estrictamente ortogonales ($0^\circ, 90^\circ, 180^\circ, 270^\circ$):
1. **Métrica Lateral $m(t)$:**
   $$m(t) = \begin{cases} \frac{d_L - d_R}{2} & \text{si hay 2 paredes} \\ d_L & \text{si hay pared izq} \\ -d_R & \text{si hay pared der} \end{cases}$$
2. **Ángulo Físico Real:**
   $$\theta_{\text{walls}} = -\frac{1}{v} \frac{dm}{dt} = -\frac{m(t) - m(t - \Delta t)}{\Delta s}$$
3. **Deriva Instantánea del IMU:**
   $$\epsilon_{\text{deriva}} = \mathrm{wrap}(\psi_{\text{imu}} - \psi_{\text{meta}} - \theta_{\text{walls}})$$
4. **Compensación Dinámica en Vivo:**
   $$\psi_{\text{offset}} \leftarrow \psi_{\text{offset}} + 0.035 \cdot \epsilon_{\text{deriva}}$$
* Al llegar a cada esquina, la deriva angular es inferior a $\pm 1.5^\circ$, garantizando giros a $90.0^\circ$ exactos.

### Reconocimiento ArUco con Persistencia
* Hilo asíncrono con `cv2.aruco.ArucoDetector` (`DICT_4X4_50`) a 30 FPS.
* Al detectar un marcador en la pared, fija un temporizador `t_clear = t + 3.5s`, garantizando que el ID permanezca en la pantalla OLED por **más de 3 segundos continuos** como exige el reglamento.

---

## 3. Pista B: Desniveles, Rampas y Recolección

* **Supervisor Procedural (`track_b_supervisor.py`):** Genera de forma aleatoria la configuración de obstáculos y baldosas de la Pista B.

### Sección 1: Cuadrícula y Trampas
* Área inicial con trampas modulares en 4 orientaciones posibles.
* El robot patrulla perimetralmente, identifica el hueco abierto con el sensor ToF frontal ($d_{\text{front}} > 0.35\text{ m}$), ingresa, retiene la pelota con su compuerta mecánica, retrocede con seguridad y sale al Checkpoint 1 `(3, 1)`.

### Sección 2: Evasión Reactiva Universal de Líneas Blancas
* Pasillo verde de 60 cm de ancho con 3 barreras transversales, cada una con un paso abierto de 30 cm en uno de los 3 carriles ($Y = 0.00\text{ m}, +0.15\text{ m}, +0.30\text{ m}$).
* **Aislamiento Geométrico de ROI:**
  ```python
  roi_white = hsv_floor[50:100, 34:94] # Filas 50-100 (inmune a barreras lejanas)
                                       # Cols 34-94 (ancho exacto del chasis)
  ```
* **Lógica Reactiva:**
  - Si el carril actual no tiene línea blanca enfrente ($< 80\text{ px}$): continúa en línea recta.
  - Si detecta línea blanca adelante: retrocede a $\ge 18\text{ cm}$, gira $90^\circ$, se traslada al carril adyacente, se reorienta al Este ($0^\circ$) y cruza a velocidad baja centrado entre paredes.

### Sección 3: Laberinto de Colores y Detección de Suelo
* En cada casilla el robot muestrea la región central del piso (`roi_floor = hsv_floor[75:102, 35:93]`).
* Determina el color dominante, calcula el nuevo rumbo absoluto, gira sobre su eje ($|\Delta\theta| \le 0.05\text{ rad}$) y avanza a la siguiente casilla.
* Al detectar la baldosa verde `FIN`, aplica frenado inmediato y despliega el mensaje de victoria en pantalla.

### Filtro Complementario de Cabeceo (Pitch en Y)
* Sensor MPU-6050 montado con eje Y alineado a la dirección de avance:
  $$\theta_{\text{acc}} = \text{atan2}(A_y, \sqrt{A_x^2 + A_z^2}) \cdot \left(\frac{180}{\pi}\right)$$
  $$\theta_{\text{pitch}}(k) = 0.96 \cdot (\theta_{\text{pitch}}(k-1) + \omega_x \cdot \Delta t) + 0.04 \cdot \theta_{\text{acc}}$$
* **Máquina de Estados de Terreno:**
  - $\theta_{\text{pitch}} > +6.0^\circ$: `SUBIDA` (aumenta PWM a 210 para superar la pendiente).
  - $\theta_{\text{pitch}} < -6.0^\circ$: `BAJADA` (reduce PWM a 120 aplicando contramarcha suave).
  - $-6.0^\circ \le \theta_{\text{pitch}} \le +6.0^\circ$: `PLANO` (velocidad nominal 160).
  - Mientras el terreno no esté nivelado ($|\text{Pitch}| > 3^\circ$), se suspende el conteo de casillas para evitar falsos positivos por inclinación óptica de los sensores ToF.

---

## 4. Arquitectura de Hardware y Electrónica Física

### Doble Línea de Alimentación Desacoplada:
* Batería LiPo 3S (11.1V nominal).
* **Línea de Potencia (LM2596 #1 a 7.8V):** Alimenta exclusivamente el driver DRV8833 y los micromotores N20. Aísla caídas de tensión (*voltage sags*) y picos inductivos.
* **Línea Lógica (LM2596 #2 a 5.0V):** Alimenta la Orange Pi Zero 2W y el Arduino Uno R3 con filtrado capacitivo limpio.
* **Topología de Tierra en Estrella:** Todas las masas confluyen en un único nodo central negativo, eliminando bucles de tierra y pérdida de sincronismo en UART.

### Comunicación Serial UART Simplex (3.3V -> 5.0V):
* Pin TX de la Orange Pi (`PH2`, 3.3V) conectado directamente al pin RX del Arduino Uno (Pin 0, 5.0V).
* **Justificación Matemática sin Convertidor de Nivel:**
  $$V_{IH} = 0.6 \times 5.0\text{ V} = 3.0\text{ V}$$
  $$3.3\text{ V} > 3.0\text{ V} \implies \text{Nivel ALTO garantizado y estable}$$
* La comunicación es estrictamente unidireccional: la Orange Pi transmite tramas `M,vel_L,vel_R,servo\n` y el Arduino ejecuta. El pin RX de la Orange Pi está desconectado, eliminando cualquier peligro de sobrevoltaje.

### Bus I2C y Pines XSHUT:
* Bus nativo `/dev/i2c-2` a 400 kHz:
  - IMU MPU-6050 (`0x68`)
  - Pantalla OLED SSD1306 (`0x3C`)
  - Sensor Frontal VL53L1X (`0x29`)
* **Secuencia de Arranque XSHUT:**
  - Pone en bajo los pines XSHUT de ambos sensores laterales VL53L0X.
  - Enciende el lateral izquierdo y le reasigna por software la dirección `0x30`.
  - Enciende el lateral derecho y le reasigna la dirección `0x31`.
  - Deja el frontal en `0x29`. Los 3 sensores operan simultáneamente en el mismo bus sin conflicto.

### Driver DRV8833 vs L298N:
* El L298N tiene una caída interna de transistores bipolares de hasta $2.0\text{ V}$ (desperdicia casi el 25% de la energía de la batería en calor masivo).
* El DRV8833 utiliza puentes H de transistores MOSFET con resistencia de conducción mínima ($R_{DS(on)} \approx 0.2\ \Omega$) y caída menor a $0.1\text{ V}$, entregando torque completo a los motores.

---

## 5. Capa de Abstracción de Hardware (HAL) y Software Embebido

### Capa HAL Multihilo (`src/orange/robot_hal.py`)
Distribuida sobre los 4 núcleos ARM Cortex-A53 de la Orange Pi:
* **Hilo I2C:** Lee ToFs a 30 Hz e integra el giróscopo MPU-6050 a 100 Hz.
* **Hilo Serial:** Envía paquetes de consigna de velocidad hacia el Arduino Uno a 50 Hz.
* **Hilo de Visión Frontal:** Detección de marcadores ArUco con OpenCV a 30 FPS.
* **Hilo SPI:** Lectura de firmas de color de la cámara PixyCam v2.

### Firmware Arduino Uno (`src/arduino/firmware_motor_controller.ino`)
* Control de PWM en pines D3, D5, D6, D11 (Timer 0 y Timer 2).
* El pin D9 (Timer 1) queda reservado para la compuerta de pelotas con el micro-servo SG90, evitando interferencias de timers del ATmega328P.
* **Watchdog de Seguridad por Hardware (300 ms):** Si transcurren más de 300 ms sin recibir una trama serial válida de la Orange Pi, el firmware corta inmediatamente la energía a ambos motores para evitar colisiones accidentales.

### Interfaz OLED SSD1306 y Menú en Pits (`src/orange/menu.py`)
* Operación autónoma en el campo de juego mediante 2 pulsadores GPIO leídos con `gpiod`:
  - **Pulsador Navegar (Pin 35):** Navega entre Pista A, Pista B y Test de Sensores.
  - **Pulsador Seleccionar (Pin 37):** Confirma e inicia cuenta regresiva de 3 segundos.
  - **Parada de Emergencia:** Cualquier pulsación física durante la carrera detiene los motores en seco en menos de 20 ms.
* Pantalla de telemetría en tiempo real: Rumbo Yaw, cabeceo Pitch, detección dinámica de rampa, lecturas ToF y estado de la FSM.

---

## 6. Registro de Planes Técnicos y Hitos de la Conversación Histórica

### Plan de la Semana 6 (Integración Física y HAL)
# Plan de Implementación - Semana 6: HAL, Firmware de Control y Menú OLED

Este plan describe la arquitectura y desarrollo técnico para la **Semana 6** del proyecto Candidates Principiantes 2026. Integra la capa de abstracción de hardware (HAL) en la Orange Pi Zero 2W, el firmware en C++ para el microcontrolador Arduino Uno R3, y el sistema de navegación por menú en la pantalla OLED para operación autónoma en competencia sin requerir computadora externa.

---

## 1. Descripción de Objetivos

1. **Firmware de Tracción y Actuación (Arduino Uno R3):**
   * Control bidireccional por PWM de dos micromotores N20 utilizando el módulo puente H doble **DRV8833**.
   * Control de compuerta basculante mediante un micro-servo **SG90**.
   * Protocolo de comunicación serial por UART a 115200 baudios con comando ASCII `M,vel_L,vel_R,servo\n`.
   * Temporizador de seguridad por hardware/software (*Watchdog Failsafe*) de 300 ms para detener los motores si se interrumpe la comunicación.
   * Prevención del conflicto crítico de timers en ATmega328P: la librería `Servo.h` se apropia del Timer 1 (inhabilitando PWM en los pines D9 y D10), por lo que el driver DRV8833 se mapea a los pines D3, D5, D6 y D11 (Timer 0 y Timer 2).

2. **Capa de Abstracción de Hardware (HAL - `robot_hal.py`):**
   * Encapsular el acceso a todos los periféricos físicos validados en la Semana 5 en una sola clase limpia (`RobotHAL`).
   * Secuencia de inicialización de los tres sensores ToF mediante pulsos en sus líneas XSHUT para reasignar direcciones I2C (`0x30` y `0x31` para los laterales VL53L0X, y `0x29` para el frontal VL53L1X).
   * Hilos asíncronos en segundo plano (multithreading sobre los 4 núcleos de la Orange Pi):
     * Hilo de sensores I2C (3x ToF a 30 Hz y MPU-6050 integrando rumbo angular a 100 Hz).
     * Hilo de visión USB para detección de marcadores ArUco `DICT_4X4_50` a 30 FPS con filtro de estabilidad temporal (3 cuadros).
     * Hilo SPI para la PixyCam v2 leyendo bloques de color y pelota.
     * Hilo de enlace serial UART (`/dev/ttyS5`) enviando comandos periódicos a 50 Hz.
   * Exponer métodos idénticos a los probados en el simulador Webots (`get_tof_distances()`, `get_yaw_deg()`, `set_motor_speeds()`, `set_servo_angle()`, `get_aruco_id()`, `stop_all()`).

3. **Menú de Selección y Telemetría en OLED (`menu.py` y `main.py`):**
   * Control de la pantalla OLED SSD1306 (128x64) mediante I2C (`0x3C`) usando `luma.oled` y `Pillow`.
   * Interacción mediante dos pulsadores físicos (Navegar y Seleccionar) leídos con `gpiod`.
   * Máquina de estados del menú:
     * **Selección de Modo:** Pista A, Pista B, Test de Sensores, Calibrar IMU.
     * **Cuenta Regresiva:** 3 segundos de preparación antes de liberar el robot.
     * **Telemetría en Vivo:** Muestra ToF Izq/Fte/Der, ángulo Yaw, ID de ArUco o pelota, y estado de la FSM durante la carrera.
     * **Parada de Emergencia:** Abortar la rutina y detener motores al presionar cualquier botón durante la ejecución.

---

## 2. Decisiones de Diseño Acordadas en Grill-Me

* **Driver de Motores:** Sustitución del chip TB6612FNG por el driver **DRV8833**. Mapeo de pines PWM en D3, D5, D6, D11 para evitar choques con el Timer 1 del servo SG90 en D9.
* **Control de Menú:** Dos pulsadores táctiles (Navegar y Seleccionar) conectados a pines GPIO libres de la Orange Pi con pull-up interno mediante `gpiod`.
* **Protocolo Serial:** Formato ASCII `M,vel_L,vel_R,servo\n` con rango de PWM de -255 a +255, ángulo de 0° a 180°, Watchdog de parada a los 300 ms y respuesta de eco corta `OK\n`.
* **Flujo de Ejecución:** Cuenta regresiva de 3 segundos en pantalla antes de iniciar la misión, telemetría continua a 5-10 Hz en OLED mientras navega, y botón de parada de emergencia activo en todo momento.

---

## 3. Cambios Propuestos

```
Candidates/
├── arduino/
│   └── firmware_motor_controller/
│       └── firmware_motor_controller.ino     [NUEVO] Firmware C++ para Arduino Uno R3
├── orange/
│   ├── robot_hal.py                          [NUEVO] Hardware Abstraction Layer
│   ├── menu.py                               [NUEVO] Interfaz gráfica OLED y manejo de botones
│   ├── main.py                               [NUEVO] Entrypoint principal y orquestador
│   ├── test.py                               [EXISTENTE] Servidor de pruebas unitarias
│   ├── sync.bat                              [MODIFICAR] Añadir soporte para sincronizar todo el árbol
│   └── sync.ps1                              [MODIFICAR] Añadir soporte para sincronizar todo el árbol
└── bitacoras/
    └── bitacora_semana_6.md                  [NUEVO] Documentación técnica de la Semana 6
```

---

### Componente 1: Firmware en C++ (`firmware_motor_controller.ino`)

#### [NEW] `arduino/firmware_motor_controller/firmware_motor_controller.ino`
* **Pines configurados:**
  * Motor A (Izquierdo): `IN1 = 5` (PWM), `IN2 = 6` (PWM)
  * Motor B (Derecho): `IN3 = 3` (PWM), `IN4 = 11` (PWM)
  * Servo SG90: `SERVO_PIN = 9` (Controlado por `Servo.h`)
  * DRV8833 Standby/Sleep: `SLEEP_PIN = 4` (Activado en HIGH en `setup()`)
* **Lógica del Watchdog:**
  * Variable `last_packet_time`. Si `millis() - last_packet_time > 300`, apaga ambas salidas (`analogWrite` a 0 en todos los pines).
* **Control de motores en DRV8833:**
  * Para girar hacia adelante: `analogWrite(IN1, pwm); digitalWrite(IN2, LOW);`
  * Para girar hacia atrás: `digitalWrite(IN1, LOW); analogWrite(IN2, pwm);`
  * Para frenar: `digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);`
* **Parser Serial:**
  * Lectura no bloqueante por buffer hasta encontrar el salto de línea `\n`.
  * Validación de prefijo `'M'` y separación por comas para extraer `vel_left`, `vel_right` y `servo_pos`.

---

### Componente 2: Capa de Abstracción de Hardware (`robot_hal.py`)

#### [NEW] `orange/robot_hal.py`
* **Estructura de la clase `RobotHAL`:**
  * Configuración centralizada de hardware (buses, pines XSHUT, dirección I2C).
  * **Método `init_tof_sensors()`:**
    1. Coloca en LOW las 3 líneas XSHUT apagando todos los ToF.
    2. Espera 10 ms para descargar capacitores.
    3. Habilita XSHUT del ToF lateral izquierdo -> instancia `VL53L0X()` -> llama `change_address(0x30)` -> `start_ranging()`.
    4. Habilita XSHUT del ToF lateral derecho -> instancia `VL53L0X()` -> llama `change_address(0x31)` -> `start_ranging()`.
    5. Habilita XSHUT del ToF frontal -> instancia `vl53l1x.VL53L1X()` en dirección nativa `0x29` -> `start_ranging(1)`.
  * **Hilo de MPU-6050 e Integración de Rumbo:**
    * Bucle a 100 Hz leyendo el eje Z del giróscopo (`gyro_z`).
    * Calibración inicial automática de 200 muestras para calcular el offset de reposo (*gyro bias*).
    * Integración trapezoidal de velocidad angular con respecto a `delta_t` para mantener un ángulo `current_yaw_deg` acumulado.
  * **Hilo de Visión ArUco:**
    * Abre `/dev/video1` en resolución 640x480.
    * Convierte a escala de grises y aplica `detectMarkers()` con diccionario `DICT_4X4_50` y parámetros optimizados.
    * Filtro de confirmación de 3 cuadros consecutivos para descartar falsos positivos.
  * **Hilo de PixyCam v2 (SPI):**
    * Lee paquetes de bloques v2 a 2 MHz desde `/dev/spidev1.0`.
    * Almacena coordenadas de la pelota de golf y firmas de color de baldosas.
  * **Hilo de Comunicación Serial con Arduino:**
    * Apertura de `/dev/ttyS5` a 115200 baudios con timeout de 0.05 s.
    * Transmisión continua a 50 Hz del último comando de motores y posición de compuerta solicitado.
  * **Métodos Públicos:**
    * `get_tof_distances()` -> retorna tupla `(dist_front_cm, dist_left_cm, dist_right_cm)`.
    * `get_yaw_deg()` -> retorna flotante del rumbo actual en grados.
    * `set_motor_speeds(v_left, v_right)` -> traduce rad/s o porcentaje a valores de PWM (-255 a +255).
    * `set_servo_angle(angle_deg)` -> fija el ángulo de la compuerta basculante.
    * `get_aruco_id()` -> retorna el ID confirmado actual o `None`.
    * `get_pixy_blocks()` -> retorna lista de bloques detectados.
    * `stop_all()` -> apaga de inmediato la tracción y envía PWM = 0 al Arduino.

---

### Componente 3: Menú Interactivo en Pantalla OLED (`menu.py` y `main.py`)

#### [NEW] `orange/menu.py`
* **Driver OLED:**
  * Uso de `luma.oled.device.ssd1306` sobre `luma.core.interface.serial.i2c` en bus `/dev/i2c-1` (dirección `0x3C`).
  * Dibujo de primitivas gráficas con `PIL.ImageDraw` (cajas de selección invertidas, barras de nivel, texto con fuente TrueType/bitmap).
* **Manejo de Botones:**
  * Lectura de pines GPIO configurados como entradas con resistencia pull-up activada mediante `gpiod`.
  * Detección de flanco de bajada con filtro de rebote (*debounce*) de 150 ms.
* **Pantallas del Menú:**
  1. `draw_menu(selected_index)`: Menú principal con cuatro opciones:
     * `[1] PISTA A (LABERINTO)`
     * `[2] PISTA B (PELOTAS)`
     * `[3] TEST SENSORES`
     * `[4] CALIBRAR IMU`
  2. `draw_countdown(seconds)`: Pantalla con número gigante central y barra de progreso.
  3. `draw_telemetry(mode_name, dist_f, dist_l, dist_r, yaw, status_text)`: Panel de instrumentos con lecturas compactas y limpias.
  4. `draw_sensor_test(sensors_data)`: Diagnóstico rápido en pits para verificar que ningún conector o cable esté flojo antes de entrar a la arena.

#### [NEW] `orange/main.py`
* Orquestador central del robot:
  1. Instancia `RobotHAL()` y `MenuManager()`.
  2. Inicializa los sensores y periféricos mostrando barra de progreso en la pantalla OLED.
  3. Ejecuta el ciclo interactivo del menú esperando interacción de los botones.
  4. Al seleccionar una opción:
     * Si es **Pista A** o **Pista B**: Ejecuta la cuenta regresiva de 3 s y transfiere el control a la rutina correspondiente, manteniendo el hilo de telemetría activo en la pantalla y vigilando el botón de emergencia.
     * Si es **Test Sensores**: Entra a un bucle continuo de refresco mostrando datos crudos de todos los componentes en la pantalla hasta que se presione Seleccionar para salir.
     * Si es **Calibrar IMU**: Mantiene el robot inmóvil durante 3 segundos, calcula el nuevo offset del giróscopo y lo guarda en memoria.

---

### Componente 4: Documentación y Bitácora

#### [NEW] `bitacoras/bitacora_semana_6.md`
* Redacción detallada de las actividades de la Semana 6 siguiendo la estructura establecida:
  * Resumen de la semana.
  * Retos y decisiones técnicas del firmware en C++ (sustitución a DRV8833 y resolución de conflicto de Timer 1 con la librería Servo).
  * Arquitectura multihilo del HAL en Python y sincronización por colas/locks.
  * Estrategia de arranque XSHUT y reasignación dinámica de direcciones I2C.
  * Diseño de la interfaz de usuario OLED para operación autónoma en competencia sin laptop.
  * Protocolo serial robusto con Watchdog y medidas de seguridad.
  * Pruebas de integración y validaciones en hardware.

---

## 4. Plan de Verificación

### Validación de Sintaxis y Análisis Estático
* Ejecutar comprobación de sintaxis de todos los scripts creados en Python mediante `python -m py_compile`.
* Validar que la compilación de C++ del sketch `.ino` cumpla las reglas del compilador AVR-GCC.

### Verificación Manual en Hardware Real
1. **Prueba de Banco del Arduino y DRV8833:**
   * Cargar el sketch en el Arduino Uno R3 desde la laptop con Arduino IDE.
   * Abrir el Monitor Serial a 115200 baudios y enviar manualmente:
     * `M,150,150,90` -> Verificar que ambos motores avancen hacia el frente y el servo se posicione a 90°.
     * `M,-150,-150,0` -> Verificar reversa y servo a 0°.
     * No enviar comandos durante 400 ms -> Verificar que el Watchdog detenga inmediatamente ambos motores.
2. **Prueba del Sistema de Sensores XSHUT en la Orange Pi:**
   * Sincronizar los scripts mediante `sync.bat`.
   * Ejecutar el script de prueba y verificar con `i2cdetect -y 1` que las direcciones `0x29`, `0x30`, `0x31`, `0x3C` y `0x68` aparezcan activas en la tabla sin errores.
3. **Prueba del Menú OLED y Botones Físicos:**
   * Arrancar `main.py` en la Orange Pi.
   * Presionar el botón "Navegar" y comprobar que el cursor salte entre las 4 opciones de forma fluida.
   * Seleccionar "Test Sensores" y observar las distancias de los 3 ToF y el ángulo del IMU en tiempo real en la pantalla.
   * Seleccionar "Pista A", observar la cuenta regresiva de 3 s y comprobar que cualquier pulsación active la parada de emergencia inmediata.


---

### Resumen de la Puesta a Punto y Limpieza
# Resumen Final: Refactorización y Limpieza Integral para Commit Limpio

## Lo que se completó

### 1. Limpieza de Imports Defensivos y Código Puro en `orange/`
Se eliminaron todos los bloques `try...except ImportError` con asignaciones `= None` y comprobaciones intermedias `if modulo is None:`, adoptando importaciones directas de hardware para la Orange Pi Zero 2W:
- [orange/menu.py](file:///c:/Users/Sin%20nombre/Documents/Candidates/orange/menu.py):
  - Importaciones directas de `luma.oled`, `Pillow` y `gpiod`.
  - Eliminados los condicionales `if i2c is not None:` y `if gpiod is not None:`.
  - Integrado diagnóstico en vivo de `Pitch` (Cabeceo) junto a `Yaw` y `ToF`.
- [orange/robot_hal.py](file:///c:/Users/Sin%20nombre/Documents/Candidates/orange/robot_hal.py):
  - Importaciones directas de `serial`, `cv2`, `smbus2`, `gpiod`, `spidev`, `VL53L0X`, `vl53l1x` y `mpu6050`.
  - Eliminadas comprobaciones redundantes.
  - Conservado cálculo de Cabeceo real en Eje Y (`pitch_deg`) y estado de rampa (`is_on_ramp()`).
- [orange/tests/test_screen.py](file:///c:/Users/Sin%20nombre/Documents/Candidates/orange/tests/test_screen.py):
  - Importaciones directas de `luma.oled` y `Pillow`.

### 2. Condensación de Comentarios Inflados
Se redujeron todos los encabezados introductorios largos (de 20-50 líneas de texto enciclopédico) a resúmenes técnicos breves y concisos de 2-4 líneas:
- [orange/main.py](file:///c:/Users/Sin%20nombre/Documents/Candidates/orange/main.py)
- [orange/robot_hal.py](file:///c:/Users/Sin%20nombre/Documents/Candidates/orange/robot_hal.py)
- [orange/menu.py](file:///c:/Users/Sin%20nombre/Documents/Candidates/orange/menu.py)
- [orange/tests/test_imu.py](file:///c:/Users/Sin%20nombre/Documents/Candidates/orange/tests/test_imu.py) y [orange/tests/test_imu_screen.py](file:///c:/Users/Sin%20nombre/Documents/Candidates/orange/tests/test_imu_screen.py)
- [arduino/firmware_motor_controller/firmware_motor_controller.ino](file:///c:/Users/Sin%20nombre/Documents/Candidates/arduino/firmware_motor_controller/firmware_motor_controller.ino)

### 3. Limpieza Integral de `WebotsSim2Real`
Se purgó la carpeta de simulación eliminando todo el material no funcional, logs temporales y scripts de análisis batch:
- **Eliminados físicamente:**
  * Carpeta de scripts de análisis: `WebotsSim2Real/scripts/`
  * Carpeta de resultados batch: `WebotsSim2Real/evaluations/`
  * Archivos temporales de logs y flags en raíz: `god_mode_live.log`, `god_mode_track_b.log`, `run1_debug.log`, `simulation_log.txt`, `target_world.txt`, `track_b_config.json`, `track_b_s1_passed.flag`, `track_b_verified.flag`.
  * Archivos de respaldo `.bak` y mallas no referenciadas pesadas en `worlds/meshes/`.
- **Código Funcional Conservado:**
  * Controladores autónomos: `controllers/robot_maze_solver/`, `controllers/robot_track_b_solver/`, `controllers/maze_supervisor/`, `controllers/track_b_supervisor/`.
  * Mundos de simulación: `worlds/sim2real_maze.wbt`, `worlds/sim2real_track_b.wbt` y texturas.
  * Documentación oficial y lanzador: `README.md`, `docs/images/`, `launch.bat`.
- **Configuración de `.gitignore`:**
  * Actualizados tanto [WebotsSim2Real/.gitignore](file:///c:/Users/Sin%20nombre/Documents/Candidates/WebotsSim2Real/.gitignore) como [.gitignore raíz](file:///c:/Users/Sin%20nombre/Documents/Candidates/.gitignore) para ignorar permanentemente logs (`*.log`), flags (`*.flag`), temporales (`*.tmp`, `*.bak`), carpetas de evaluaciones, scripts de análisis y mallas pesadas (`*.obj`).
  * Cambios preparados y listos para commit en el repositorio git de `WebotsSim2Real`.

### 4. Documentación de Bitácora
- Se actualizó [bitacoras/bitacora_semana_6.md](file:///c:/Users/Sin%20nombre/Documents/Candidates/bitacoras/bitacora_semana_6.md) documentando la resolución del bus I2C (`/dev/i2c-2`), los tests individuales, el diagnóstico del filtro complementario de cabeceo (Eje Y), la corrección de llamadas en Pillow y la refactorización final.
- Documentación normalizada sin caracteres LaTeX.

---

## Verificación
```cmd
python -m py_compile WebotsSim2Real/controllers/robot_maze_solver/robot_maze_solver.py WebotsSim2Real/controllers/robot_track_b_solver/robot_track_b_solver.py WebotsSim2Real/controllers/maze_supervisor/maze_supervisor.py WebotsSim2Real/controllers/track_b_supervisor/track_b_supervisor.py
```
**Resultado:** Código de salida `0` en todos los controladores de simulación.


---

<div align="center">
  <p><strong>Memoria Técnica Consolidada</strong> — <i>Proyecto 3nIgMa S3t3ki | Candidates 2026</i></p>
</div>
